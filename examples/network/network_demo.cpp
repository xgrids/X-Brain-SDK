/**
 * @file   network_demo.cpp
 * @brief  XGRIDS Robot SDK 示例：修改设备对外网口(eth1)的 IP / 网段
 *
 * 流程：在「设备连接IP」上 setNetwork -> 关闭连接 -> 窗口内连接「确认地址」
 *   -> confirmNetwork。窗口内没确认成功，设备自动恢复 eth1 的旧配置。
 *
 * 运行：
 *   ./network_demo <设备连接IP> <新IP> <前缀长度> [网关|keep|none] [确认窗口秒]
 *                  [--confirm-via <确认用的设备地址>]
 *   ./network_demo 192.168.123.103 192.168.123.150 24            # 经
 * eth1，同网段改 IP
 *   ./network_demo 192.168.123.103 172.16.5.20 24 172.16.5.1 90  # 经
 * eth1，换网段
 *   ./network_demo 192.168.55.1 192.168.123.150 24 --confirm-via 192.168.55.1
 *                                                                # 经 USB
 * 网卡改 eth1
 *
 * 前置条件与副作用（本示例会真实修改设备网络，本轮板端验证未执行）：
 *   - 设备须处于待机：正在扫描或建图时设备拒绝（Rejected）。不会 startRecord。
 *   - 修改的是 eth1 的 IP / 网段，确认窗口内没确认，设备自动恢复旧配置；
 *     确认成功后永久生效，之后要改回需再跑一次本示例。
 *   - 经 eth1 本身改地址时连接会断，须在窗口内用新 IP（或 --confirm-via 指定的
 *     另一网卡上的设备地址）确认。
 *   - 不要在没有第二条通路（如 USB 网卡 192.168.55.1）时对唯一连接的网口做跨网段修改。
 *   - 退出码：0 已确认；1 参数或连接失败；2 设备未修改；3 未确认；4 连接地址都连不上。
 *
 * 预期输出（依据代码推导，未上板实测）：
 *   连接设备 <IP> ... -> 请求修改 eth1: <新IP>/<前缀>, 网关=..., 确认窗口=N 秒
 *   -> 确认方式: 经 <地址> 发送确认 -> 设备已接受修改, 在 N 秒窗口内连接 <地址> 并确认 ...
 *   -> （确认地址未就绪时重复打印 "<地址> 暂不可达, 重试 ..."）-> 已连上 <地址>, 发送确认 ...
 *   -> 确认成功: eth1 已永久改为 <新IP>/<前缀>；退出码 0。
 *   经 --confirm-via 确认时，成功后会再探测新 IP 并打印"参考"行（仅供参考，不影响退出码）。
 *
 * 失败现象与处理（依据代码推导，未上板实测）：
 *   - "连接失败: ..."（退出码 1）：检查网线、网段、设备应用是否已起来。
 *   - "设备未做修改: Rejected"（退出码 2）：正在扫描或建图、已有变更待确认、
 *     新网段与设备其他网口（含 USB 网卡）重叠、保持网关但现有网关不在新网段内；
 *     先 stop 当前任务或换网段，详情看设备日志。InvalidArg / Protocol 同样表示设备未改。
 *   - "设备未应答, 结果未知"（Timeout）：设备可能已切换地址，示例会继续尝试确认，
 *     不要手动重复 setNetwork。
 *   - "窗口内未能连上 <地址>, 设备会把 eth1 恢复为旧配置"：确认窗口内本机到不了
 *     确认地址（常见为本机网卡没有新网段的地址，见下方 ip addr add）；设备自动回滚，
 *     原地址探测可达时退出码 3，全都连不上时退出码 4。
 *   - "窗口内设备一直没有接受确认"（退出码 3）：变更可能应用失败或已回滚，保持旧配置，看设备日志。
 *   - "确认失败: ..."（退出码 3）：确认命令返回了非 Ok/Rejected/Timeout 的错误，
 *     本示例不重试，先核对设备当前 eth1 地址。
 *
 * 调用顺序与每步为什么：先 setNetwork（设备记下新配置并开始计时），再关闭旧连接
 * （地址即将变化），在窗口内重新连接确认地址并 confirmNetwork，成功才算生效。
 *
 * 线程模型：同步调用，无订阅回调。
 *
 * 网关：省略或 keep 保持现有网关，none 删除默认路由，其他为新网关。
 * 确认窗口：省略或 0 为默认 60 秒，否则 10~120。
 * --confirm-via：省略时经新 IP 确认（本机须能到达新网段，同时证明新 IP 可达）；
 *   指定 USB 等其他网卡上的设备地址时，本机不必到达新网段，但只证明设备在线。
 * SDK 不修改本机网络：经新 IP 确认且新网段与本机网卡不同网段时，先给网卡加
 * 辅助地址，如 sudo ip addr add 172.16.5.10/24 dev <网卡>。
 */

#include "xg/robot_sdk.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr std::uint32_t kProbeConnectTimeoutMs = 1500;

// 退出码
constexpr int kExitConfirmed = 0;
constexpr int kExitBadArgsOrConnect = 1;
constexpr int kExitNotModified = 2;
constexpr int kExitNotConfirmed = 3;
constexpr int kExitUnreachable = 4;

struct Args {
    std::string connectIp;         // 现在用来连接设备的地址
    std::string newIp;             // eth1 的新 IP
    int prefixLen{0};              // eth1 的新前缀长度
    const char* gateway{nullptr};  // nullptr 保持，"" 删除，其余为新网关
    int confirmS{0};               // 0 表示设备默认窗口
    std::string confirmVia;  // 确认用的设备地址，空表示用新 IP
};

void printUsage(const char* prog) {
    std::printf(
        "用法: %s <设备连接IP> <新IP> <前缀长度> [网关|keep|none] "
        "[确认窗口秒]\n"
        "          [--confirm-via <确认用的设备地址>]\n"
        "  例(经 USB 网卡改 eth1): %s 192.168.55.1 192.168.123.150 24 "
        "--confirm-via 192.168.55.1\n",
        prog, prog);
}

bool parseArgs(int argc, char** argv, Args* args) {
    std::vector<const char*> positional;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--confirm-via") == 0) {
            if (i + 1 >= argc) {
                std::printf("--confirm-via 需要一个设备地址\n");
                return false;
            }
            args->confirmVia = argv[++i];
        } else if (std::strncmp(argv[i], "--", 2) == 0) {
            std::printf("未知选项: %s\n", argv[i]);
            return false;
        } else {
            positional.push_back(argv[i]);
        }
    }
    if (positional.size() < 3 || positional.size() > 5) {
        return false;
    }
    args->connectIp = positional[0];
    args->newIp = positional[1];
    args->prefixLen = std::atoi(positional[2]);
    if (positional.size() >= 4) {
        if (std::strcmp(positional[3], "keep") == 0) {
            args->gateway = nullptr;
        } else if (std::strcmp(positional[3], "none") == 0) {
            args->gateway = "";
        } else {
            args->gateway = positional[3];
        }
    }
    if (positional.size() >= 5) {
        args->confirmS = std::atoi(positional[4]);
    }
    return true;
}

std::unique_ptr<xg::Device> tryOpen(const std::string& ip) {
    xg::Options opts;
    opts.deviceIp = ip;
    opts.connectTimeoutMs = kProbeConnectTimeoutMs;
    xg::Error err = xg::Error::Ok;
    return xg::Device::open(opts, &err);
}

bool probeRepeatedly(const std::string& ip, int attempts) {
    for (int i = 0; i < attempts; ++i) {
        if (tryOpen(ip)) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    return false;
}

// 经其他网卡确认时，顺带看一眼新 IP 是否可达，仅供参考
void probeNewIpForInfo(const std::string& newIp) {
    if (tryOpen(newIp)) {
        std::printf("参考: 本机也能连上新 IP %s, 新地址可达。\n",
                    newIp.c_str());
    } else {
        std::printf(
            "参考: 本机暂时连不上新 IP %s。若本机网卡不在该网段，这是正常的；"
            "请从能到达该网段的主机自行验证。\n",
            newIp.c_str());
    }
}

}  // namespace

int main(int argc, char** argv) {
    Args args;
    if (!parseArgs(argc, argv, &args)) {
        printUsage(argv[0]);
        return kExitBadArgsOrConnect;
    }
    const std::string confirmIp =
        args.confirmVia.empty() ? args.newIp : args.confirmVia;
    const bool confirmOnNewIp = (confirmIp == args.newIp);
    const int windowS = (args.confirmS == 0)
                            ? XG_NETWORK_CONFIRM_TIMEOUT_DEFAULT_S
                            : args.confirmS;

    std::printf("连接设备 %s ...\n", args.connectIp.c_str());
    xg::Error err = xg::Error::Ok;
    xg::Options opts;
    opts.deviceIp = args.connectIp;
    auto device = xg::Device::open(opts, &err);
    if (!device) {
        std::printf("连接失败: %s\n", xg::toString(err));
        return kExitBadArgsOrConnect;
    }

    const char* gatewayText =
        (args.gateway == nullptr)
            ? "保持"
            : (*args.gateway == '\0' ? "删除" : args.gateway);
    std::printf("请求修改 eth1: %s/%d, 网关=%s, 确认窗口=%d 秒\n",
                args.newIp.c_str(), args.prefixLen, gatewayText, windowS);
    std::printf("确认方式: 经 %s 发送确认%s\n", confirmIp.c_str(),
                confirmOnNewIp ? "(同时验证新 IP 可达)" : "(不验证新 IP 可达)");

    const xg::Error rc = device->setNetwork(args.newIp, args.prefixLen,
                                            args.gateway, args.confirmS);
    device.reset();  // 成功后只允许销毁旧句柄，确认用新打开的句柄

    if (rc == xg::Error::InvalidArg || rc == xg::Error::Rejected ||
        rc == xg::Error::Protocol) {
        std::printf("设备未做修改: %s\n", xg::toString(rc));
        if (rc == xg::Error::Rejected) {
            std::printf(
                "  常见原因: 正在扫描或建图; 已有变更待确认; "
                "新网段与设备其他网口"
                "(含 USB 网卡)重叠; "
                "保持网关但现有网关不在新网段内。详见设备日志。\n");
        }
        return kExitNotModified;
    }
    if (rc == xg::Error::Timeout) {
        std::printf(
            "设备未应答, 结果未知: 设备可能已切换地址, 继续尝试确认。\n");
    } else if (rc != xg::Error::Ok) {
        std::printf("setNetwork 失败: %s\n", xg::toString(rc));
        return kExitNotModified;
    } else {
        std::printf("设备已接受修改, 在 %d 秒窗口内连接 %s 并确认 ...\n",
                    windowS, confirmIp.c_str());
    }

    // 设备从完成地址切换起算窗口，这里从现在起算，略保守
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(windowS);
    std::unique_ptr<xg::Device> confirmDev;
    bool reached = false;
    while (std::chrono::steady_clock::now() < deadline) {
        if (!confirmDev) {
            confirmDev = tryOpen(confirmIp);
            if (!confirmDev) {
                std::printf("  %s 暂不可达, 重试 ...\n", confirmIp.c_str());
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }
            reached = true;
            std::printf("已连上 %s, 发送确认 ...\n", confirmIp.c_str());
        }
        const xg::Error confirmRc = confirmDev->confirmNetwork();
        if (confirmRc == xg::Error::Ok) {
            std::printf("确认成功: eth1 已永久改为 %s/%d\n", args.newIp.c_str(),
                        args.prefixLen);
            if (!confirmOnNewIp) {
                probeNewIpForInfo(args.newIp);
            }
            return kExitConfirmed;
        }
        if (confirmRc == xg::Error::Rejected) {
            // 经 USB 等不受影响的连接确认时, 确认可能早于设备应用新配置到达
            std::printf("  设备暂不接受确认(可能还在应用新配置), 重试 ...\n");
        } else if (confirmRc == xg::Error::Timeout) {
            std::printf("  确认无应答, 重新连接后重试 ...\n");
            confirmDev.reset();
        } else {
            std::printf("确认失败: %s\n", xg::toString(confirmRc));
            return kExitNotConfirmed;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    if (reached) {
        std::printf(
            "窗口内设备一直没有接受确认: eth1 的变更可能应用失败或已回滚, "
            "应保持旧配置。请查看设备日志。\n");
        return kExitNotConfirmed;
    }
    std::printf("窗口内未能连上 %s, 设备会把 eth1 恢复为旧配置。\n",
                confirmIp.c_str());
    if (args.connectIp != confirmIp) {
        std::printf("探测原连接地址 %s ...\n", args.connectIp.c_str());
        if (probeRepeatedly(args.connectIp, 10)) {
            std::printf(
                "设备可经 %s 访问: eth1 变更已回滚或未生效, 当前配置未变。\n",
                args.connectIp.c_str());
            return kExitNotConfirmed;
        }
    }
    std::printf(
        "连接地址都连不上。请检查本机网卡/线缆是否能到达设备, 稍后重试。\n");
    return kExitUnreachable;
}
