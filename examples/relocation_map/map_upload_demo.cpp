/******************************************************************************
This file is part of XGRIDS Robot SDK.
Project: lixel_sdk
Author : XGRIDS Embedded Software Team
*******************************************************************************
History:
1. create this file. (DATE: 20260903)
******************************************************************************/

/**
 * @file   map_upload_demo.cpp
 * @brief  XGRIDS Robot SDK 示例：重定位地图上传
 *
 * 演示把上位机本机的一整套重定位地图上传到设备并激活：
 *   连接设备 -> uploadMap(带进度) -> setActiveMap -> listMaps 确认。
 *
 * 本机地图目录结构要求（SDK 上传前会本地校验，缺文件立即失败）：
 *   <mapDir>/
 *   ├── location_map.las
 *   └── loc_database/
 *       ├── camera_param.proto
 *       ├── fpfh_feature.proto
 *       ├── keyframes.proto
 *       └── vlad.proto
 *
 * 上传激活后，重定位还需注入初值(见 reloc_demo 的 setLocInitPose)才会开始
 * 全局初始化重定位；本示例只负责"把地图放进设备并激活"。
 *
 * 前置条件与副作用（本示例会改设备的有效地图，本轮板端验证未执行）：
 *   - 本机地图目录结构完整（见上）；设备磁盘有足够空间。
 *   - 不 startRecord、不订阅；会把地图写进设备，并通过 setActiveMap 切换设备当前
 *     有效地图（会影响之后的重定位）。同名 mapId 是否覆盖、旧地图是否保留，
 *     头文件未说明，运行前请先用本示例或 listMaps 查看设备现有地图。
 *   - uploadMap 阻塞到完成或失败；需要换回原地图时再对原 mapId 调 setActiveMap。
 *
 * 调用顺序与每步为什么：uploadMap（带进度回调，内部分片 + NACK 重传）->
 * setActiveMap（激活后重定位才用新地图）-> listMaps（确认上传结果，标注激活项）。
 *
 * 预期输出（依据代码推导，未上板实测）：
 *   已连接设备 <IP> -> 开始上传地图: dir=... mapId=... -> 上传进度: 0.0% 逐步到 100.0%
 *   （同一行用 \r 刷新）-> 上传完成。 -> 已激活地图: <mapId> ->
 *   设备上现有地图 (N): 每个地图一行 "[*] mapId=...  files=..  total=.. MB"（* 为当前激活），
 *   其下列出各文件路径与大小 -> 完成。退出码 0。
 *
 * 失败现象与处理（依据代码推导，未上板实测）：
 *   - 不带参数或参数少于 3 个：打印用法，退出码 1。
 *   - "连接失败: ..."：检查网线、网段、设备端服务是否已启动；退出码 1。
 *   - "上传失败: InvalidArg"：本机地图目录结构不全（见上方目录要求），补齐后重跑；
 *     "上传失败: Rejected"：设备端磁盘不足 / 完整性校验失败 / category 不允许，
 *     清理设备空间或核对地图文件后重跑；退出码 1，此时尚未激活，设备有效地图不变。
 *   - "激活地图失败: ..."：地图已上传但未切换，退出码 1；可用 listMaps 确认地图在设备上，
 *     再重跑或单独调 setActiveMap。
 *   - 最后的 listMaps 失败时不打印列表，也不影响退出码（仍为 0）。
 *
 * 线程模型：同步调用；进度回调只做打印，不要在里面做耗时操作。
 *
 * 运行：
 *   ./map_upload_demo <设备IP> <本机地图目录> <mapId>
 *   ./map_upload_demo 192.168.123.103 /home/user/maps/site_A site_A
 */

#include "xg/robot_sdk.h"

#include <cstdio>
#include <string>

int main(int argc, char** argv) {
    if (argc < 4) {
        std::printf("XGRIDS Robot SDK 地图上传示例 (SDK 版本 %s)\n\n",
                    xg::sdkVersion().c_str());
        std::printf("用法:\n");
        std::printf("  %s <设备IP> <本机地图目录> <mapId>\n\n", argv[0]);
        std::printf("示例:\n");
        std::printf("  %s 192.168.123.103 /home/user/maps/site_A site_A\n", argv[0]);
        return 1;
    }

    const std::string deviceIp = argv[1];
    const std::string mapDir = argv[2];
    const std::string mapId = argv[3];

    // 1) 连接设备
    xg::Options opts;
    opts.deviceIp = deviceIp;
    xg::Error err = xg::Error::Ok;
    auto dev = xg::Device::open(opts, &err);
    if (!dev) {
        std::printf("连接失败: %s\n", xg::toString(err));
        std::printf("请检查: 网线是否连通、是否与设备同网段、设备端服务是否已启动\n");
        return 1;
    }
    std::printf("已连接设备 %s\n", deviceIp.c_str());

    // 2) 上传地图（带进度回调）
    xg::Device::MapUploadOptions up;
    up.localMapDir = mapDir;
    up.mapId = mapId;
    std::printf("开始上传地图: dir=%s mapId=%s\n", mapDir.c_str(), mapId.c_str());

    err = dev->uploadMap(up, [](float progress) {
        std::printf("\r上传进度: %5.1f%%", progress * 100.0f);
        std::fflush(stdout);
    });
    std::printf("\n");
    if (err != xg::Error::Ok) {
        std::printf("上传失败: %s\n", xg::toString(err));
        std::printf("提示: InvalidArg 多为本机地图目录结构不全(见文件头注释)；\n");
        std::printf("      Rejected 多为设备端磁盘不足/完整性校验失败/category 不允许。\n");
        return 1;
    }
    std::printf("上传完成。\n");

    // 3) 激活地图
    err = dev->setActiveMap(mapId);
    if (err != xg::Error::Ok) {
        std::printf("激活地图失败: %s\n", xg::toString(err));
        return 1;
    }
    std::printf("已激活地图: %s\n", mapId.c_str());

    // 4) 列出设备上的地图确认
    std::vector<xg::Device::MapInfo> maps;
    if (dev->listMaps(&maps) == xg::Error::Ok) {
        std::printf("\n设备上现有地图 (%zu):\n", maps.size());
        for (const auto& m : maps) {
            std::printf("  [%s] mapId=%s  files=%zu  total=%.2f MB\n",
                        m.isActive ? "*" : " ", m.mapId.c_str(),
                        m.files.size(),
                        static_cast<double>(m.totalSize) / (1024.0 * 1024.0));
            for (const auto& f : m.files) {
                std::printf("        %-48s %8.1f KB\n", f.path.c_str(),
                            static_cast<double>(f.size) / 1024.0);
            }
        }
    }

    std::printf("\n完成。后续注入初值可参考 reloc_demo。\n");
    return 0;
}
