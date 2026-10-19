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
