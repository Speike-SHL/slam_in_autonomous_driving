//
// Created by xiang on 2022/3/15.
//

#ifndef SLAM_IN_AUTO_DRIVING_LIDAR_2D_UTILS_H
#define SLAM_IN_AUTO_DRIVING_LIDAR_2D_UTILS_H

#include "common/eigen_types.h"
#include "common/lidar_utils.h"

#include <opencv2/core/core.hpp>

/// 为2D lidar的一些辅助函数
namespace sad {

/**
 * 将一帧2d scan画到image上
 * @param scan 一帧2d scan
 * @param pose 当前雷达在世界坐标系下的位姿
 * @param image image引用传递
 * @param color 颜色
 * @param image_size 图片大小(800)
 * @param resolution 分辨率，一米多少个像素(20)
 * @param pose_submap 如果是子地图，提供子地图的pose(SE2())
 */
void Visualize2DScan(Scan2d::Ptr scan, const SE2& pose, cv::Mat& image, const Vec3b& color, int image_size = 800,
                     float resolution = 20.0, const SE2& pose_submap = SE2());

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_LIDAR_2D_UTILS_H
