//
// Created by xiang on 2022/3/15.
//

#include "ch6/lidar_2d_utils.h"
#include <opencv2/imgproc.hpp>

namespace sad {

void Visualize2DScan(Scan2d::Ptr scan, const SE2& pose, cv::Mat& image, const Vec3b& color, int image_size,
                     float resolution, const SE2& pose_submap) {
    // 如果image数据为空，则初始化一个白色的image
    if (image.data == nullptr) {
        image = cv::Mat(image_size, image_size, CV_8UC3, cv::Vec3b(255, 255, 255));
    }

    // 遍历当前2d scan上的每个点
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        // 如果当前点不在有效范围内，则跳过
        if (scan->ranges[i] < scan->range_min || scan->ranges[i] > scan->range_max) {
            continue;
        }
        // 当前点的角度值
        double real_angle = scan->angle_min + i * scan->angle_increment;
        // 当前点的坐标值
        double x = scan->ranges[i] * std::cos(real_angle);
        double y = scan->ranges[i] * std::sin(real_angle);

        // 如果当前点的角度值在[angle_min+30, angle_max-30]范围内，则跳过
        if (real_angle < scan->angle_min + 30 * M_PI / 180.0 || real_angle > scan->angle_max - 30 * M_PI / 180.0) {
            continue;
        }

        // 将当前点转换到子地图坐标系下,公式为：p_submap = T_world_submap.inverse() * (T_world_lidar * p_lidar)
        Vec2d psubmap = pose_submap.inverse() * (pose * Vec2d(x, y));

        // 将子地图坐标系下的点转换到image坐标系下
        int image_x = int(psubmap[0] * resolution + image_size / 2);
        int image_y = int(psubmap[1] * resolution + image_size / 2);
        // 如果当前点在image范围内，则在image上画出来
        if (image_x >= 0 && image_x < image.cols && image_y >= 0 && image_y < image.rows) {
            image.at<cv::Vec3b>(image_y, image_x) = cv::Vec3b(color[0], color[1], color[2]);
        }
    }

    // 同时画出pose自身所在位置，公式为：pose_in_image = T_world_submap.inverse() * p_world_lidar * resolution +
    // image_size / 2
    Vec2d pose_in_image =
        pose_submap.inverse() * (pose.translation()) * double(resolution) + Vec2d(image_size / 2, image_size / 2);
    cv::circle(image, cv::Point2f(pose_in_image[0], pose_in_image[1]), 5, cv::Scalar(color[0], color[1], color[2]), 2);
}

}  // namespace sad
