//
// Created by xiang on 2022/3/15.
//
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/highgui.hpp>

#include "ch6/lidar_2d_utils.h"
#include "common/io_utils.h"

using namespace std;

DEFINE_string(bag_path, "./dataset/sad/2dmapping/floor2.bag", "数据包路径");

/// 测试从rosbag中读取2d scan并plot的结果

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);    // 初始化Glog
    FLAGS_stderrthreshold = google::INFO;  // 设置Glog级别
    FLAGS_colorlogtostderr = true;         // 设置Glog输出带颜色
    google::ParseCommandLineFlags(&argc, &argv, true);  // 解析Gflags参数，然后后文使用时可以直接使用“FLAGS_参数名”

    // 创建RosbagIO对象, 传入数据包路径
    sad::RosbagIO rosbag_io(fLS::FLAGS_bag_path);

    // 1. 首先调用AddScan2DHandle函数, 并使用lambda表达式添加一个处理并显示2d scan的回调函数f。
    // 2. 然后在AddScan2DHandle函数中, 调用AddHandle函数, 并传入另一个lambda表达式func, 把func和topic绑定起来。
    // 3. 然后调用Go函数, 开始读取数据包, 如果找到了话题，就调用AddHandle函数中的lambda表达式func函数，
    // 4. 然后进一步调用AddScan2DHandle函数中的lambda表达式f函数
    // 5. 然后在f函数中, 调用Visualize2DScan函数, 把2d scan转换成图像, 并显示出来
    // 345步是在Go函数的for循环中不断找话题调用的
    rosbag_io
        .AddScan2DHandle("/pavo_scan_bottom",
                         [](Scan2d::Ptr scan) {
                             cv::Mat image;
                             sad::Visualize2DScan(scan, SE2(), image, Vec3b(255, 0, 0));
                             cv::namedWindow("2d scan", cv::WINDOW_KEEPRATIO);
                             cv::imshow("2d scan", image);
                             cv::waitKey(20);
                             return true;
                         })
        .Go();

    return 0;
}
