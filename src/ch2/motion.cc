//
// Created by xiang on 22-12-29.
//

/**
 * @brief 本节程序演示一个正在作圆周运动的车辆,车辆的角速度与线速度可以在flags中设置
 */

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "common/eigen_types.h"
#include "common/math_utils.h"
#include "tools/ui/pangolin_window.h"

// 设置Gflags参数
DEFINE_double(angular_velocity, 10.0, "角速度（角度）制");
DEFINE_double(linear_velocity, 5.0, "车辆前进线速度 m/s");
DEFINE_bool(use_quaternion, false, "是否使用四元数计算");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);    // 初始化Glog
    FLAGS_stderrthreshold = google::INFO;  // 设置Glog级别
    FLAGS_colorlogtostderr = true;         // 设置Glog输出带颜色
    google::ParseCommandLineFlags(&argc, &argv, true);  // 解析Gflags参数，然后后文使用时可以直接使用“FLAGS_参数名”

    // 初始化Pangolin窗口
    sad::ui::PangolinWindow ui;
    if (ui.Init() == false) {
        return -1;
    }

    double angular_velocity_rad = FLAGS_angular_velocity * sad::math::kDEG2RAD;  // 弧度制转角速度
    SE3 pose;                                                                    // T_wb表示的位姿
    Vec3d omega(0, 0, angular_velocity_rad);                                     // 角速度矢量
    Vec3d v_body(FLAGS_linear_velocity, 0, 0);                                   // 本体系速度
    const double dt = 0.05;                                                      // 每次更新的时间

    int count = 0;

    while (ui.ShouldQuit() == false) {
        count++;
        if (count > 10000) {
            break;
        }

        // 更新自身位置
        Vec3d v_world = pose.so3() * v_body;  // 世界系下速度 V_w = R_wb * V_b
        pose.translation() += v_world * dt;   // 世界系下位置 P_w = P_w + V_w * dt

        // 更新自身旋转
        if (FLAGS_use_quaternion) {
            // 课本公式2.78
            Quatd q = pose.unit_quaternion() * Quatd(1, 0.5 * omega[0] * dt, 0.5 * omega[1] * dt, 0.5 * omega[2] * dt);
            // 因为上一步是近似的更新，所以需要重新归一化
            q.normalize();
            pose.so3() = SO3(q);
        } else {
            pose.so3() = pose.so3() * SO3::exp(omega * dt);  // 2.2.1李群视角下的运动学
        }

        LOG(INFO) << "pose: " << pose.translation().transpose();
        ui.UpdateNavState(sad::NavStated(0, pose, v_world));

        // 由于Pangolin的更新频率很高，为了模拟真实的时间流逝，usleep是模拟微妙，dt*1e6=50000微秒=50毫秒=0.05秒
        // 另一方面，太快会导致Pangolin无法正常显示
        usleep(dt * 1e6);
    }

    ui.Quit();
    return 0;
}
