//
// Created by xiang on 2021/11/11.
//
/**
 * @details 本程序演示使用GNSS+IMU, 使用ESKF进行组合导航 (抄自课本P94的实验现象)
 * 1. 在有良好的RTK和IMU数据的情况下，轨迹是比较平滑的，读者可以在UI中对轨迹进行放大查看
 * 2. 车体系下的速度以X轴正向为主，这与实际是相符的。而世界系下的速度既有X的分量，也有Y的分量
 * 3. 在一部分RTK不良的场合，ESKF会缺少观测，位移部分会呈现快速发散状态。当RTK恢复以后，ESKF会自动收敛
 * 4. 本节程序选择在RTK航行有效时无条件相信RTK，但实际RTK数据本身也会有抖动和不良的情况，这时会导致ESKF的轨迹也发生抖动
 */

#include "ch3/eskf.hpp"
#include "ch3/static_imu_init.h"
#include "common/io_utils.h"
#include "tools/ui/pangolin_window.h"
#include "utm_convert.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <fstream>
#include <iomanip>

DEFINE_string(txt_path, "./data/ch3/10.txt", "数据文件路径");
DEFINE_double(antenna_angle, 12.06, "RTK天线安装偏角（角度）");
DEFINE_double(antenna_pox_x, -0.17, "RTK天线安装偏移X");
DEFINE_double(antenna_pox_y, -0.20, "RTK天线安装偏移Y");
DEFINE_bool(with_ui, true, "是否显示图形界面");
DEFINE_bool(with_odom, false, "是否加入轮速计信息");

/**
 * 本程序演示使用RTK+IMU进行组合导航
 */
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (fLS::FLAGS_txt_path.empty()) {
        return -1;
    }

    // 初始化器
    sad::StaticIMUInit imu_init;  // 使用默认配置
    sad::ESKFD eskf;

    sad::TxtIO io(FLAGS_txt_path);
    Vec2d antenna_pos(FLAGS_antenna_pox_x, FLAGS_antenna_pox_y);  // RTK安装偏移

    auto save_vec3 = [](std::ofstream& fout, const Vec3d& v) { fout << v[0] << " " << v[1] << " " << v[2] << " "; };
    auto save_quat = [](std::ofstream& fout, const Quatd& q) {
        fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
    };

    auto save_result = [&save_vec3, &save_quat](std::ofstream& fout, const sad::NavStated& save_state) {
        fout << std::setprecision(18) << save_state.timestamp_ << " " << std::setprecision(9);
        save_vec3(fout, save_state.p_);
        save_quat(fout, save_state.R_.unit_quaternion());
        save_vec3(fout, save_state.v_);
        save_vec3(fout, save_state.bg_);
        save_vec3(fout, save_state.ba_);
        fout << std::endl;
    };

    std::ofstream fout("./data/ch3/gins.txt");
    bool imu_inited = false, gnss_inited = false;

    std::shared_ptr<sad::ui::PangolinWindow> ui = nullptr;
    if (FLAGS_with_ui) {
        ui = std::make_shared<sad::ui::PangolinWindow>();
        ui->Init();
    }

    /// 设置各类回调函数
    bool first_gnss_set = false;
    Vec3d origin = Vec3d::Zero();

    io.SetIMUProcessFunc([&](const sad::IMU& imu) {
          /// IMU 处理函数
          // StaticIMUInit 初始化器初始化估计IMU零偏、噪声和重力方向
          if (!imu_init.InitSuccess()) {
              imu_init.AddIMU(imu);
              return;
          }

          // 初始化器初始化成功，读取初始化器中的零偏、噪声和重力方向来设置ESKF
          if (!imu_inited) {
              // 读取初始零偏，设置ESKF
              sad::ESKFD::Options options;
              // 噪声由初始化器估计
              options.gyro_var_ = sqrt(imu_init.GetCovGyro()[0]);
              options.acce_var_ = sqrt(imu_init.GetCovAcce()[0]);
              eskf.SetInitialConditions(options, imu_init.GetInitBg(), imu_init.GetInitBa(), imu_init.GetGravity());
              imu_inited = true;
              LOG(INFO) << "Gavity: " << eskf.GetGravity().transpose();
              return;
          }

          if (!gnss_inited) {
              /// 等待有效的RTK数据, 因为ESKF还需要首个RTK数据来确定地图原点和首个名义状态的初始值
              return;
          }

          /// GNSS 也接收到之后，再开始进行预测
          eskf.Predict(imu);

          /// predict就会更新ESKF，所以此时就可以发送数据
          auto state = eskf.GetNominalState();
          if (ui) {
              ui->UpdateNavState(state);
          }

          /// 记录数据以供绘图
          save_result(fout, state);

          usleep(1e3);
      })
        .SetGNSSProcessFunc([&](const sad::GNSS& gnss) {
            /// GNSS 处理函数
            if (!imu_inited) {
                // 如果IMU未初始化，不处理GNSS
                return;
            }

            sad::GNSS gnss_convert = gnss;
            if (!sad::ConvertGps2UTM(gnss_convert, antenna_pos, FLAGS_antenna_angle) || !gnss_convert.heading_valid_) {
                // GNSS数据转为UTM失败，或者heading无效，不处理
                return;
            }

            /// 去掉原点
            if (!first_gnss_set) {
                origin = gnss_convert.utm_pose_.translation();
                first_gnss_set = true;
            }
            gnss_convert.utm_pose_.translation() -= origin;

            // 要求RTK heading有效，才能合入ESKF。首个GNSS数据会用于初始化ESKF的R_, p_
            eskf.ObserveGps(gnss_convert);

            auto state = eskf.GetNominalState();
            if (ui) {
                ui->UpdateNavState(state);
            }
            save_result(fout, state);

            gnss_inited = true;
        })
        .SetOdomProcessFunc([&](const sad::Odom& odom) {
            /// Odom 处理函数，本章Odom只给初始化使用
            imu_init.AddOdom(odom);
            if (FLAGS_with_odom && imu_inited && gnss_inited) {
                eskf.ObserveWheelSpeed(odom);
            }
        })
        .Go();

    while (ui && !ui->ShouldQuit()) {
        usleep(1e5);
    }
    if (ui) {
        ui->Quit();
    }
    return 0;
}
