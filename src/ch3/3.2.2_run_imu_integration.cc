//
// Created by xiang on 2021/11/5.
//

/// 程序运行完后结果会保存到data/ch3/state.txt中，可以用python3 scripts/plot_ch3_state.py data/ch3/state.txt进行可视化

#include <glog/logging.h>
#include <iomanip>

#include "ch3/imu_integration.h"
#include "common/io_utils.h"
#include "tools/ui/pangolin_window.h"

DEFINE_string(imu_txt_path, "./data/ch3/11.txt", "数据文件路径");
DEFINE_bool(with_ui, true, "是否显示图形界面");

/// 本程序演示如何对IMU进行直接积分
/// 该程序需要输入data/ch3/下的文本文件，同时它将状态输出到data/ch3/state.txt中，在UI中也可以观察到车辆运动
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_imu_txt_path.empty()) {
        return -1;
    }

    sad::TxtIO io(FLAGS_imu_txt_path);

    // 该实验中，我们假设零偏已知
    Vec3d gravity(0, 0, -9.8);  // 重力方向
    Vec3d init_bg(00.000224886, -7.61038e-05, -0.000742259);
    Vec3d init_ba(-0.165205, 0.0926887, 0.0058049);

    sad::IMUIntegration imu_integ(gravity, init_bg, init_ba);

    // 根据参数决定是否初始化UI
    std::shared_ptr<sad::ui::PangolinWindow> ui = nullptr;
    if (FLAGS_with_ui) {
        ui = std::make_shared<sad::ui::PangolinWindow>();
        ui->Init();
    }

    /// 记录结果
    auto save_result = [](std::ofstream& fout, double timestamp, const Sophus::SO3d& R, const Vec3d& v,
                          const Vec3d& p) {
        auto save_vec3 = [](std::ofstream& fout, const Vec3d& v) { fout << v[0] << " " << v[1] << " " << v[2] << " "; };
        auto save_quat = [](std::ofstream& fout, const Quatd& q) {
            fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
        };

        fout << std::setprecision(18) << timestamp << " " << std::setprecision(9);
        save_vec3(fout, p);
        save_quat(fout, R.unit_quaternion());
        save_vec3(fout, v);
        fout << std::endl;
    };

    /**
     * 程序的主要部分。
     * 1. 用lambda表达式设置一个imu处理函数，该函数输入sad::IMU类型的imu数据，返回void，同时捕获了imu_integ,
     * save_result, fout, ui
     * 2. 该lambda表达式是一个using IMUProcessFuncType = std::function<void(const IMU &)>类型的对象,
     * 传入了io.SetIMUProcessFunc中
     * 3. 在io.SetIMUProcessFunc中通过imu_proc_ = std::move(imu_proc)将imu_proc的所有权转移到imu_proc_,
     * 即把这里定义的所有lambda表达式的所有权转移到imu_proc_
     * 4. 然后调用io.Go(), 遍历文件内容, 如果遍历到了imu行, 就调用imu_proc_处理, 然后开始执行.
     * 5. 首先执行imu_integ.AddIMU(imu), 然后调用save_result的lambda表达式保存结果, 最后更新ui
     * 6. 继续在io.Go()中遍历文件内容, 直到遍历完毕
     */
    std::ofstream fout("./data/ch3/state.txt");
    io.SetIMUProcessFunc([&imu_integ, &save_result, &fout, &ui](const sad::IMU& imu) {
          imu_integ.AddIMU(imu);
          save_result(fout, imu.timestamp_, imu_integ.GetR(), imu_integ.GetV(), imu_integ.GetP());
          if (ui) {
              ui->UpdateNavState(imu_integ.GetNavState());
              usleep(1e2);  // 100微秒 = 0.0001秒
          }
      }).Go();

    // 打开了可视化的话，等待界面退出
    std::cout << "遍历完毕，等待界面退出..." << std::endl;
    while (ui && !ui->ShouldQuit()) {
        usleep(1e4);
    }

    if (ui) {
        ui->Quit();
    }

    return 0;
}
