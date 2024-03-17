add_executable(3.2.2_run_imu_integration
        3.2.2_run_imu_integration.cc
        )

target_link_libraries(3.2.2_run_imu_integration
        glog gflags ${PROJECT_NAME}.common
        )

add_executable(3.5_run_eskf_gins 3.5_run_eskf_gins.cc)
target_link_libraries(3.5_run_eskf_gins
        glog gflags ${PROJECT_NAME}.common ${PROJECT_NAME}.ch3
        )

add_executable(3.3.4_process_gnss 3.3.4_process_gnss.cc)
target_link_libraries(3.3.4_process_gnss
        glog gflags ${PROJECT_NAME}.common ${PROJECT_NAME}.ch3
        )

add_library(${PROJECT_NAME}.ch3
        static_imu_init.cc
        utm_convert.cc
        # ieskf/nav_state_manifold.cc
        # ieskf/ieskf.cc

        ${PROJECT_SOURCE_DIR}/thirdparty/utm_convert/utm.cc
        ${PROJECT_SOURCE_DIR}/thirdparty/utm_convert/tranmerc.cc
        )

target_link_libraries(${PROJECT_NAME}.ch3
        glog gflags ${PROJECT_NAME}.common
        )

