add_executable(2.3_motion 2.3_motion.cc)
target_link_libraries(2.3_motion
        ${PROJECT_NAME}.common
        ${PROJECT_NAME}.tools
        )
