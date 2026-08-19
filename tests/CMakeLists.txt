cmake_minimum_required(VERSION 3.22)
project(agv_trajectory_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(agv_trajectory_tests
    test_main.cpp
    test_trajectory_gen.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../components/trajectory/trajectory_gen.cpp
)

target_include_directories(agv_trajectory_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party
    ${CMAKE_CURRENT_SOURCE_DIR}/../components/trajectory
    ${CMAKE_CURRENT_SOURCE_DIR}/../components/kinematics
)

enable_testing()
add_test(NAME agv_trajectory_tests COMMAND agv_trajectory_tests)

add_executable(agv_kinematics_tests
    test_main.cpp
    test_swerve_kinematics.cpp
)

target_include_directories(agv_kinematics_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party
    ${CMAKE_CURRENT_SOURCE_DIR}/../components/kinematics
)

add_test(NAME agv_kinematics_tests COMMAND agv_kinematics_tests)