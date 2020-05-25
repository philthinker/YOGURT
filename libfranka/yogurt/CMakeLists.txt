cmake_minimum_required(VERSION 3.4)

project(libfranka-yogurt CXX)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/../cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_library(examples_common STATIC
  examples_common.cpp
)

target_link_libraries(examples_common PUBLIC Franka::Franka Eigen3::Eigen3)

set(YOGURTS
  yogurtJointMotion
  yogurtJointVelocity
  yogurtCartesianPose
)

foreach(yogurt ${YOGURTS})
  add_executable(${yogurt} ${yogurt}.cpp)
  target_link_libraries(${yogurt} Franka::Franka examples_common Eigen3::Eigen3)
endforeach()

#target_link_libraries(joint_impedance_control Threads::Threads)
#target_link_libraries(motion_with_control Poco::Foundation)

include(GNUInstallDirs)
install(TARGETS ${YOGURTS}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
