cmake_minimum_required(VERSION 3.5.1)
project(haptiquad_mujoco_bringup)

find_package(ament_cmake)


install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
