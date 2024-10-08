cmake_minimum_required(VERSION 3.5)
project(ros2_control_demo_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY rrbot/urdf rrbot/rviz
  DESTINATION share/${PROJECT_NAME}/rrbot
)

install(
  DIRECTORY diffbot/urdf diffbot/rviz
  DESTINATION share/${PROJECT_NAME}/diffbot
)

install(
  DIRECTORY carlikebot/urdf carlikebot/rviz
  DESTINATION share/${PROJECT_NAME}/carlikebot
)

install(
  DIRECTORY r6bot/meshes r6bot/srdf r6bot/urdf r6bot/rviz
  DESTINATION share/${PROJECT_NAME}/r6bot
)


install(
  DIRECTORY octo_gauge_rover/meshes octo_gauge_rover/urdf octo_gauge_rover/config  octo_gauge_rover/worlds
  DESTINATION share/${PROJECT_NAME}/octo_gauge_rover
)

ament_package()
