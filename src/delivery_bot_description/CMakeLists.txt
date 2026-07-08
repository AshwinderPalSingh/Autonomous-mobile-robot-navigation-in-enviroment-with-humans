cmake_minimum_required(VERSION 3.8)
project(delivery_bot_description)

find_package(ament_cmake REQUIRED)

# meshes/ ships empty for now -- SolidWorks STL exports (one per link,
# meters, binary) land there later without any build-system change.
install(
  DIRECTORY urdf launch meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
