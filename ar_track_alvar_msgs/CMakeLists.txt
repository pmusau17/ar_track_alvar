cmake_minimum_required(VERSION 3.5)
project(ar_track_alvar_msgs)

if(NOT WIN32)
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
  endif()
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        msg/AlvarMarkers.msg
        msg/AlvarMarker.msg
    DEPENDENCIES
        geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
