cmake_minimum_required(VERSION 3.5)
project(ar_track_alvar)

# if(NOT WIN32)
#   if(NOT CMAKE_CXX_STANDARD)
#     set(CMAKE_CXX_STANDARD 14)
#   endif()
#   if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#     add_compile_options(-Wall -Wextra -Wpedantic)
#   endif()
# endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED COMPONENTS calib3d core highgui imgproc)
if(OpenCV_VERSION VERSION_GREATER 4.5.0)
  target_compile_definitions(library PUBLIC USE_LEGACY_TRACKING=1)
endif()



find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(image_transport REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(perception_pcl  REQUIRED)
find_package(pcl_ros)
find_package(pcl_conversions REQUIRED)
find_package(ar_track_alvar_msgs)
find_package(std_msgs)
find_package(sensor_msgs)
find_package(geometry_msgs)
find_package(visualization_msgs)
find_package(PCL REQUIRED QUIET COMPONENTS common io)

find_package(Eigen3 REQUIRED)
find_package(TinyXML2 REQUIRED)

include_directories(include)


#set(GENCPP_DEPS ar_track_alvar_msgs_gencpp std_msgs_gencpp sensor_msgs_gencpp geometry_msgs_gencpp visualization_msgs_gencpp)

# Kinect filtering code
set(KINECT_FILTERING_TARGETS kinect_filtering medianFilter)
set(ALVAR_TARGETS ar_track_alvar individualMarkers individualMarkersNoKinect trainMarkerBundle findMarkerBundles findMarkerBundlesNoKinect createMarker ar_track_alvar)

# dynamic reconfigure support
#generate_dynamic_reconfigure_options(cfg/Params.cfg)

set(dependencies
  OpenCV
  tf2_ros 
  tf2
  pcl_ros
  pcl_conversions 
  std_msgs 
  image_transport
  perception_pcl 
  visualization_msgs 
  rclcpp 
  resource_retriever 
  geometry_msgs 
  cv_bridge 
  sensor_msgs
  ar_track_alvar_msgs
)

include_directories(include
                    ${OpenCV_INCLUDE_DIRS}
                    ${TinyXML2_INCLUDE_DIRS}
                    ${PCL_COMMON_INCLUDE_DIRS}
)

add_library(ar_track_alvar
    src/Camera.cpp
    src/CaptureDevice.cpp
    src/Pose.cpp
    src/Marker.cpp
    src/MarkerDetector.cpp
    src/Bitset.cpp
    src/Rotation.cpp
    src/CvTestbed.cpp
    src/CaptureDevice.cpp
    src/CaptureFactory.cpp
    src/CaptureFactory_unix.cpp
    src/FileFormatUtils.cpp
    src/Threads.cpp
    src/Threads_unix.cpp
    src/Mutex.cpp
    src/Mutex_unix.cpp
    src/ConnectedComponents.cpp
    src/Line.cpp 
    src/Plugin.cpp
    src/Plugin_unix.cpp
    src/DirectoryIterator.cpp
    src/DirectoryIterator_unix.cpp
    src/Draw.cpp
    src/Util.cpp
    src/Filter.cpp
    src/Kalman.cpp
    src/kinect_filtering.cpp
    src/Optimization.cpp
    src/MultiMarker.cpp
    src/MultiMarkerBundle.cpp
    src/MultiMarkerInitializer.cpp
    )
    
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${TinyXML_LIBRARIES})
ament_target_dependencies(ar_track_alvar ${dependencies})

add_library(kinect_filtering src/kinect_filtering.cpp)
ament_target_dependencies(kinect_filtering ${dependencies})


add_library(medianFilter src/medianFilter.cpp)
target_link_libraries(medianFilter ar_track_alvar)
ament_target_dependencies(medianFilter ${dependencies})

add_executable(individualMarkers nodes/IndividualMarkers.cpp)
target_link_libraries(individualMarkers ar_track_alvar kinect_filtering)
ament_target_dependencies(individualMarkers ${dependencies})

add_executable(individualMarkersNoKinect nodes/IndividualMarkersNoKinect.cpp)
target_link_libraries(individualMarkersNoKinect ar_track_alvar)
ament_target_dependencies(individualMarkersNoKinect ${dependencies})

add_executable(trainMarkerBundle nodes/TrainMarkerBundle.cpp)
target_link_libraries(trainMarkerBundle ar_track_alvar)
ament_target_dependencies(trainMarkerBundle ${dependencies})


add_executable(findMarkerBundles nodes/FindMarkerBundles.cpp)
target_link_libraries(findMarkerBundles ar_track_alvar kinect_filtering medianFilter)
ament_target_dependencies(findMarkerBundles ${dependencies})

add_executable(findMarkerBundlesNoKinect nodes/FindMarkerBundlesNoKinect.cpp)
target_link_libraries(findMarkerBundlesNoKinect ar_track_alvar)
ament_target_dependencies(findMarkerBundlesNoKinect ${dependencies})

# add_executable(createMarker src/SampleMarkerCreator.cpp)
# target_link_libraries(createMarker ar_track_alvar)
# ament_target_dependencies(createMarker ${dependencies})


ament_export_include_directories(include)
ament_export_libraries(ar_track_alvar)
ament_export_dependencies(OpenCV ar_track_alvar_msgs std_msgs rclcpp tf2_ros tf2 message_runtime image_transport sensor_msgs geometry_msgs visualization_msgs resource_retriever cv_bridge perception_pcl pcl_conversions)
ament_package()

# install(TARGETS ${ALVAR_TARGETS} ${KINECT_FILTERING_TARGETS}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}
# )

# install(DIRECTORY launch bundles
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
# )
# if(COMPILER_SUPPORTS_CXX11)
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
# elseif(COMPILER_SUPPORTS_CXX0X)
# else()
#         message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
# endif()



#TODO fix these tests
# if(BUILD_TESTING)
#   add_subdirectory(test)
# endif()


# if (BUILD_TESTING)
#   find_package(catkin REQUIRED COMPONENTS roslaunch rostest)
#   add_rostest(test/marker_arg_config-basic.test DEPENDENCIES ${PROJECT_NAME}_4markers_tork.bag)
#   add_rostest(test/marker_arg_config-full.test DEPENDENCIES ${PROJECT_NAME}_4markers_tork.bag)
#   add_rostest(test/marker_param_config-basic.test DEPENDENCIES ${PROJECT_NAME}_4markers_tork.bag)
#   add_rostest(test/marker_param_config-full.test DEPENDENCIES ${PROJECT_NAME}_4markers_tork.bag)
  
#   catkin_download_test_data(
#     ${PROJECT_NAME}_4markers_tork.bag
#     http://download.ros.org/data/ar_track_alvar/ar_track_alvar_4markers_tork_2017-02-08-11-21-14.bag
#     # Workaround the issue http://answers.ros.org/question/253787/accessing-data-downloaded-via-catkin_download_test_data/
#     # by downloading into source folder.
#     #DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
#     DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
#     MD5 627aa0316bbfe4334e06023d7c2b4087
#    )
#   foreach(LAUNCH_FILE ${LAUNCH_FILES})
#     roslaunch_add_file_check(${LAUNCH_FILE} USE_TEST_DEPENDENCIES)
#   endforeach()
  
#   file(GLOB LAUNCH_FILES launch/*.launch test/*.test)
# endif()

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

cmake_policy(SET CMP0046 OLD)
