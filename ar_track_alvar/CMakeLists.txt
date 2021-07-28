cmake_minimum_required(VERSION 3.5)
project(ar_track_alvar)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED COMPONENTS calib3d core highgui imgproc)
if(OpenCV_VERSION VERSION_GREATER 4.5.0)
  target_compile_definitions(library PUBLIC USE_LEGACY_TRACKING=1)
endif()

if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
     set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(image_transport REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(perception_pcl  REQUIRED)
find_package(pcl_ros)
find_package(pcl_conversions REQUIRED)
find_package(ar_track_alvar_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosbag2_bag_v2_plugins REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS common io)

find_package(Eigen3 REQUIRED)
find_package(TinyXML2 REQUIRED)

include_directories(include)


# Kinect filtering code
set(KINECT_FILTERING_TARGETS kinect_filtering medianFilter)
set(ALVAR_TARGETS ar_track_alvar individualMarkers individualMarkersNoKinect trainMarkerBundle findMarkerBundles findMarkerBundlesNoKinect createMarker ar_track_alvar)

set(dependencies
  OpenCV
  tf2_ros 
  tf2
  pcl_ros
  pcl_conversions 
  std_msgs 
  TinyXML2
  image_transport
  perception_pcl 
  visualization_msgs 
  rclcpp 
  resource_retriever 
  geometry_msgs 
  tf2_geometry_msgs
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

add_executable(createMarker src/SampleMarkerCreator.cpp)
target_link_libraries(createMarker ar_track_alvar)
ament_target_dependencies(createMarker ${dependencies})


ament_export_include_directories(include)
ament_export_libraries(ar_track_alvar)
ament_export_dependencies(OpenCV ar_track_alvar_msgs std_msgs rclcpp tf2_ros tf2 message_runtime image_transport sensor_msgs geometry_msgs visualization_msgs resource_retriever cv_bridge perception_pcl pcl_conversions)


install(TARGETS ${ALVAR_TARGETS} ${KINECT_FILTERING_TARGETS}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch bundles
  DESTINATION share/${PROJECT_NAME}
)



if(BUILD_TESTING)
  
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_download)



  ament_download(http://download.ros.org/data/ar_track_alvar/ar_track_alvar_4markers_tork_2017-02-08-11-21-14.bag
    FILENAME ar_track_alvar_4markers_tork_2017-02-08-11-21-14.bag
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}
    MD5 627aa0316bbfe4334e06023d7c2b4087
   )

  set(_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}")
  if(WIN32 AND CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE_DEBUG}")
  endif()


  # Test DisparityNode in launch test
  add_launch_test("test/test_ar_track_alvar.py"
    PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}"
  )
  #ament_add_gtest(test)
endif()
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

ament_package()