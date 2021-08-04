cmake_minimum_required(VERSION 3.5)
project(ar_track_alvar)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)
if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
     set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
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
find_package(ar_track_alvar_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosbag2_bag_v2_plugins REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS common io)
find_package(Eigen3 REQUIRED)
find_package(tinyxml_vendor REQUIRED)

include_directories(include)

# Kinect filtering code
set(KINECT_FILTERING_TARGETS kinect_filtering medianFilter)
set(ALVAR_TARGETS ar_track_alvar individualMarkers individualMarkersNoKinect trainMarkerBundle findMarkerBundles findMarkerBundlesNoKinect createMarker)

set(dependencies
  OpenCV
  tf2_ros 
  tf2
  pcl_ros
  pcl_conversions 
  std_msgs 
  tinyxml_vendor
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
                    ${TinyXML_INCLUDE_DIRS}
                    ${PCL_COMMON_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/Bitset.cpp
    src/Camera.cpp
    src/CaptureDevice.cpp
    src/CaptureFactory.cpp
    src/CaptureFactory_unix.cpp
    src/ConnectedComponents.cpp
    src/CvTestbed.cpp
    src/DirectoryIterator.cpp
    src/DirectoryIterator_unix.cpp
    src/Draw.cpp
    src/FileFormatUtils.cpp
    src/Filter.cpp
    src/Kalman.cpp
    src/Line.cpp
    src/Marker.cpp
    src/MarkerDetector.cpp
    src/MultiMarker.cpp
    src/MultiMarkerBundle.cpp
    src/MultiMarkerInitializer.cpp
    src/Mutex.cpp
    src/Mutex_unix.cpp
    src/Optimization.cpp
    src/Plugin.cpp
    src/Plugin_unix.cpp
    src/Pose.cpp
    src/Rotation.cpp
    src/Threads.cpp
    src/Threads_unix.cpp
    src/Util.cpp
)
    
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} tinyxml)
ament_target_dependencies(ar_track_alvar ${dependencies})

add_library(kinect_filtering src/kinect_filtering.cpp)
ament_target_dependencies(kinect_filtering ${dependencies})


add_library(medianFilter src/medianFilter.cpp)
target_link_libraries(medianFilter ar_track_alvar ${TinyXML_LIBRARIES})
ament_target_dependencies(medianFilter ${dependencies})

add_executable(individualMarkers nodes/IndividualMarkers.cpp)
target_link_libraries(individualMarkers ar_track_alvar kinect_filtering  ${TinyXML_LIBRARIES})
ament_target_dependencies(individualMarkers ${dependencies})

add_executable(individualMarkersNoKinect nodes/IndividualMarkersNoKinect.cpp)
target_link_libraries(individualMarkersNoKinect ar_track_alvar)
ament_target_dependencies(individualMarkersNoKinect ${dependencies})

add_executable(trainMarkerBundle nodes/TrainMarkerBundle.cpp)
target_link_libraries(trainMarkerBundle ar_track_alvar  ${TinyXML_LIBRARIES})
ament_target_dependencies(trainMarkerBundle ${dependencies})


add_executable(findMarkerBundles nodes/FindMarkerBundles.cpp)
target_link_libraries(findMarkerBundles ar_track_alvar kinect_filtering medianFilter  ${TinyXML_LIBRARIES})
ament_target_dependencies(findMarkerBundles ${dependencies})

add_executable(findMarkerBundlesNoKinect nodes/FindMarkerBundlesNoKinect.cpp)
target_link_libraries(findMarkerBundlesNoKinect ar_track_alvar  ${TinyXML_LIBRARIES})
ament_target_dependencies(findMarkerBundlesNoKinect ${dependencies})

add_executable(createMarker src/SampleMarkerCreator.cpp)
target_link_libraries(createMarker ar_track_alvar  ${TinyXML_LIBRARIES})
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
  
  set(_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}")
  if(WIN32 AND CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE_DEBUG}")
  endif()

  # Test bag playing (if this fails all other tests will fail) 
  add_launch_test("test/test_ar_track_alvar_bag.py"
    PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}"
  )

  # Test Individual Markers in launch test
  add_launch_test("test/test_ar_track_alvar_individual_markers.py"
    PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}"
  )

  # Legacy Tests
  add_launch_test("test/test_ar_legacy.py"
    PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}"
  )

  # Test Launch Files
  add_launch_test("test/test_launch_files.py"
    PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}"
  )

endif()

ament_package()