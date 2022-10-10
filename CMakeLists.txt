cmake_minimum_required(VERSION 2.8.3)
project(task2)

find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV)
find_package(zbar_ros)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	cv_bridge
	sensor_msgs
	nav_msgs
	std_msgs
	geometry_msgs
	tf2
	tf2_geometry_msgs
	message_generation
	pcl_conversions
  	pcl_msgs
	zbar_ros
)

## Add include directories
include_directories(
	${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${ZBAR_INCLUDE_DIRS}
)

## Add link directories
link_directories(
  ${Boost_LIBRARY_DIRS}
  ${Eigen_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
  ${ZBAR_LIBRARY_DIRS}
)

link_libraries(${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${ZBAR_LIBRARIES})

add_message_files(DIRECTORY msg
  FILES
  xy.msg
  ColorRecognition.msg

)

add_service_files(DIRECTORY srv
  FILES
  goal.srv
  qr.srv
  knn.srv
  ColorRecognition0.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)



#catkin_package( CATKIN_DEPENDS message_runtime std_msgs sensor_msgs )

## Declare the catkin package
catkin_package(
  LIBRARIES
    pcl_ros_filters
    pcl_ros_io
    pcl_ros_tf
  CATKIN_DEPENDS
    dynamic_reconfigure
    pcl_conversions
    pcl_msgs
    roscpp
    sensor_msgs
    std_msgs
  DEPENDS
    Boost
    Eigen
    PCL
    ZBAR
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_executable(circle_detection src/circle_detection.cpp)
target_link_libraries(circle_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(goal_processor src/goal_processor.cpp)
target_link_libraries(goal_processor ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(processing_node src/processing_node.cpp)
target_link_libraries(processing_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(ring_locations src/ring_locations.cpp)
target_link_libraries(ring_locations ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(ring_collector src/ring_collector.cpp)
target_link_libraries(ring_collector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(ring_detection src/ring_detection.cpp)
target_link_libraries(ring_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(processing_node_rings src/processing_node_rings.cpp)
target_link_libraries(processing_node_rings ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(navigation src/navigation.cpp)
target_link_libraries(navigation ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(laser_node src/laser_node.cpp)
target_link_libraries(laser_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(record_data src/record_data.cpp)
target_link_libraries(record_data ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#add_executable(qr_service src/qr_service.cpp)
#target_link_libraries(qr_service ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${ZBAR_LIBRARIES})
