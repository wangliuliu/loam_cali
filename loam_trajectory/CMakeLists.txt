cmake_minimum_required(VERSION 2.8.3)
project(loam_velodyne)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")

set(IMUPOSE_SOURCE_FILES         
    src/imupose       
    src/imuEstimate.cpp 
    src/imustate.cpp	
)

set(IMUPOSE_HEADER_FILES        
    include/imu_pose/imupose.h  
    include/imu_pose/imustate.h  
)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
        ${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL OpenCV
  INCLUDE_DIRS include
)

add_executable(Imupose ${IMUPOSE_SOURCE_FILES} ${IMUPOSE_HEADER_FILES})
target_link_libraries(Imupose ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${EIGEN3_LIBS})
add_dependencies(Imupose imu_pose_generate_messages_cpp)

add_executable(scanRegistration src/scanRegistration.cpp)
target_link_libraries(scanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(laserOdometry src/laserOdometry.cpp)
target_link_libraries(laserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(laserMapping src/laserMapping.cpp)
target_link_libraries(laserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(transformMaintenance src/transformMaintenance.cpp)
target_link_libraries(transformMaintenance ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(transCoordinate src/transCoordinate.cpp)
target_link_libraries(transCoordinate ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(map_visualization src/map_visualization.cpp)
target_link_libraries(map_visualization ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(normal_distributions_transform src/normal_distributions_transform.cpp)
target_link_libraries(normal_distributions_transform ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(normal_distributions_transform1 src/normal_distributions_transform1.cpp)
target_link_libraries(normal_distributions_transform1 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # TODO: Download test data
  catkin_download_test_data(${PROJECT_NAME}_test_data.tar.gz
    https://dl.dropboxusercontent.com/s/y4hn486461tfmpm/velodyne_loam_test_data.tar.gz
    MD5 3d5194e6981975588b7a93caebf79ba4)
  add_custom_target(${PROJECT_NAME}_test_data
    COMMAND ${CMAKE_COMMAND} -E tar -xzf velodyne_loam_test_data.tar.gz
    DEPENDS ${PROJECT_NAME}_test_data.tar.gz)
  configure_file(tests/loam.test.in
                 ${PROJECT_BINARY_DIR}/test/loam.test)
  add_rostest(${PROJECT_BINARY_DIR}/test/loam.test
    DEPENDENCIES
      ${PROJECT_NAME}_test_data
      scanRegistration
      laserOdometry
      laserMapping
      transformMaintenance)
endif()


