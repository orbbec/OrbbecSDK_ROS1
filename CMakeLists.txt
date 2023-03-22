cmake_minimum_required(VERSION 3.0.2)
project(orbbec_camera)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}  -fPIC -g3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fPIC -g3")
set(CMAKE_BUILD_TYPE "Release")
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(catkin REQUIRED
  camera_info_manager
  cv_bridge
  dynamic_reconfigure
  image_geometry
  image_transport
  message_filters
  message_generation
  roscpp
  sensor_msgs
  std_srvs
  tf2
  tf2_ros
  )

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(Boost REQUIRED)

find_package(PkgConfig REQUIRED)

pkg_search_module(GLOG REQUIRED libglog)

if (NOT GLOG_FOUND)
  message(FATAL_ERROR "glog is not found")
endif ()

execute_process(COMMAND uname -m OUTPUT_VARIABLE MACHINES)
execute_process(COMMAND getconf LONG_BIT OUTPUT_VARIABLE MACHINES_BIT)
message(STATUS "ORRBEC Machine : ${MACHINES}")
message(STATUS "ORRBEC Machine Bits : ${MACHINES_BIT}")
if ((${MACHINES} MATCHES "x86_64") AND (${MACHINES_BIT} MATCHES "64"))
  set(HOST_PLATFORM "x64")
elseif (${MACHINES} MATCHES "arm")
  set(HOST_PLATFORM "arm")
elseif ((${MACHINES} MATCHES "aarch64") AND (${MACHINES_BIT} MATCHES "64"))
  set(HOST_PLATFORM "arm64")
elseif ((${MACHINES} MATCHES "aarch64") AND (${MACHINES_BIT} MATCHES "32"))
  set(HOST_PLATFORM "arm")
endif ()

set(ORBBEC_LIBS ${CMAKE_CURRENT_SOURCE_DIR}/SDK/lib/${HOST_PLATFORM})
set(ORBBEC_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/SDK/include/)

add_message_files(
  FILES
  DeviceInfo.msg
  Extrinsics.msg
  Metadata.msg
)

add_service_files(
  FILES
  GetBool.srv
  SetBool.srv
  GetCameraInfo.srv
  GetCameraParams.srv
  GetDeviceInfo.srv
  GetInt32.srv
  GetString.srv
  SetInt32.srv
  SetString.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  camera_info_manager
  cv_bridge
  dynamic_reconfigure
  image_geometry
  image_transport
  message_filters
  message_runtime
  roscpp
  sensor_msgs
  std_srvs
  tf2
  tf2_ros
)

link_directories(${ORBBEC_LIBS}
  ${catkin_LINK_DIRS}
  ${OpenCV_LIBS})

add_library(${PROJECT_NAME}
  src/d2c_viewer.cpp
  src/ob_camera_node.cpp
  src/ob_camera_node_driver.cpp
  src/ros_sensor.cpp
  src/ros_service.cpp
  src/utils.cpp
  src/ros_setup.cpp
  )

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDED_DIRS}
  ${ORBBEC_INCLUDE_DIR}
  )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  ${OpenCV_LIBS}
  ${GLOG_LIBRARIES}
  -lOrbbecSDK
  -L${ORBBEC_LIBS}
  )

add_executable(${PROJECT_NAME}_node
  src/main.cpp
  )

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDED_DIRS}
  ${ORBBEC_INCLUDE_DIR}
  )

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  ${OpenCV_LIBS}
  ${GLOG_LIBRARIES}
  -lOrbbecSDK
  -L${ORBBEC_LIBS}
  )

add_executable(list_devices_node
  src/list_devices_node.cpp
  )

target_include_directories(list_devices_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDED_DIRS}
  )

target_link_libraries(list_devices_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  ${OpenCV_LIBS}
  ${GLOG_LIBRARIES}
  -lOrbbecSDK
  -L${ORBBEC_LIBS}
  )

add_executable(ob_cleanup_shm_node
  src/ob_cleanup_shm.cpp
  )

target_include_directories(ob_cleanup_shm_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDED_DIRS}
  )

target_link_libraries(ob_cleanup_shm_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  ${OpenCV_LIBS}
  ${GLOG_LIBRARIES}
  -lOrbbecSDK
  -L${ORBBEC_LIBS}
  )

add_executable(list_depth_work_mode_node
  src/list_depth_work_mode.cpp
  )

target_include_directories(list_depth_work_mode_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDED_DIRS}
  )

target_link_libraries(list_depth_work_mode_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  ${OpenCV_LIBS}
  ${GLOG_LIBRARIES}
  -lOrbbecSDK
  -L${ORBBEC_LIBS}
  )
add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_generate_messages_cpp)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
install(TARGETS ${PROJECT_NAME}
  ${PROJECT_NAME}_node
  list_devices_node
  ob_cleanup_shm_node
  list_depth_work_mode_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )


install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
  )

install(DIRECTORY
  ${ORBBEC_INCLUDE_DIR}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
  )


install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY
  ${ORBBEC_LIBS}/
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/
  FILES_MATCHING PATTERN "*.so"
  PATTERN "*.so.*"
  )
