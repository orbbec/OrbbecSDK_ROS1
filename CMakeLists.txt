cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

project(orbbec_camera)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  image_transport
  sensor_msgs
  dynamic_reconfigure
)

add_message_files(
  FILES
  DeviceInfo.msg
)

add_service_files(
  FILES
  GetDeviceList.srv
  GetCameraInfo.srv
  GetExposure.srv
  SetExposure.srv
  GetGain.srv
  SetGain.srv
  GetWhiteBalance.srv
  SetWhiteBalance.srv
  SetAutoExposure.srv
  SetAutoWhiteBalance.srv
  SetLaserEnable.srv
  SetLdpEnable.srv
  SetFloorEnable.srv
  SetFanMode.srv
  GetVersion.srv
  EnableStream.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

generate_dynamic_reconfigure_options(
  cfg/Orbbec.cfg
)

catkin_package(
 CATKIN_DEPENDS roscpp
 rospy
 std_msgs
 message_runtime
 image_transport
 sensor_msgs
 dynamic_reconfigure
)


set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

find_package(ORBBEC_SDK)

if(!ORBBEC_SDK_FOUND)
message("Could not find Orbbec SDK!")
endif()

add_subdirectory(libjpeg)
set(JPEG_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libjpeg")
set(JPEG_LIBRARY jpeg-static)
set(JPEG_FOUND TRUE)

add_subdirectory(libyuv)
set(YUV_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libyuv/include")

set(orbbec_camera_INCLUDE_DIR "include/orbbec_camera")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${orbbec_camera_INCLUDE_DIR}
  ${ORBBEC_SDK_INCLUDE_DIR}
  ${YUV_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}_nodelet
  src/nodelet.cpp
  src/orbbec_device_manager.cpp
  src/orbbec_device.cpp
  src/color_sensor.cpp
  src/depth_sensor.cpp
  src/ir_sensor.cpp
)

add_dependencies(${PROJECT_NAME}_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_nodelet
  ${catkin_LIBRARIES}
  ${ORBBEC_SDK_LIBRARIES}
  yuv
)

add_executable(${PROJECT_NAME}_node 
  src/main.cpp
  src/orbbec_device_manager.cpp
  src/orbbec_device.cpp
  src/color_sensor.cpp
  src/depth_sensor.cpp
  src/ir_sensor.cpp
)


add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${ORBBEC_SDK_LIBRARIES}
  yuv
)

add_executable(device_list_node src/orbbec_device_list.cpp)

add_dependencies(device_list_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(device_list_node 
  ${catkin_LIBRARIES}
  ${ORBBEC_SDK_LIBRARIES}
)
