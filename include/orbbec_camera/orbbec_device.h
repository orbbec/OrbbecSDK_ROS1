#pragma once

#include "libobsensor/hpp/Context.hpp"
#include "libobsensor/hpp/Device.hpp"
#include "libobsensor/hpp/Sensor.hpp"
#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "image_transport/camera_publisher.h"
#include "dynamic_reconfigure/server.h"
#include "orbbec_camera/SetLaserEnable.h"
#include "orbbec_camera/SetLdpEnable.h"
#include "orbbec_camera/SetFloorEnable.h"
#include "orbbec_camera/SetFanMode.h"
#include "orbbec_camera/OrbbecConfig.h"
#include "color_sensor.h"
#include "depth_sensor.h"
#include "ir_sensor.h"

class OrbbecDevice
{
  private:
    ros::NodeHandle& mNodeHandle;
    ros::NodeHandle& mPrivateNodeHandle;

    std::shared_ptr<ob::Device> mDevice;
    std::shared_ptr<ob::Sensor> mColorSensor;
    std::shared_ptr<ob::Sensor> mDepthSensor;
    std::shared_ptr<ob::Sensor> mIrSensor;

    std::shared_ptr<ColorSensor> mColorSensorNode;
    std::shared_ptr<DepthSensor> mDepthSensorNode;
    std::shared_ptr<IrSensor> mIrSensorNode;

    ros::ServiceServer mSetLaserEnableService;
    ros::ServiceServer mSetLdpEnableService;
    ros::ServiceServer mSetFloorEnableService;
    ros::ServiceServer mSetFanModeService;

    dynamic_reconfigure::Server<orbbec_camera::OrbbecConfig> mReconfigServer;

    bool setLaserEnableCallback(orbbec_camera::SetLaserEnableRequest& req, orbbec_camera::SetLaserEnableResponse& res);
    bool setFloorEnableCallback(orbbec_camera::SetFloorEnableRequest& req, orbbec_camera::SetFloorEnableResponse& res);
    bool setLdpEnableCallback(orbbec_camera::SetLdpEnableRequest& req, orbbec_camera::SetLdpEnableResponse& res);
    bool setFanModeCallback(orbbec_camera::SetFanModeRequest& req, orbbec_camera::SetFanModeResponse& res);

    void reconfigCallback(orbbec_camera::OrbbecConfig& config, uint32_t level);

  public:
    OrbbecDevice(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::shared_ptr<ob::Device> dev);
    ~OrbbecDevice();
};
