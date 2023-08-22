#include "ros/ros.h"
#include "orbbec_camera/ob_camera_node_driver.h"
#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>


namespace orbbec_camera {
    class OBCameraNodelet : public nodelet::Nodelet {
    public:
        OBCameraNodelet() {}

        ~OBCameraNodelet() {}

    private:
         void onInit() override {
            ros::NodeHandle nh = getNodeHandle();
            ros::NodeHandle nh_private = getPrivateNodeHandle();
            ob_camera_node_driver_.reset(new OBCameraNodeDriver(nh, nh_private));
        }

        boost::shared_ptr<OBCameraNodeDriver> ob_camera_node_driver_;
    };
}  // namespace orbbec_camera


PLUGINLIB_EXPORT_CLASS(orbbec_camera::OBCameraNodelet, nodelet::Nodelet)