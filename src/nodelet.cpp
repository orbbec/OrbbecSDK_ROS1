#include "orbbec_device_manager.h"
#include <nodelet/nodelet.h>

class OrbbecNodelet : public nodelet::Nodelet
{
  public:
    OrbbecNodelet(){};
    ~OrbbecNodelet()
    {
    }

  private:
    virtual void onInit()
    {
        lp.reset(new OrbbecDeviceManager(getNodeHandle(), getPrivateNodeHandle()));
    };

    boost::shared_ptr<OrbbecDeviceManager> lp;
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(OrbbecNodelet, nodelet::Nodelet)
