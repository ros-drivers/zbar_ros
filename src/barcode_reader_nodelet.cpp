#include <zbar_ros/zbar_ros_base.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace zbar_ros{

class BarcodeReaderNodelet : public nodelet::Nodelet
{

public:
  virtual void onInit()
  {
    nodelet.reset(new ZbarBase(getNodeHandle(), getPrivateNodeHandle()));
  };

  boost::shared_ptr<ZbarBase> nodelet;

};

}

PLUGINLIB_DECLARE_CLASS(zbar_ros, BarcodeReaderNodelet, zbar_ros::BarcodeReaderNodelet, nodelet::Nodelet);
