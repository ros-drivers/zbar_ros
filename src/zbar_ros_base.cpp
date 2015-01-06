#include <zbar_ros/zbar_ros_base.h>
#include <std_msgs/String.h>

namespace zbar_ros{

  ZbarBase::ZbarBase(ros::NodeHandle nh, ros::NodeHandle private_nh)
      : nh_(nh), private_nh_(private_nh)
  {
    scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
    barcode_pub_ = nh.advertise<std_msgs::String>("barcode", 10,
        boost::bind(&ZbarBase::connectCb, this),
        boost::bind(&ZbarBase::disconnectCb, this));

  }

  void ZbarBase::connectCb() {
    if (!camera_sub_ && barcode_pub_.getNumSubscribers() > 0) {
      ROS_DEBUG("Connecting to camera topic.");
      camera_sub_ = nh_.subscribe("image", 10, &ZbarBase::imageCb, this);
    }
  }

  void ZbarBase::disconnectCb() {
    if (barcode_pub_.getNumSubscribers() == 0) {
      ROS_DEBUG("Unsubscribing from camera topic.");
      camera_sub_.shutdown();
    }
  }

  void ZbarBase::imageCb(const sensor_msgs::ImageConstPtr& image) {
    cv_bridge::CvImageConstPtr cv_image;
    cv_image = cv_bridge::toCvShare(image, "mono16");

    zbar::Image zbar_image(cv_image->image.cols, cv_image->image.rows, "Y800", cv_image->image.data, cv_image->image
        .cols * cv_image->image.rows);
    scanner_.scan(zbar_image);

    for(zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin();
        symbol != zbar_image.symbol_end();
        ++symbol) {

      std_msgs::String barcode_string;
      barcode_string.data = symbol->get_data();
      barcode_pub_.publish(barcode_string);
    }

  }

}