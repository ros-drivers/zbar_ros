#ifndef ZBAR_ROS_BASE_H
#define ZBAR_ROS_BASE_H

#include <ros/ros.h>
#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <zbar.h>
#include <tf/transform_listener.h>

namespace zbar_ros
{

  class ZbarBase
  {
  public:

    ZbarBase(ros::NodeHandle nh, ros::NodeHandle private_nh);

  private:

    void connectCb();

    void disconnectCb();

    void imageCb(const sensor_msgs::ImageConstPtr &image);


    ros::NodeHandle nh_, private_nh_;
    tf::TransformListener tf_;
    ros::Subscriber camera_sub_;
    ros::Publisher barcode_pub_;
    zbar::ImageScanner scanner_;

  };

}

#endif  // ZBAR_ROS_BASE_H