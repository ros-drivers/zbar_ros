#include <zbar_ros/zbar_ros_base.h>
#include <ros/ros.h>

int main(int argc, char *argv[])
{

  ros::init(argc, argv, "barcode_reader");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  zbar_ros::ZbarBase node(nh, private_nh);
  ros::spin();

  return 0;

}