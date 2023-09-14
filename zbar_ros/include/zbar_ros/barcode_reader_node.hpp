/**
*
*  \author     Paul Bovbel <pbovbel@clearpathrobotics.com>
*  \copyright  Copyright (c) 2014, Clearpath Robotics, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Clearpath Robotics, Inc. nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Please send comments, questions, or patches to code@clearpathrobotics.com
*
*/
#ifndef ZBAR_ROS__BARCODE_READER_NODE_HPP_
#define ZBAR_ROS__BARCODE_READER_NODE_HPP_

#include <string>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "./zbar.h"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "zbar_ros_interfaces/msg/symbol.hpp"

namespace zbar_ros
{

class BarcodeReaderNode : public rclcpp::Node
{
public:
  BarcodeReaderNode();

private:
  void imageCb(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void cleanCb();

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Publisher<zbar_ros_interfaces::msg::Symbol>::SharedPtr symbol_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr barcode_pub_;  // DEPRECATED
  zbar::ImageScanner scanner_;

  rclcpp::TimerBase::SharedPtr clean_timer_;
  std::mutex memory_mutex_;
  std::unordered_map<std::string, rclcpp::Time> barcode_memory_;
  double throttle_;
};

}  // namespace zbar_ros

#endif  // ZBAR_ROS__BARCODE_READER_NODE_HPP_
