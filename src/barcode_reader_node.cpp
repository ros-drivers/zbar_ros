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
#include <string>
#include <functional>
#include <chrono>
#include "zbar_ros/barcode_reader_node.hpp"
#include "cv_bridge/cv_bridge.hpp"

using namespace std::chrono_literals;

namespace zbar_ros
{

BarcodeReaderNode::BarcodeReaderNode()
: Node("BarcodeReader")
{
  scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);


  camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "image", 10, std::bind(&BarcodeReaderNode::imageCb, this, std::placeholders::_1));

  barcode_pub_ = this->create_publisher<std_msgs::msg::String>("barcode", 10);

  throttle_ = this->declare_parameter<double>("throttle_repeated_barcodes", 0.0);
  RCLCPP_DEBUG(get_logger(), "throttle_repeated_barcodes : %f", throttle_);

  if (throttle_ > 0.0) {
    clean_timer_ = this->create_wall_timer(
      10s, std::bind(&BarcodeReaderNode::cleanCb, this));
  }
}

void BarcodeReaderNode::imageCb(sensor_msgs::msg::Image::ConstSharedPtr image)
{
  RCLCPP_DEBUG(get_logger(), "Image received on subscribed topic");

  cv_bridge::CvImageConstPtr cv_image;
  cv_image = cv_bridge::toCvShare(image, "mono8");

  zbar::Image zbar_image(cv_image->image.cols, cv_image->image.rows, "Y800", cv_image->image.data,
    cv_image->image.cols * cv_image->image.rows);
  scanner_.scan(zbar_image);

  auto it_start = zbar_image.symbol_begin();
  auto it_end = zbar_image.symbol_end();
  if (it_start != it_end) {
    // If there are barcodes in the image, iterate over all barcode readings from image
    for (zbar::Image::SymbolIterator symbol = it_start; symbol != it_end; ++symbol) {
      std::string barcode = symbol->get_data();
      RCLCPP_DEBUG(get_logger(), "Barcode detected with data: '%s'", barcode.c_str());

      // verify if repeated barcode throttling is enabled
      if (throttle_ > 0.0) {
        const std::lock_guard<std::mutex> lock(memory_mutex_);

        // check if barcode has been recorded as seen, and skip detection
        if (barcode_memory_.count(barcode) > 0) {
          // check if time reached to forget barcode
          if (now() > barcode_memory_.at(barcode)) {
            RCLCPP_DEBUG(get_logger(), "Memory timed out for barcode, publishing");
            barcode_memory_.erase(barcode);
          } else {
            // if timeout not reached, skip this reading
            continue;
          }
        }
        // record barcode as seen, with a timeout to 'forget'
        barcode_memory_.insert(
          std::make_pair(
            barcode,
            now() + rclcpp::Duration(std::chrono::duration<double>(throttle_))));
      }

      // publish barcode
      RCLCPP_DEBUG(get_logger(), "Publishing data as string");
      std_msgs::msg::String barcode_string;
      barcode_string.data = barcode;
      barcode_pub_->publish(barcode_string);
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "No barcode detected in image");
  }

  zbar_image.set_data(NULL, 0);
}


void BarcodeReaderNode::cleanCb()
{
  const std::lock_guard<std::mutex> lock(memory_mutex_);
  auto it = barcode_memory_.begin();
  while (it != barcode_memory_.end()) {
    if (now() > it->second) {
      RCLCPP_DEBUG(get_logger(), "Cleaned %s from memory", it->first.c_str());
      it = barcode_memory_.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace zbar_ros
