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

#include "zbar_ros/zbar_ros_base.h"
#include "std_msgs/String.h"
#include <string>

namespace zbar_ros
{

  ZbarBase::ZbarBase(ros::NodeHandle nh, ros::NodeHandle private_nh)
      : nh_(nh), private_nh_(private_nh)
  {
    scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
    barcode_pub_ = nh.advertise<std_msgs::String>("barcode", 10,
        boost::bind(&ZbarBase::connectCb, this),
        boost::bind(&ZbarBase::disconnectCb, this));
    private_nh_.param<double>("throttle_repeated_barcodes", throttle_, 0.0);
  }

  void ZbarBase::connectCb()
  {
    ROS_INFO_STREAM("connectCb " << barcode_pub_.getNumSubscribers());
    if (!camera_sub_ && barcode_pub_.getNumSubscribers() > 0)
    {
      ROS_INFO("Connecting to camera topic.");
      camera_sub_ = nh_.subscribe("image", 10, &ZbarBase::imageCb, this);
    }
  }

  void ZbarBase::disconnectCb()
  {
    ROS_INFO_STREAM("disconnectCb " << barcode_pub_.getNumSubscribers());
    if (barcode_pub_.getNumSubscribers() == 0)
    {
      ROS_INFO("Unsubscribing from camera topic.");
      camera_sub_.shutdown();
    }
  }

  void ZbarBase::imageCb(const sensor_msgs::ImageConstPtr &image)
  {
    cv_bridge::CvImageConstPtr cv_image;
    cv_image = cv_bridge::toCvShare(image, "mono16");

    zbar::Image zbar_image(cv_image->image.cols, cv_image->image.rows, "Y800", cv_image->image.data, cv_image->image
        .cols * cv_image->image.rows);
    scanner_.scan(zbar_image);

    // iterate over all barcode readings from image
    for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin();
         symbol != zbar_image.symbol_end();
         ++symbol)
    {
      std::string barcode = symbol->get_data();

      // verify if repeated barcode throttling is enabled
      if (throttle_ > 0.0)
      {
        // check if barcode has been recorded as seen, and skip detection
        if (barcode_memory_.count(barcode) != 0)
        {
          // check if time reached to forget barcode
          if (ros::Time::now() > barcode_memory_.at(barcode))
          {
            barcode_memory_.erase(barcode);
          }
          else
          {
            // if timeout not reached, skip this reading
            continue;
          }
        }
        // record barcode as seen, with a timeout to 'forget'
        barcode_memory_.insert(std::make_pair(barcode, ros::Time::now() + ros::Duration(throttle_)));
      }

      // publish barcode
      std_msgs::String barcode_string;
      barcode_string.data = barcode;
      barcode_pub_.publish(barcode_string);
    }
  }
}  // namespace zbar_ros
