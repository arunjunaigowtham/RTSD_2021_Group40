// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "./policy_maps.hpp"

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {    
    parse_parameters();
    this->declare_parameter<int>("threshold", 40);

    auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(
        // The history policy determines how messages are saved until taken by
        // the reader.
        // KEEP_ALL saves all messages until they are taken.
        // KEEP_LAST enforces a limit on the number of messages that are saved,
        // specified by the "depth" parameter.
        history_policy_,
        // Depth represents how many messages to store in history when the
        // history policy is KEEP_LAST.
        depth_
    ));
    qos.reliability(reliability_policy_);

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image", qos, std::bind(&MinimalSubscriber::topic_callback, this, _1));

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("imageProc", qos);
    light_publisher_ = this->create_publisher<std_msgs::msg::String>("lightStatus", 10);

  }


private:
  void parse_parameters()
  {
    // Parse 'reliability' parameter
    rcl_interfaces::msg::ParameterDescriptor reliability_desc;
    reliability_desc.description = "Reliability QoS setting for the image subscription";
    reliability_desc.additional_constraints = "Must be one of: ";
    for (auto entry : image_tools::name_to_reliability_policy_map) {
      reliability_desc.additional_constraints += entry.first + " ";
    }
    const std::string reliability_param = this->declare_parameter(
      "reliability", "reliable", reliability_desc);
    auto reliability = image_tools::name_to_reliability_policy_map.find(reliability_param);
    if (reliability == image_tools::name_to_reliability_policy_map.end()) {
      std::ostringstream oss;
      oss << "Invalid QoS reliability setting '" << reliability_param << "'";
      throw std::runtime_error(oss.str());
    }
    reliability_policy_ = reliability->second;

    // Parse 'history' parameter
    rcl_interfaces::msg::ParameterDescriptor history_desc;
    history_desc.description = "History QoS setting for the image subscription";
    history_desc.additional_constraints = "Must be one of: ";
    for (auto entry : image_tools::name_to_history_policy_map) {
      history_desc.additional_constraints += entry.first + " ";
    }
    const std::string history_param = this->declare_parameter(
      "history", image_tools::name_to_history_policy_map.begin()->first, history_desc);
    auto history = image_tools::name_to_history_policy_map.find(history_param);
    if (history == image_tools::name_to_history_policy_map.end()) {
      std::ostringstream oss;
      oss << "Invalid QoS history setting '" << history_param << "'";
      throw std::runtime_error(oss.str());
    }
    history_policy_ = history->second;

    // Declare and get remaining parameters
    depth_ = this->declare_parameter("depth", 10);
    show_image_ = this->declare_parameter("show_image", true);
    window_name_ = this->declare_parameter("window_name", "");
  }

  
  // void topic_callback(const std_msgs::msg::String::ConstSharedPtr msg) const
  void topic_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) const
  {
    // RCLCPP_INFO(this->get_logger(), "First pixel values (0,0) (bgr): [%d, %d, %d]", msg->data[0], msg->data[1], msg->data[2]);

    /* Convert msg to cv::Mat via cv_bridge::CvImagePtr */
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg);
    cv::Mat cvImg = cv_ptr->image;

    // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60) {
    //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
    // }

    /* Image processing */
    rclcpp::Parameter int_param = this->get_parameter("threshold");
    int darknessThreshold_ = int_param.as_int();
    auto message = std_msgs::msg::String();
    
    cv::Mat cvImgOut;

    cv::Mat greyMat;
    cv::cvtColor(cvImg, greyMat, CV_BGR2GRAY);                              // convert to grayscale
    cv::Scalar m = cv::mean(greyMat);
    RCLCPP_INFO(this->get_logger(), "Mean (grayscale) pixel value: %f", m[0]);

    if (m[0] < darknessThreshold_) {
      message.data = "Light is OFF";
    } else {
      message.data = "Light is ON";
    }
    RCLCPP_INFO(this->get_logger(), "Threshold parameter = %d", darknessThreshold_);
    RCLCPP_INFO(this->get_logger(), "Publishing:  '%s'", message.data.c_str());

    /* Convert from cv::Mat back to sensor_msgs:msg:Image*/
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Node::now();
    cv_bridge::CvImage img_bridge;
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, cvImgOut);   // output GRAYSCALE (mono8) image
    sensor_msgs::msg::Image img_msg;
    img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::msg::Image

    // publish to Topic
    publisher_->publish(img_msg);
    light_publisher_->publish(message);
    // publisher_->publish(*cv_ptr->toImageMsg());
    //publisher_->publish(*msg);

  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr light_publisher_;

  size_t depth_ = rmw_qos_profile_default.depth;
  rmw_qos_reliability_policy_t reliability_policy_ = rmw_qos_profile_default.reliability;
  rmw_qos_history_policy_t history_policy_ = rmw_qos_profile_default.history;
  bool show_image_ = true;
  std::string topic_ = "image";
  std::string window_name_;
  // int darknessThreshold_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

