#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <memory>

#include <glog/logging.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/nav_sat_fix.h>

#include "localization_wrapper.h"



int main(int argc, char** argv) {
  // Force flush of the stdout buffer.
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  
  FLAGS_colorlogtostderr = true;

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<localization::LocalizationWrapper>(rclcpp::NodeOptions()));

  rclcpp::shutdown();

  return 0;
}
