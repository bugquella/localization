#pragma once

// ROS includes
//#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <fstream>
#include <memory>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "imu_gps_localizer/imu_gps_localizer.h"


// cpplint: c++ system headers
#include <memory>

namespace localization {

class LocalizationWrapper final : public rclcpp::Node {
 public:
 
    explicit LocalizationWrapper(const rclcpp::NodeOptions & options);

    ~LocalizationWrapper() override;

    LocalizationWrapper(LocalizationWrapper &&c) = delete;
    LocalizationWrapper &operator=(LocalizationWrapper &&c) = delete;
    LocalizationWrapper(const LocalizationWrapper &c) = delete;
    LocalizationWrapper &operator=(const LocalizationWrapper &c) = delete;

    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg_ptr);
    
    void GpsPositionCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg_ptr);
private:

    void LogState(const ImuGpsLocalization::State& state);
    void LogGps(const ImuGpsLocalization::GpsPositionDataPtr gps_data);

    void ConvertStateToRosTopic(const ImuGpsLocalization::State& state);
    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_position_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr state_pub_;
    



    std::ofstream file_state_;
    std::ofstream file_gps_;

    nav_msgs::msg::Path ros_path_;

    std::unique_ptr<ImuGpsLocalization::ImuGpsLocalizer> imu_gps_localizer_ptr_;
    
    
private:
    rclcpp::Logger logger_ = this->get_logger();

    
 
};

}  // namespace camera_node


