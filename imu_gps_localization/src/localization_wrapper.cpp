#include "localization_wrapper.h"

#include <iomanip>

#include <glog/logging.h>

#include "imu_gps_localizer/base_type.h"
#include <iostream>
using namespace std;



using std::placeholders::_1;
namespace localization {

// 2, LocalizationWrapper 构造函数；
LocalizationWrapper::LocalizationWrapper(const rclcpp::NodeOptions & options) : rclcpp::Node("imu_gps_localization_node", options) {

    RCLCPP_INFO(logger_, "imu_gps_localization ROS2 arg:");
    
    // Load configs.
    double acc_noise, gyro_noise, acc_bias_noise, gyro_bias_noise;      // 2.1.1 读入 ESKF 需要用到的噪声参数；
    acc_noise = this->declare_parameter("acc_noise", 1e-2);             // a_n;  // noise 一般是指白噪声；
    gyro_noise = this->declare_parameter("gyro_noise", 1e-4);           // w_n;
    acc_bias_noise = this->declare_parameter("acc_bias_noise", 1e-6);   // a_b;  // bias 是随时间缓慢变化的噪声；
    gyro_bias_noise = this->declare_parameter("gyro_bias_noise", 1e-8); // w_b;

    
    double x, y, z;
    x = this->declare_parameter("I_p_Gps_x", 0.0);            
    y = this->declare_parameter("I_p_Gps_y", 0.0);   
    z = this->declare_parameter("I_p_Gps_z", 0.0);  
    const Eigen::Vector3d I_p_Gps(x, y, z);  // 2.1.2 IMU坐标系下的GPS坐标；同理，G_p_I：全局坐标系下IMU的位置；G_R_I：全局坐标系下IMU的姿态；G_v_I：全局坐标系下IMU的速度；

    std::string log_folder;
    log_folder = this->declare_parameter("log_folder", "imu_gps_localization");  // 2.1.3 设置日志保存路径；
    
    RCLCPP_INFO(logger_, "acc_noise:%f", acc_noise);
    RCLCPP_INFO(logger_, "gyro_noise: %f", gyro_noise);
    RCLCPP_INFO(logger_, "acc_bias_noise: %f", acc_bias_noise);
    RCLCPP_INFO(logger_, "gyro_bias_noise: %f", gyro_bias_noise);
    RCLCPP_INFO(logger_, "x: %f", x);
    RCLCPP_INFO(logger_, "y : %f", y );
    RCLCPP_INFO(logger_, "z: %f", z);
    RCLCPP_INFO(logger_, "log_folder: %s", log_folder.c_str());

    // Log.
    file_state_.open(log_folder + "/state.csv");
    file_gps_.open(log_folder +"/gps.csv");
    
    
    // Initialization imu gps localizer.
    // 2.2 初始化 ImuGpsLocalizer 对象，用于ESKF的预测（由IMU负责）和更新（由GPS负责）；
    imu_gps_localizer_ptr_ = 
        std::make_unique<ImuGpsLocalization::ImuGpsLocalizer>(acc_noise, gyro_noise,
                                                              acc_bias_noise, gyro_bias_noise,
                                                              I_p_Gps);  


    // Subscribe topics.
    // 2.3.1 接收imu消息，转成ImuGpsLocalization::ImuData结构体类型，并用 imu_gps_localizer 处理IMU数据，并保存IMU数据日志；
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10, std::bind(&LocalizationWrapper::ImuCallback, this, _1)); 
     // 2.3.2 接GPS数据并转成ImuGpsLocalization::GpsPositionData类型，用 imu_gps_localizer 处理GPS数据，并保存GPS数据日志；
    gps_position_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/fix", 10, std::bind(&LocalizationWrapper::GpsPositionCallback, this, _1));
    state_pub_ = this->create_publisher<nav_msgs::msg::Path>("fused_path", 10);// 发布位姿路径；
     
}




LocalizationWrapper::~LocalizationWrapper() {

}


void LocalizationWrapper::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg_ptr) { // 2.3.1
    ImuGpsLocalization::ImuDataPtr imu_data_ptr = std::make_shared<ImuGpsLocalization::ImuData>();
    imu_data_ptr->timestamp = static_cast<double>(imu_msg_ptr->header.stamp.sec) + 1e-9*static_cast<double>(imu_msg_ptr->header.stamp.nanosec);

    imu_data_ptr->acc << imu_msg_ptr->linear_acceleration.x, 
                         imu_msg_ptr->linear_acceleration.y,
                         imu_msg_ptr->linear_acceleration.z;
    imu_data_ptr->gyro << imu_msg_ptr->angular_velocity.x,
                          imu_msg_ptr->angular_velocity.y,
                          imu_msg_ptr->angular_velocity.z;
    
    ImuGpsLocalization::State fused_state;
    const bool ok = imu_gps_localizer_ptr_->ProcessImuData(imu_data_ptr, &fused_state); // 2.3.1 处理IMU数据，得到融合后的状态 fused_state;；==> 3, imu_gps_localizer.cpp, ProcessImuData
    if (!ok) {
        return;
    }

    // Publish fused state.
    ConvertStateToRosTopic(fused_state);  // 将状态结构体类数据型转换成ros对应的消息类型
    state_pub_->publish(ros_path_);  // 发布消息；

    // Log fused state.
    LogState(fused_state);  // 保存 fused_state 轨迹数据日志；
}

void LocalizationWrapper::GpsPositionCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg_ptr) {  // 2.3.2
    // Check the gps_status.
    if (gps_msg_ptr->status.status != 2) {  // 在 fix 消息中，GPS数据的 status 表示卫星定位状态信息；（-1，无法确定位置；0，未能精准定位；1，with satellite-based augmentation；2，with ground-based augmentation）
        LOG(WARNING) << "[GpsCallBack]: Bad gps message!";
        return;
    }


    ImuGpsLocalization::GpsPositionDataPtr gps_data_ptr = std::make_shared<ImuGpsLocalization::GpsPositionData>();
    gps_data_ptr->timestamp = static_cast<double>(gps_msg_ptr->header.stamp.sec) + 1e-9*static_cast<double>(gps_msg_ptr->header.stamp.nanosec);
    
    gps_data_ptr->lla << gps_msg_ptr->latitude,
                         gps_msg_ptr->longitude,
                         gps_msg_ptr->altitude;
    gps_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(gps_msg_ptr->position_covariance.data());

    imu_gps_localizer_ptr_->ProcessGpsPositionData(gps_data_ptr);  // 2.3.2 处理GPS数据，用于状态更新；==> 3, imu_gps_localizer.cpp, ProcessGpsPositionData

    LogGps(gps_data_ptr);  // 保存GPS数据日志；
}


void LocalizationWrapper::LogState(const ImuGpsLocalization::State& state) {
    const Eigen::Quaterniond G_q_I(state.G_R_I);
    file_state_ << std::fixed << std::setprecision(15)
                << state.timestamp << ","
                << state.lla[0] << "," << state.lla[1] << "," << state.lla[2] << ","
                << state.G_p_I[0] << "," << state.G_p_I[1] << "," << state.G_p_I[2] << ","
                << state.G_v_I[0] << "," << state.G_v_I[1] << "," << state.G_v_I[2] << ","
                << G_q_I.x() << "," << G_q_I.y() << "," << G_q_I.z() << "," << G_q_I.w() << ","
                << state.acc_bias[0] << "," << state.acc_bias[1] << "," << state.acc_bias[2] << ","
                << state.gyro_bias[0] << "," << state.gyro_bias[1] << "," << state.gyro_bias[2] << "\n";
}

void LocalizationWrapper::LogGps(const ImuGpsLocalization::GpsPositionDataPtr gps_data) {  // 保存GPS数据的时间戳，经纬度，高程值到日志文件；
    file_gps_ << std::fixed << std::setprecision(15)
              << gps_data->timestamp << ","
              << gps_data->lla[0] << "," << gps_data->lla[1] << "," << gps_data->lla[2] << "\n";
}

void LocalizationWrapper::ConvertStateToRosTopic(const ImuGpsLocalization::State& state) {  // 提取 state 中的当前位姿并转成geometry_msgs::PoseStamped类型发布出去；
    ros_path_.header.frame_id = "world";
    ros_path_.header.stamp = this->now();  

    geometry_msgs::msg::PoseStamped pose;
    pose.header = ros_path_.header;

    pose.pose.position.x = state.G_p_I[0];
    pose.pose.position.y = state.G_p_I[1];
    pose.pose.position.z = state.G_p_I[2];

    const Eigen::Quaterniond G_q_I(state.G_R_I);
    pose.pose.orientation.x = G_q_I.x();
    pose.pose.orientation.y = G_q_I.y();
    pose.pose.orientation.z = G_q_I.z();
    pose.pose.orientation.w = G_q_I.w();

    ros_path_.poses.push_back(pose);
}



}  // namespace localization

RCLCPP_COMPONENTS_REGISTER_NODE(localization::LocalizationWrapper)
