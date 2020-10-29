imu_gps_localization
=====

# imu_gps_localization
 使用ESKF算法对imu和gps进行数据融合。

## 环境部署
 '''
 sudo apt-get install nmea_navsat_driver
 '''
## 数据集
 [EU Long-term Dataset with Multiple Sensors for Autonomous Driving](https://epan-utbm.github.io/utbm_robocar_dataset/)

## 输入输出
- 输入
| topic     | 格式                    | 说明       |
| :-------- | --------:               | :--:       |
| /imu/data | sensor_msgs::Imu        |  imu数据   |
| /fix      | sensor_msgs::NavSatFix  |  gps数据   |

- 输出
| topic     | 格式                    | 说明       |
| :-------- | --------:               | :--:       |
| /fused_path | nav_msgs::Path        |  融合后位置数据   |

## 编译运行
'''
cd build && catkin_make --source .. 
source devel/setup.bash && roslaunch imu_gps_localization imu_gps_localization.launch
'''
