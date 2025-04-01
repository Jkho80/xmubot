// #include "xmubot/xmu_test_robot.h"
// #include "xmubot/quaternion_update.h"
#include "../include/xmu_test_robot.h"
#include "../include/quaternion_update.h"
#include "wheeltec_robot_msg/msg/data.hpp"
using std::placeholders::_1;
using namespace std;

rclcpp::Node::SharedPtr node = nullptr;

sensor_msgs::msg::Imu imu_mpu_;


bool check_autocharge = false;
bool charger_status = false;

xmu_test_robot::xmu_test_robot() : rclcpp::Node("xmu_test_robot"){
    interval_time_ = 0;
    voltage_ = 0;

    memset(&robot_position_data_, 0, sizeof(robot_position_data_));
    memset(&robot_velocity_data_, 0, sizeof(robot_velocity_data_));
    memset(&receive_vel_data_, 0, sizeof(receive_vel_data_));
    memset(&send_vel_data_, 0, sizeof(receive_vel_data_));
    memset(&imu_mpu_data_, 0, sizeof(imu_mpu_data_));

    
    this->declare_parameter<int>("serial_baud_rate");
    this->declare_parameter<string>("usart_port_name");
    

}

void xmu_test_robot::Robot_Control() {
    last_ = rclcpp::Node::now();
    // 只有在 单线程 情况下才推荐使用以下方法存储 角度 的变换值
    // static float last_z = NAN;  // 初始化为非法值
    static float cos_z = 0.0f;
    static float sin_z = 0.0f;
    static float atan2_z = 0.0f;


    while(rclcpp::ok()){
        try{
            now_ = rclcpp::Node::now();
            interval_time_ = (now_ - last_).seconds();

            if(get_sensor_data() == true){
                // 像下列代码如果数据类型不同，例如 前面是 float 后面是 int, 有可能输出还是 float 但是会失去后面的 小数点 (如: 124.00)
                // a.x *= b;
                // 下列代码虽然不会有这种问题，但是相对更慢些
                // a.x = a
                
                // 注 : X 是前后方向，Y 是左右方向 (m/s)，而 Z 是角速度 (绕Z轴) (rad/s)
                robot_vel_data_.x *= odom_x_scale_;
                robot_vel_data_.y *= odom_y_scale_;

                cos_z = cos(robot_position_data_.z);
                sin_z = sin(robot_position_data_.z);


                if(robot_vel_data_.z >= 0)
                    robot_vel_data_.z *= odom_z_scale_positive_;
                else
                    robot_vel_data_.z *= odom_z_scale_negative_;
                
                // robot_position_data_.x += (robot_vel_data_.x * cos(robot_position_data_.z) - robot_vel_data_.y * sin(robot_position_data_.z)) * interval_time_;
                // robot_position_data_.y += (robot_vel_data_.x * sin(robot_position_data_.z) + robot_vel_data_.y * cos(robot_position_data_.z)) * interval_time_;
                robot_position_data_.x += (robot_vel_data_.x * cos_z - robot_vel_data_.y * sin_z) * interval_time_;
                robot_position_data_.y += (robot_vel_data_.x * sin_z + robot_vel_data_.y * cos_z) * interval_time_;
                robot_position_data_.z += robot_vel_data_.z * interval_time_;

                quaternion_update(
                    imu_data.angular_velocity.x, 
                    imu_data.angular_velocity.y,
                    imu_data.angular_velocity.z,
                    imu_data.linear_acceleration.x,
                    imu_data.linear_acceleration.y,
                    imu_data.linear_acceleration.z,
                );

                publish_odom();
                publish_imu();
                publish_voltage();

                last_ = now_;
            }

            if(check_autocharge){
                publish_autocharge();
                publish_red_target();
                publish_ampere();
                check_autocharge = false;
            }

            rclcpp::spin_some(this->get_node_base_interface());
        }
        catch (const rclcpp::exceptions::RCLError &e) {
            RCLCPP_ERROR(this->get_logger(), "Unexpected fail withi %s", e.what().c_str())
        }
    }
}


bool xmu_test_robot::get_sensor_data() {
    uint16_t transition_16 = 0;
    // static int flag_error = 0, temp = 1;
    static int count1, count2;

    uint8_t receive_vel_temp[1] = {0};
    uint8_t check1 = 0, check2 = 0;

    STM32_serial_.read(receive_vel_temp, sizeof(receive_vel_temp));

    // uint8_t temp_array_tr[temp] = {0};

    receive_vel_data_.rx[count1] = receive_vel_temp[0]; 
    autocharge_.rx[count2] = receive_vel_temp[0]; 

    receive_vel_data_.frame_head = receive_vel_data_.rx[0];
    receive_vel_data_.frame_tail = receive_vel_data_.rx[23];


    if((recieve_vel_pr[0] == AUTOCHARGE_HEAD) || count2 > 0)
        count2++;
    else
        count2 = 0;

    if((recieve_vel_pr[0] == FRAME_HEAD) || count1 > 0)
        count1++;
    else
        count1 = 0; 


    if(count2 == AUTOCHARGE_DATA_SIZE){
        count2 = 0;
        if(autocharge_.rx[AUTOCHARGE_DATA_SIZE - 1] == AUTOCHARGE_TAIL){
            check2 = check_sum_autoCharge(6, READ_DATA_BCC);

            if(check2 == autocharge_data_.rx[AUTOCHARGE_DATA_SIZE - 2]){                
                // 当需要用到 汇编 调试时才使用下面的第一种方式
                // transition_16 = 0;
                // transition_16 |= (autocharge_data_.rx[1] << 8);
                // transition_16 |= autocharge_data_.rx[2];

                transition_16 = (autocharge_data_.rx[1] << 8) | autocharge_data_.rx[2];
                ampere_ = transition_16 / 1000.0f;
                // ampere_ = (transition_16 / 1000) + (transition_16 % 1000) * 0.001;
                
                // 红外接收状态通常用于指示机器人是否检测到了充电桩发出的红外信号。
                // 在自动充电过程中，机器人需要能够准确地找到充电桩，并与之对接以进行充电。
                // 红外信号作为一种无线通信技术，常被用于这种短距离的定位和通信场景。
                red_target_ = autocharge_data_.rx[3];

                // 机器人是否在充电
                charging_ = autocharge_data_.rx[4];

                // check_autocharge 表示 是否已进行充电检查，
                // charger_status 表示 是否在电桩，
                charger_status = autocharge_data_.rx[5];
                check_autocharge = true;
            }
        }
    }

    if(count1 == RECEIVE_DATA_SIZE){
        count1 = 0;
        if(receive_vel_data_.frame_tail == FRAME_TAIL){
            check1 = check_sum(22, READ_DATA_BCC);
            
            if(check1 == receive_vel_data_.rx[22]){
                // 更新停止标志，指示是否接收到停止命令
                receive_vel_data_.stop_flag = receive_vel_data_.rx[1];

                // 将接收到的速度数据转换为实际的物理量 (m/s)
                robot_velocity_data_.x = odom_transform(receive_vel_data_.rx[2], receive_vel_data_.rx[3]);
                robot_velocity_data_.y = odom_transform(receive_vel_data_.rx[4], receive_vel_data_.rx[5]);
                robot_velocity_data_.z = odom_transform(receive_vel_data_.rx[6], receive_vel_data_.rx[7]);

                // 将接收到的IMU数据（加速度计）转换为实际的物理量 (m/s^2)
                imu_mpu_data_.accelerate_x = imu_transform(receive_vel_data_.rx[8], receive_vel_data_.rx[9]);
                imu_mpu_data_.accelerate_y = imu_transform(receive_vel_data_.rx[10], receive_vel_data_.rx[11]);
                imu_mpu_data_.accelerate_z = imu_transform(receive_vel_data_.rx[12], receive_vel_data_.rx[13]);
                // 将接收到的IMU数据（陀螺仪）转换为实际的物理量 (rad/s)
                imu_mpu_data_.gyroscope_x = imu_transform(receive_vel_data_.rx[14], receive_vel_data_.rx[15]);
                imu_mpu_data_.gyroscope_y = imu_transform(receive_vel_data_.rx[16], receive_vel_data_.rx[17]);
                imu_mpu_data_.gyroscope_z = imu_transform(receive_vel_data_.rx[18], receive_vel_data_.rx[19]);

                // 将加速度计数据从原始单位转换为 ros2 的内置消息 sensor_msgs::msg::Imu
                imu_mpu_.linear_acceleration.x = imu_mpu_data_.accelerate_x / ACCELEROMETER_RATIO;
                imu_mpu_.linear_acceleration.y = imu_mpu_data_.accelerate_y / ACCELEROMETER_RATIO;
                imu_mpu_.linear_acceleration.z = imu_mpu_data_.accelerate_z / ACCELEROMETER_RATIO;

                imu_mpu_.angular_velocity.x = imu_mpu_data_.gyroscope_x / GYROSCOPE_RATIO;
                imu_mpu_.angular_velocity.y = imu_mpu_data_.gyroscope_y / GYROSCOPE_RATIO;
                imu_mpu_.angular_velocity.z = imu_mpu_data_.gyroscope_z / GYROSCOPE_RATIO;

                // imu_pub_->publish(imu_mpu_);
                
                // 当需要用到 汇编 调试时才使用下面的第一种方式
                // 将接收到的电池电压数据(毫伏)转换为伏特
                // transition_16 = 0;
                // transition_16 |= (receive_vel_data_.rx[20] << 8);
                // transition_16 |= receive_vel_data_.rx[21];

                transition_16 = (autocharge_data_.rx[20] << 8) | autocharge_data_.rx[21];
                voltage_ = transition_16 / 1000.0f;
                return true;
            }    
        }
    }

    return false;
}

unsigned char xmu_test_robot::check_sum(unsigned char count_num, unsigned char mode) {
    unsigned char check_sum = 0;

    if(mode == 0){
        for(int i = 0; i < count_num; i++){
            check_sum = check_sum ^ receive_vel_data_.rx[i];
        }
    }

    else if(mode == 1){
        for(int i = 0; i < count_num; i++){
            check_sum = check_sum ^ send_vel_data_.rx[i];
        }
    }

    return check_sum;
}


unsigned char xmu_test_robot::check_sum_autoCharge(unsigned char count_num, unsigned char mode) {
    unsigned char check_sum = 0;

    if(mode == 0){
        for(int i = 0; i < count_num; i++){
            check_sum = check_sum ^ autocharge_data_.rx[i];
        }
    }

    return check_sum;
}


void xmu_test_robot::publish_odom() {
    tf2::Quaternion q;
    q.setRPY(0, 0, robot_position_data_.z);
    geometry_msgs::msg::Quaternion odom_quaternion_ = tf2::toMsg(q);

    nav_msgs::mgs::Odometry odom_msg_;
    odom_msg_.header.stamp = this->now();
    odom_msg_.header.frame_id = odom_frame_id_;
    odom_msg_.child_frame_id = robot_frame_id_;

    odom_msg_.pose.pose.position.x = robot_position_data_.x;
    odom_msg_.pose.pose.position.y = robot_position_data_.y;
    odom_msg_.pose.pose.position.z = robot_position_data_.z;
    odom_msg_.pose.pose.orientation = odom_quaternion_;

    odom_msg_.twist.twist.linear.x = robot_velocity_data_.x;
    odom_msg_.twist.twist.linear.y = robot_velocity_data_.y;
    odom_msg_.twist.twist.angular.z = robot_velocity_data_.z;

    if((robot_vel_data_.x != 0) || (robot_vel_data_.y != 0) || (robot_vel_data_.z == 0)){
        memcpy(&odom_msg_.pose.covariance, odom_position_Cov, sizeof(odom_position_Cov));
        memcpy(&odom_msg_.twist.covariance, odom_twist_Cov, sizeof(odom_twist_Cov));
    }
    else{
        memcpy(&odom_msg_.pose.covariance, odom_position_Cov2, sizeof(odom_position_Cov2));
        memcpy(&odom_msg_.twist.covariance, odom_twist_Cov2, sizeof(odom_twist_Cov2));
    }

    odom_pub_->publish(odom_msg_);
}

void xmu_test_robot::publish_imu() {
    sensor_msgs::msg::Imu imu_msg_;

    imu_msg_.header.stamp = this->now();
    imu_msg_.header.frame_id = gyro_frame_id_;

    imu_msg_.orientation.x = imu_mpu_.orientation.x;
    imu_msg_.orientation.y = imu_mpu_.orientation.y;
    imu_msg_.orientation.z = imu_mpu_.orientation.z;
    imu_msg_.orientation.w = imu_mpu_.orientation.w;

    imu_msg_.angular_velocity.x = imu_mpu_.angular_velocity.x;
    imu_msg_.angular_velocity.y = imu_mpu_.angular_velocity.y;
    imu_msg_.angular_velocity.z = imu_mpu_.angular_velocity.z;

    imu_msg_.linear_acceleration.x = imu_mpu_.linear_acceleration.x;
    imu_msg_.linear_acceleration.y = imu_mpu_.linear_acceleration.y;
    imu_msg_.linear_acceleration.z = imu_mpu_.linear_acceleration.z;

    imu_msg_.orientation_covariance[0] = 1e6;
    imu_msg_.orientation_covariance[4] = 1e6;
    imu_msg_.orientation_covariance[8] = 1e-6;
    
    imu_msg_.angular_velocity_covariance[0] = 1e6;
    imu_msg_.angular_velocity_covariance[4] = 1e6;
    imu_msg_.angular_velocity_covariance[8] = 1e6;

    imu_pub_->publish(imu_msg_);
}

void xmu_test_robot::publish_voltage() {
    std_msgs::msg::Float32 voltage_msg_;

    static float count_voltage_pub_ = 0;
    if(count_voltage_pub_++ > 10) {
        count_voltage_pub_ = 0;
        voltage_msg_.data = voltage_;
        charging_volt_pub_->publish(voltage_msg_);
    }
}


void xmu_test_robot::publish_red() {
    std_msgs::msg::UInt8 red_msg_;
    red_msg_.data = red_target_;
    red_pub_->publish(red_msg_);
}


void xmu_test_robot::publish_charging() {
    static bool last_charging;
    std_msgs::msg::Bool charging_msg_;
    charging_msg_.data = charging_;

    if (last_charging != charging_) {
        // last_charging == false && charging_ == true 表示充电器刚插上，开始充电
        if (!last_charging && charging_)
            cout << GREEN << "Robot charger plugged. [Start charging]" << endl << RESET;
        else 
            cout << RED << "Robot charger unplugged. [Stop charging]" << endl << RESET;
        last_charging = charging_;  // 更新状态缓存
    }
}







int main(int argc, char** argv){
    rclcpp::intit(argc, argv);
    xmu_test_robot Robot;
    Robot.Robot_Control();
    
    return 0;
}