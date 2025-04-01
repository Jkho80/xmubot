#ifndef _XMU_TEST_ROBOT_H_
#define _XMU_TEST_ROBOT_H_

#include <iostream>
#include <cmath>
#include <math.h>
#include <cstring>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <fcntl.h>


#include <rclcpp/rclcpp.hpp>
#include <rcl/types.h>
#include <serial/serial.h>
#include <sys/types.h>
#include <sys/stat.h>


#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <turtlesim/srv/spawn.hpp>

#include "wheeltec_robot_msg/msg/data.hpp"
#include "wheeltec_robot_msg/msg/supersonic.hpp"


using namespace std;
#define PI 3.14159265358979323846f // 定义圆周率π的近似值，使用单精度浮点数表示

// ANSI转义序列用于文本颜色和样式
#define RESET string("\033[0m") // 重置颜色和样式
#define RED string("\033[31m")  // 红色文本
#define GREEN string("\033[32m") // 绿色文本
#define YELLOW string("\033[33m") // 黄色文本
#define BLUE string("\033[34m") // 蓝色文本
#define MAGENTA string("\033[35m") // 紫色文本
#define CYAN string("\033[36m") // 青色文本



// 数据传输标识位 和 帧头帧尾的标识位
#define SEND_DATA_BCC 1 // 标识位，用于指示发送数据给机器人的操作，用作校验或状态标志
#define READ_DATA_BCC 0 // 标识位，用于指示从机器人读取数据的操作，用作校验或状态标志
#define FRAME_HEAD 0X7B // 定义帧头字节，用作数据包的开始标记，十六进制表示为{ (ASCII)
#define FRAME_TAIL 0X7D // 定义帧尾字节，用作数据包的结束标记，十六进制表示为} (ASCII)

#define RECEIVE_DATA_SIZE 24 // 定义从STM32接收到的数据包的大小，固定为24字节
#define SEND_DATA_SIZE 11 // 定义发送到STM32的数据包的大小，最大为11字节


#define DISTANCE_DATA_SIZE 19 // 定义从STM32接收到的距离数据包的大小，固定为19字节
#define DISTANCE_HEAD 0XFA // 定义距离数据包的头部字节，用作距离数据包的开始标记
#define DISTANCE_TAIL 0XFC // 定义距离数据包的尾部字节，用作距离数据包的结束标记

#define AUTOCHARGE_HEAD 0X7C // 定义自动充电数据包的头部字节，用作自动充电数据包的开始标记
#define AUTOCHARGE_TAIL 0X7F // 定义自动充电数据包的尾部字节，用作自动充电数据包的结束标记
#define AUTOCHARGE_DATA_SIZE 8 // 定义从STM32接收到的自动充电数据包的大小，固定为8字节


//与IMU陀螺仪设置的量程有关，量程±500°，对应数据范围±32768
//陀螺仪原始数据转换位弧度(rad)单位，1/65.5/57.30=0.00026644
#define GYROSCOPE_RATIO 0.00026644f // 定义了将陀螺仪原始数据（单位：度数）转换为弧度（rad）的转换比例。这个比例是基于陀螺仪的量程（±500°）和数据范围（±32768）计算得出的。

//与IMU加速度计设置的量程有关，量程±2g，对应数据范围±32768
//加速度计原始数据转换位m/s^2单位，32768/2g=32768/19.6=1671.84
#define ACCELEROMETER_RATIO 1671.84f // 定义了将加速度计原始数据（单位：未知，但基于上下文应为某种与g相关的单位）转换为米每二次方秒（m/s^2）的转换比例。这个比例是基于加速度计的量程（±2g）和数据范围（±32768）计算得出的。

extern sensor_msgs::msg::Imu imu_mpu; // 于存储IMU话题的数据。这个变量可能在其他文件中定义和初始化。


// odom_position_Cov 和 odom_position_Cov2：两个协方差矩阵，用于描述里程计位置数据的不确定性。
// 具体的值是根据wheeltec robot 的配置
const double odom_position_Cov[36] = {1e-3,    0,    0,   0,   0,    0, 
                                        0, 1e-3,    0,   0,   0,    0,
                                        0,    0,  1e6,   0,   0,    0,
                                        0,    0,    0, 1e6,   0,    0,
                                        0,    0,    0,   0, 1e6,    0,
                                        0,    0,    0,   0,   0,  1e3 };

const double odom_position_Cov2[36] = {1e-9,    0,    0,   0,   0,    0, 
                                        0, 1e-3, 1e-9,   0,   0,    0,
                                        0,    0,  1e6,   0,   0,    0,
                                        0,    0,    0, 1e6,   0,    0,
                                        0,    0,    0,   0, 1e6,    0,
                                        0,    0,    0,   0,   0, 1e-9 };

// odom_twist_covariance 和 odom_twist_covariance2：两个协方差矩阵，用于描述里程计速度数据的不确定性。
// 具体的值是根据wheeltec robot 的配置                    
const double odom_twist_Cov[36] = {1e-3,    0,    0,   0,   0,    0, 
                                    0, 1e-3,    0,   0,   0,    0,
                                    0,    0,  1e6,   0,   0,    0,
                                    0,    0,    0, 1e6,   0,    0,
                                    0,    0,    0,   0, 1e6,    0,
                                    0,    0,    0,   0,   0,  1e3 };

const double odom_twist_Cov2[36] = {1e-9,    0,    0,   0,   0,    0, 
                                    0, 1e-3, 1e-9,   0,   0,    0,
                                    0,    0,  1e6,   0,   0,    0,
                                    0,    0,    0, 1e6,   0,    0,
                                    0,    0,    0,   0, 1e6,    0,
                                    0,    0,    0,   0,   0, 1e-9} ;


// 用途：存储超声波传感器在不同方向上的测量距离。
// 成员变量：每个变量代表一个特定方向（以度为单位）上的测量距离。
typedef struct _DISTANCE_DATA_ {
    float direction_0;     // 0度方向上的超声波测量距离
    float direction_45;    // 45度方向上的超声波测量距离
    float direction_90;    // 90度方向上的超声波测量距离
    float direction_135;   // 135度方向上的超声波测量距离
    float direction_180;   // 180度方向上的超声波测量距离
    float direction_225;   // 225度方向上的超声波测量距离
    float direction_270;   // 270度方向上的超声波测量距离
    float direction_315;   // 315度方向上的超声波测量距离
} supersonic_range;   


// 用途：存储速度或位置数据。
// 成员变量：x、y、z分别代表X、Y、Z轴方向上的速度或位置。
typedef struct _VEL_POS_DATA_ {
    float x;   // X轴方向上的速度或位置
    float y;   // Y轴方向上的速度或位置
    float z;   // Z轴方向上的速度或位置
} vel_pos;

// 用途：存储IMU（惯性测量单元）数据。
// 成员变量：包含加速度计和陀螺仪在X、Y、Z轴方向上的原始数据。
typedef struct _IMU_DATA_ {
    short accelerate_x;   // X轴方向上的加速度计原始数据
    short accelerate_y;   // Y轴方向上的加速度计原始数据
    short accelerate_z;   // Z轴方向上的加速度计原始数据
    short gyroscope_x;    // X轴方向上的陀螺仪原始数据
    short gyroscope_y;    // Y轴方向上的陀螺仪原始数据
    short gyroscope_z;    // Z轴方向上的陀螺仪原始数据
} imu_data;

// 用途：封装要发送给机器人的速度数据。
// 成员变量：包括速度数据缓冲区、X、Y、Z轴方向上的速度，以及数据帧的头部和尾部标记。
typedef struct _SEND_VEL_DATA_ {
    uint8_t tx[SEND_DATA_SIZE];   // 用于发送的速度数据缓冲区
    float x_speed;                // X轴方向上的速度
    float y_speed;                // Y轴方向上的速度
    float z_speed;                // Z轴方向上的速度
    unsigned char frame_head;     // 数据帧的头部标记
    unsigned char frame_tail;     // 数据帧的尾部标记
} send_vel;


// 用途：封装从机器人接收到的速度数据。
// 成员变量：包括速度或位置数据缓冲区、X、Y、Z轴方向上的速度，数据帧的头部和尾部标记，以及电源电压。
typedef struct _RECEIVE_VEL_DATA_ {
    uint8_t rx[RECEIVE_DATA_SIZE]; // 用于接收的速度数据缓冲区
    uint8_t stop_flag;                // 停止标志，用于指示是否需要停止机器人的运动
    float x_speed;                       // X轴方向上的速度
    float y_speed;                       // Y轴方向上的速度
    float z_speed;                       // Z轴方向上的速度
    unsigned char frame_head;      // 数据帧的头部标记
    unsigned char frame_tail;      // 数据帧的尾部标记
    float voltage;              // 电源电压
} receive_vel;


// 用途：封装从机器人接收到的距离数据。
// 成员变量：包括距离数据缓冲区以及数据帧的头部和尾部标记。
typedef struct _DISTANCE_DATA_ {
    uint8_t rx[DISTANCE_DATA_SIZE]; // 用于接收的距离数据缓冲区
    unsigned char frame_head;       // 数据帧的头部标记
    unsigned char frame_tail;       // 数据帧的尾部标记
} distances;


// 用途：封装从机器人接收到的自动回充数据。
// 成员变量：包括自动回充数据缓冲区以及数据帧的头部和尾部标记。
typedef struct _AUTOCHARGE_DATA_ {
    uint8_t rx[AUTOCHARGE_HEAD]; // 用于接收的自动回充数据缓冲区
    unsigned char frame_head;         // 数据帧的头部标记
    unsigned char frame_tail;         // 数据帧的尾部标记
} autocharge;



class xmu_test_robot : public rclcpp::Node {
public:
    xmu_test_robot();
    ~xmu_test_robot();
    void Robot_Control();
    serial::Serial STM32_serial_;

private:
    // 订阅red_vel话题，接收geometry_msgs/Twist类型的消息，可能用于控制机器人跟随红色目标的速度。
    rclcpp::Subcription<geometry_msgs::msg::Twist>::SharedPtr red_vel_sub_;
    // 订阅cmd_vel话题，接收geometry_msgs/Twist类型的消息，用于控制机器人的线速度和角速度。
    rclcpp::Subcription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    // 订阅recharge话题，接收std_msgs/Int8类型的消息，用于指示机器人是否开始或停止自动回充。
    rclcpp::Subcription<std_msgs::msg::Int8>::SharedPtr recharge_sub_;


    // 发布odom话题，发布nav_msgs/Odometry类型的消息，包含机器人的里程计数据。
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    // 发布imu话题，发布sensor_msgs/Imu类型的消息，包含IMU（惯性测量单元）的数据。
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    // 发布distance话题，发布wheeltec_robot_msg/Supersonic类型的消息，包含超声波传感器的距离数据。
    rclcpp::Publisher<wheeltec_robot_msg::msg::Supersonic>::SharedPtr distance_pub_;
    // 发布charging话题，发布std_msgs/Bool类型的消息，用于指示机器人是否正在充电。
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr charging_pub_;
    // 发布red话题，发布std_msgs/UInt8类型的消息，用于指示机器人是否检测到红色目标。
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr red_pub_;
    // 发布voltage话题，发布std_msgs/Float32类型的消息，用于指示机器人的电源电压。
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr charging_volt_pub_;
    // 发布current话题，发布std_msgs/Float32类型的消息，用于指示机器人的充电电流。
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr charging_ampere_pub_;
    // 发布charge服务，用于设置机器人的自动回充状态。
    rclcpp::Service<turtlesim::srv::Spawn>::SharedPtr Set_Charge_Srv_;


    // 回调函数，用于处理cmd_vel话题接收到的速度控制消息。
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    
    // 回调函数，用于处理red_vel话题接收到的速度控制消息。
    void red_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    // 回调函数，用于处理recharge话题接收到的自动回充控制消息。
    void recharge_callback(const std_msgs::msg::Int8::SharedPtr msg);
    // 回调函数，用于处理set_charge服务请求，设置机器人的自动回充状态。
    void set_charge_callback(
        const shared_ptr<turtlesim::srv::Spawn::Request> request, 
        const shared_ptr<turtlesim::srv::Spawn::Response> response);

    // 发布里程计数据、IMU数据、距离数据等。
    void publish_odom();
    void publish_imu();
    void publish_distance();

    // 发布充电状态、电流和电压等。
    void publish_charging();
    void publish_ampere();
    void publish_red();
    void publish_voltage();

    // 获取传感器数据，包括速度、位置、IMU和距离等。
    bool get_sensor_data();
    bool get_sensor_data_new();
    unsigned char check_sum(unsigned char count_num, unsigned char mode); // BBC 校验函数
    unsigned char check_sum_autoCharge(unsigned char count_num, unsigned char mode);
    short imu_transform(uint_8 high_bit, uint8_t low_bit);
    float odom_transform(uint8_t high_bit, uint8_t low_bit);


    // 串口端口名称，用于与机器人的通信
    string usart_port_name_;
    // 机器人的TF帧ID，用于坐标变换
    string robot_frame_id_;
    string imu_frame_id_;       // IMU传感器的TF帧ID
    string odom_frame_id_;      // 里程计的TF帧ID
    string gyro_frame_id_;      // 陀螺仪的TF帧ID

    // 串口通信的波特率和超时时间
    int serial_baud_rate_;
    int serial_timeout_;

    // 用于接收从机器人传来的速度数据
    receive_vel receive_vel_data_; // 注意：这里应该是 receive_vel_data 类型，但原代码中写成了 recieve_vel_data，可能是个拼写错误
    // 用于发送给机器人的速度数据
    send_vel send_vel_data_;
    // 用于存储从机器人接收到的距离数据
    distances distance_data_;
    // 用于存储从机器人接收到的自动回充数据
    autocharge autocharge_data_;
    // 用于存储超声波传感器在不同方向上的测量距离
    supersonic_range supersonic_distance_data_;
    // 用于存储机器人的 位置，速度 数据
    vel_pos robot_position_data_;
    vel_pos robot_velocity_data_;
    // 用于存储IMU（惯性测量单元）的数据
    imu_data imu_mpu_data_;

    // 指示机器人当前是否正在充电
    bool charging_ = false;
    // 自动回充标志位，用于指示机器人是否应自动回充
    int8_t autorecharge_ = 0;
    // 电源电压, 电流值
    float voltage_ = 0.0;
    float ampere_ = 0.0;
    // 指示机器人是否检测到充电桩的红外信号的标志位
    uint8_t red_target_ = 0;

    // 里程计数据的X, Y, Z正负轴缩放比例
    float odom_x_scale_;
    float odom_y_scale_;
    float odom_z_scale_positive_;
    float odom_z_scale_negative_;

    // 用于记录当前时间的变量
    rclcpp::Time now_, last_;

    // 时间间隔，用于计算速度等
    float interval_time_;
};

#endif