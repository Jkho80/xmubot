#include "xmubot/xmu_test_robot.h"
#include "xmu_test_robot.h"
#define SAMPLING_FREQUENCY 20.0f



float inv_srqt(float number){
    volatile long  i;
    volatile float x, y;
    volatile const float f = 1.5f;

    x = number * 0.5f;
    y  = number;
    i  = *(long *)&y;
    i  = 0x5f3759df - ( i >> 1 );
    y  = *(float *)&i;
    y  = y * ( f - ( x * y * y ) );   
    return y;
}


volatile float kp_double_ = 1.0f;
volatile float ki_double_ = 0.0f;
volatile float quaternion_w_ = 1.0f, quaternion_x_ = 0.0f, quaternion_y_ = 0.0f, quaternion_z_ = 0.0f;
volatile float error_x_integral_ = 0.0f, error_y_integral_ = 0.0f, error_z_integral_ = 0.0f;


void quaternion_update(float gyro_x, float gyro_y, float gyro_z, float accel_x, float accel_y, float accel_z){
    float norm_reciprocal_;
    float half_vx_, half_vy_, half_vz_;
    float half_ex_, half_ey_, half_ez_;
    float qa_, qb_, qc_;
    float sampling_time = 1.0f / SAMPLING_FREQUENCY; // 1.0 / SAMPLING_FREQUENCY = 0.05s

    if((accel_x != 0.0f) || (accel_y != 0.0f) || (accel_z != 0.0f)) {
        // 计算方差的平方根倒数，用于归一化加速度向量
        norm_reciprocal_ = inv_srqt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z); // 计算方差
        
        half_vx_ = quaternion_x_ * quaternion_z_- quaternion_w_ * quaternion_y_;
        half_vy_ = quaternion_w_ * quaternion_x_ + quaternion_y_ * quaternion_z_;
        half_vz_ = quaternion_w_ * quaternion_w_ - 0.5f + quaternion_z_ * quaternion_z_;

        // 归一化加速度向量
        accel_x *= norm_reciprocal_;
        accel_y *= norm_reciprocal_;
        accel_z *= norm_reciprocal_;

        // 通过乘积来获取 half_ex_ i + half_ey_ j + half_ez_ k
        half_ex_ = (accel_y * half_vz_) - (accel_z * half_vy_);
        half_ey_ = (accel_z * half_vx_) - (accel_x * half_vz_);
        half_ez_ = (accel_x * half_vy_) - (accel_y * half_vx_);

        // 积分误差累加，用于消除静态偏差
        if(ki_double_ > 0.0f){
            error_x_integral_ += ki_double_ * half_ex_ * sampling_time; 
            error_y_integral_ += ki_double_ * half_ey_ * sampling_time;
            error_z_integral_ += ki_double_ * half_ez_ * sampling_time;
            gyro_x += error_x_integral_;
            gyro_y += error_y_integral_;
            gyro_z += error_z_integral_;
        }
        else{
            error_x_integral_ = 0.0f;
            error_y_integral_ = 0.0f;
            error_z_integral_ = 0.0f;
        }

        gyro_x += kp_double_ * half_ex_;
        gyro_y += kp_double_ * half_ey_;
        gyro_z += kp_double_ * half_ez_;
    }

    gyro_x *= 0.5f * sampling_time;
    gyro_y *= 0.5f * sampling_time;
    gyro_z *= 0.5f * sampling_time;

    qa_ = quaternion_w_;
    qb_ = quaternion_x_;
    qc_ = quaternion_y_;

    quaternion_w_ += (-qb_ * gyro_x - qc_ * gyro_y - quaternion_z_ * gyro_z);
    quaternion_x_ += (qa_ * gyro_x + qc_ * gyro_z - quaternion_z_ * gyro_y);
    quaternion_y_ += (qa_ * gyro_y - qb_ * gyro_z + quaternion_z_ * gyro_x);
    quaternion_z_ += (qa_ * gyro_z + qb_ * gyro_y - qc_ * gyro_x);

    norm_reciprocal_ = inv_srqt(quaternion_w_ * quaternion_w_ + quaternion_x_ * quaternion_x_ + quaternion_y_ * quaternion_y_ + quaternion_z_ * quaternion_z_);
    quaternion_w_ *= norm_reciprocal_;
    quaternion_x_ *= norm_reciprocal_;
    quaternion_y_ *= norm_reciprocal_;
    quaternion_z_ *= norm_reciprocal_;

    imu_mpu.orientation.w = quaternion_w_;
    imu_mpu.orientation.x = quaternion_x_;
    imu_mpu.orientation.y = quaternion_y_;
    imu_mpu.orientation.z = quaternion_z_;
}
