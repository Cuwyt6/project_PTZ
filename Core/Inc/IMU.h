//
// Created by lenovo on 24-12-5.
//

#ifndef PTZ_IMU_H
#define PTZ_IMU_H

#include <cstdint>

class IMU {
private:
    float acc_x_; //加速度计
    float acc_y_;
    float acc_z_;
    float gyro_x_; //角速度计
    float gyro_y_;
    float gyro_z_;

    float yaw_gyro_;
    float pitch_gyro_;
    float roll_gyro_;

    float pitch_acc_;
    float roll_acc_;

    float K; //线性互补滤波系数

public:
    IMU(float k);
    void AccCalculate();
    void GyroCalculate();
    void IMUCalculate();

    float IMUYaw(){return yaw_gyro_;}
    float IMUPitch(){return pitch_gyro_;}
    float IMURoll(){return roll_gyro_;}
};

void BMI088_Init();

void BMI088_gyro_read_reg(uint8_t reg, uint8_t *return_data, uint8_t length);
void BMI088_accel_read_reg(uint8_t reg, uint8_t *return_data, uint8_t length);
void BMI088_accel_write_single_reg(uint8_t reg, uint8_t data);


#endif //PTZ_IMU_H
