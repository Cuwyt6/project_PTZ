//
// Created by lenovo on 24-12-5.
//

#include "IMU.h"
#include "spi.h"
#include <cmath>

#define g 9.7940
#define pi 3.14159

float loop_rate = 0.001;
extern uint8_t rx_acc_data[6];
extern uint8_t rx_gyro_data[6];

IMU::IMU(float k): K(k){
    yaw_gyro_ = 0.0;
    pitch_gyro_ = 0.0;
    roll_gyro_ =0.0;
}

//片选
void BMI088_ACCEL_NS_L(void)//accel low
{
    HAL_GPIO_WritePin(ACC_GPIO_Port, ACC_Pin, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void)//accel high
{
    HAL_GPIO_WritePin(ACC_GPIO_Port, ACC_Pin,  GPIO_PIN_SET);
}
void BMI088_GYRO_NS_L(void)//gyro low
{
    HAL_GPIO_WritePin(GYRO_GPIO_Port, GYRO_Pin,  GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void)//gyro high
{
    HAL_GPIO_WritePin(GYRO_GPIO_Port, GYRO_Pin,  GPIO_PIN_SET);
}

//写入一个字节（不是一位）
void BMI088_write_byte(uint8_t txdata){
    HAL_SPI_Transmit(&hspi1, &txdata, 1, 1000);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
}

//读取一个字节（不是一位）
void BMI088_read_byte(uint8_t *rxdata, uint8_t length){
    HAL_SPI_Receive(&hspi1, rxdata, length, 1000);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX);
}

//写入一个寄存器
void BMI088_accel_write_single_reg(uint8_t reg, uint8_t data){

    BMI088_GYRO_NS_H();
    BMI088_ACCEL_NS_L();
    HAL_Delay(1);

    BMI088_write_byte(reg & 0x7F);
    BMI088_write_byte(data);

    HAL_Delay(1);
    BMI088_ACCEL_NS_H();
}

//读取一个寄存器
void BMI088_accel_read_reg(uint8_t reg, uint8_t *return_data, uint8_t length){

    BMI088_GYRO_NS_H();
    BMI088_ACCEL_NS_L();

    BMI088_write_byte(reg | 0x80);
    BMI088_read_byte(return_data, 1);
    BMI088_read_byte(return_data, length);

    BMI088_ACCEL_NS_H();
}

void BMI088_gyro_write_single_reg(uint8_t reg, uint8_t data){

    BMI088_ACCEL_NS_H();
    BMI088_GYRO_NS_L();
//  HAL_Delay(1);

    BMI088_write_byte(reg & 0x7F);
    BMI088_write_byte(data);

//  HAL_Delay(1);
    BMI088_GYRO_NS_H();
}

void BMI088_gyro_read_reg(uint8_t reg, uint8_t *return_data, uint8_t length){

    BMI088_ACCEL_NS_H();
    BMI088_GYRO_NS_L();
//  HAL_Delay(1);

    BMI088_write_byte(reg | 0x80);
    BMI088_read_byte(return_data, length);

//  HAL_Delay(1);
    BMI088_GYRO_NS_H();
}

void BMI088_write_reg(uint8_t reg, uint8_t data){
    BMI088_write_byte(reg & 0x7F);//写入寄存器
    BMI088_write_byte(data);//写入数据
}

void BMI088_Init(void) {
    // Soft Reset ACCEL
    BMI088_ACCEL_NS_L();
    BMI088_write_reg(0x7E, 0xB6); // Write 0xB6 to ACC_SOFTRESET(0x7E)
    HAL_Delay(1);
    BMI088_ACCEL_NS_H();

    // Soft Reset GYRO
    BMI088_GYRO_NS_L();
    BMI088_write_reg(0x14, 0xB6); // Write 0xB6 to GYRO_SOFTRESET(0x14)
    HAL_Delay(30);
    BMI088_GYRO_NS_H();

    // Switch ACCEL to Normal Mode
    BMI088_ACCEL_NS_L();
    HAL_Delay(1);
    BMI088_write_reg(0x7D, 0x04); // Write 0x04 to ACC_PWR_CTRL(0x7D)
    HAL_Delay(1);
    BMI088_ACCEL_NS_H();

    // Set ACCEL RANGE
    BMI088_ACCEL_NS_L();
    HAL_Delay(1);
    BMI088_write_reg(0x41, 0x02); // Write 0x04 to ACC_PWR_CTRL(0x7D)
    HAL_Delay(1);
    BMI088_ACCEL_NS_H();
}

uint8_t rx_acc_range_raw, rx_gyro_range_raw;//量程
int rx_acc_range_k, rx_gyro_range_k; //量程系数

void IMU::AccCalculate() {

    BMI088_accel_read_reg(0x41, &rx_acc_range_raw, 1);
    rx_acc_range_k = pow(2, (int) rx_acc_range_raw) * 3;

    BMI088_accel_read_reg(0x12, rx_acc_data, 6);

    acc_x_ = (int16_t)((uint16_t)rx_acc_data[0] | (uint16_t) rx_acc_data[1] << 8) / 32768.f * rx_acc_range_k * g;
    acc_y_ = (int16_t)((uint16_t)rx_acc_data[2] | (uint16_t) rx_acc_data[3] << 8) / 32768.f * rx_acc_range_k * g;
    acc_z_ = (int16_t)((uint16_t)rx_acc_data[4] | (uint16_t) rx_acc_data[5] << 8) / 32768.f * rx_acc_range_k * g;
}

void IMU::GyroCalculate() {

//  uint8_t rx_gyro_range_raw;
    BMI088_gyro_read_reg(0x0F, &rx_gyro_range_raw, 1);
    rx_gyro_range_k = pow(2, (int) rx_gyro_range_raw) ;

    BMI088_gyro_read_reg(0x02, rx_gyro_data, 6);

    gyro_x_ = (int16_t)((uint16_t)rx_gyro_data[0] | (uint16_t) rx_gyro_data[1] << 8) / 32768.f * 2000.f / rx_gyro_range_k;
    gyro_y_ = (int16_t)((uint16_t)rx_gyro_data[2] | (uint16_t) rx_gyro_data[3] << 8) / 32768.f * 2000.f / rx_gyro_range_k;
    gyro_z_ = (int16_t)((uint16_t)rx_gyro_data[4] | (uint16_t) rx_gyro_data[5] << 8) / 32768.f * 2000.f / rx_gyro_range_k;
}

void IMU::IMUCalculate() {

    AccCalculate();
    GyroCalculate();

    roll_acc_ = atan2(acc_y_, acc_z_) * 180. / pi ;
    pitch_acc_ = -atan2(acc_x_, sqrt(acc_y_ * acc_y_ + acc_z_ * acc_z_)) * 180. / pi;

    float roll_rad = roll_gyro_ * pi / 180;
    float pitch_rad = pitch_gyro_ * pi / 180;
    float yaw_rad = yaw_gyro_ * pi / 180; //弧度制

    float dot_yaw_gyro = sin(roll_rad) / cos(pitch_rad) * gyro_y_ + cos(roll_rad) / cos(pitch_rad) * gyro_z_;
    float dot_roll_gyro = gyro_x_ + sin(pitch_rad) * sin(roll_rad) / cos(pitch_rad) * gyro_y_ + cos(roll_rad) * sin(pitch_rad) / cos(pitch_rad) * gyro_z_;
    float dot_pitch_gyro = cos(roll_rad) * gyro_y_ - sin(roll_rad) * gyro_z_; //角速度积分

    yaw_gyro_ = yaw_gyro_ + dot_yaw_gyro * loop_rate;
    roll_gyro_ = (roll_gyro_ + dot_roll_gyro * loop_rate) * (1 - K) + roll_acc_ * K;
    pitch_gyro_ = (pitch_gyro_ + dot_pitch_gyro * loop_rate) * (1 - K) + pitch_acc_ * K;
}
