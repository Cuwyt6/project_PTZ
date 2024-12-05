//
// Created by lenovo on 24-11-13.
//
#include "motor.h"
#include "tool.h"

#define pi 3.14159

uint8_t tx_data1[8];
uint8_t tx_data2[8];

motor::motor(motor::MotorType type, uint16_t StdID, uint8_t ID, const PID& pid_vel, const PID& pid_ang, bool is_imu_fdb, float ecd_angle,float (*feed_forward)(float angle_imu))
        : motor_type_(type), control_stdid_(StdID), id_(ID),
          pid_vel_(pid_vel),
          pid_ang_(pid_ang), ecd_angle_(ecd_angle), is_imu_fdb_(is_imu_fdb),
          FeedForward(feed_forward)
{
    angle_ = 0.0f; 	        // deg 输出端累计转动角度
    delta_angle_ = 0.0f; 		// deg 输出端新转动的角度
    last_ecd_angle_ = 0.0f;	// deg 上次电机编码器角度
    delta_ecd_angle_ = 0.0f; 	// deg 编码器端新转动的角度
    rotate_speed_ = 0.0f;         // dps 反馈转子转速
    torque_ = 0.0f;               // 反馈转矩
    temp_ = 0.0f;                 // 反馈电机温度

    ref_vel_ = 0.0f;              // 单环控制预期转速
    target_current_ = 0.0f;       // 目标输入电流

    angle_imu_ = 0.0f;

    stop_flag_ = false;
    forward_current_ = 0.0f;
    forward_voltage_ = 0.0f;

    if(type == GM6020){
        ratio_ = GM6020_RATIO;
        output_max_ = GM6020_MAXCURRENT;
    }
    else if(type == M3508){
        ratio_ = M3508_RATIO;
        output_max_ = M3508_MAXCURRENT;
    }
    else if(type == M2006){
        ratio_ = M2006_RATIO;
        output_max_ = M2006_MAXCURRENT;
    }
}

void motor::CanRxMsgCallback(const uint8_t rx_data[8]){

    last_ecd_angle_ = ecd_angle_;
    int16_t ecd_angle = (int16_t)((uint16_t)rx_data[0] << 8) | rx_data[1];
    ecd_angle_ = LinearMap(ecd_angle, (int16_t)0, (int16_t)8191, 0.0, 360.0);

    float delta = ecd_angle_ - last_ecd_angle_;
    // 编码器变化角度过零点修正
    if (delta < -180.0) delta_ecd_angle_ = delta + 360.0;
    else if (delta > 180.0) delta_ecd_angle_ = delta - 360.0;
    else delta_ecd_angle_ = delta;

    int16_t rotate_speed_rpm = (int16_t)((uint16_t)rx_data[2] << 8) | rx_data[3];
    rotate_speed_ = rotate_speed_rpm * 6.0;

    delta_angle_ = delta_ecd_angle_ / ratio_;
    angle_ += delta_angle_;

    torque_ = (int16_t)((uint16_t)rx_data[4] << 8) | rx_data[5];

    if (motor_type_ == M3508 || motor_type_ == GM6020) temp_ = (int16_t)rx_data[6];

    CalculatePID();
}

void motor::CalculatePID(){
    if (!stop_flag_){
        if(is_imu_fdb_){
            ref_vel_ = pid_ang_.calc(ref_ang_, angle_imu_);
            forward_voltage_ = FeedForward(angle_imu_);
            forward_current_ = FeedForward(angle_imu_);
        }
        else{
            ref_ang_ = 30;
            ref_vel_ = pid_ang_.calc(ref_ang_, angle_);
            //forward_voltage_ = FeedForward(angle_);
            //forward_current_ = FeedForward(angle_imu_);
        }
        target_current_ = pid_vel_.calc(ref_vel_, rotate_speed_);
    }
    else target_current_= 0.0;

//  error_acc_gyro = angle_ - angle_imu_;

    if(control_stdid_ == GM6020_StdID_CONTROL_VOLTAGE1 || control_stdid_ == GM6020_StdID_CONTROL_VOLTAGE2){
        current_tx_ = LinearMap(target_current_ + forward_voltage_, -output_max_, output_max_, -25000, 25000);
    }
    else{
        current_tx_ = LinearMap(target_current_ + forward_current_, -output_max_, output_max_, -16384, 16384);
    }
}

void motor::Handle() {
    if(id_ < 0x05){
        tx_data1[2 * id_ - 1] = current_tx_;
        tx_data1[2 * id_ - 2] = current_tx_ >> 8;
    }
    else{
        tx_data2[2 * (id_ - 4) - 1] = current_tx_;
        tx_data2[2 * (id_ - 4) - 2] = current_tx_ >> 8;
    }
}

void motor::Stop(){ stop_flag_ = true;}

void motor::RCControl(float channel_data) {
    stop_flag_ = false;
    ref_ang_ += channel_data;
}

void motor::SetIMUAngle(float angle) {
    angle_imu_ = angle;
}

