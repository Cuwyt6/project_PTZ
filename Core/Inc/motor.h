//
// Created by lenovo on 24-11-13.
//
#include "main.h"
#include "PID.h"

#ifndef PTZ_MOTOR_H
#define PTZ_MOTOR_H
#ifdef __cplusplus
extern "C" {
#endif

//3种电机的标识符(电调反馈报文用)
#define GM6020_StdID_FEEDBACK 0x204
#define M3508_StdID_FEEDBACK 0x200
#define M2006_StdID_FEEDBACK 0x200

//3种电机的标识符(电调接收报文用)
#define GM6020_StdID_CONTROL_CURRENT1 0x1FE
#define GM6020_StdID_CONTROL_CURRENT2 0x2FE
#define GM6020_StdID_CONTROL_VOLTAGE1 0x1FF
#define GM6020_StdID_CONTROL_VOLTAGE2 0x2FF
#define M3508_StdID_CONTROL1 0x200
#define M3508_StdID_CONTROL2 0x1FF
#define M2006_StdID_CONTROL1 0x200
#define M2006_StdID_CONTROL2 0x1FF

//电机减速比
#define GM6020_RATIO 1.0
#define M3508_RATIO 36.0
#define M2006_RATIO 19.2032

//转矩电流最大值
#define GM6020_MAXCURRENT 3.0
#define M3508_MAXCURRENT 20.0
#define M2006_MAXCURRENT 10.0

#define YAW_MOTOR_ID 0x03
#define PITCH_MOTOR_ID 0x01
#define SHOOTER1_MOTOR_ID 0x01
#define SHOOTER2_MOTOR_ID 0x02

class motor {
private:

    float ratio_;                 // 电机减速比
    float angle_;                 // deg 输出端累计转动角度
    float delta_angle_;           // deg 输出端新转动的角度
    float ecd_angle_;             // deg 当前电机编码器角度
    float last_ecd_angle_;        // deg 上次电机编码器角度
    float delta_ecd_angle_;       // deg 编码器端新转动的角度
    float rotate_speed_;          // dps 反馈转子转速
    int16_t torque_;                // 反馈转矩
    int16_t temp_;                  // °C 反馈电机温度

    int current_tx_;

    float ref_vel_;               // 单环控制预期速度
    float ref_ang_;               // 双环控制预期角度
    float target_current_;        // 目标输入电流
    float output_max_;            // 最大电流

    uint16_t control_stdid_;
    uint8_t id_;

    bool stop_flag_;
    bool is_imu_fdb_;

    float angle_imu_; //imu解算角度

public:
    enum MotorType{
        M2006, M3508, GM6020
    } motor_type_; //三种电机

    PID pid_vel_; //速度环
    PID pid_ang_; //位置环
    float forward_current_;
    float forward_voltage_;

    explicit motor(MotorType type, uint16_t StdID, uint8_t ID,const PID& pid_vel,  const PID& pid_ang,
                   bool is_imu_fdb, float ecd_angle, float (*feed_forward)(float angle_imu));
    void CanRxMsgCallback(const uint8_t rx_data[8]); //接收到一个新的 CAN 消息时触发的回调函数
    void Stop(); //急停
    void RCControl(float channel_data); //
    void CalculatePID();
    void Handle(); //发给电调的信息处理

    float (*FeedForward)(float angle_imu_);

    void SetIMUAngle(float angle);
};





#ifdef __cplusplus
}
#endif
#endif //PTZ_MOTOR_H
