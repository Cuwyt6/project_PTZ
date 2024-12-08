//
// Created by lenovo on 24-11-29.
//
#include "iwdg.h"
#include "tim.h"
#include "RC.h"
#include "motor.h"
#include "can.h"
#include "IMU.h"


uint16_t yaw_motor_control_stdid = GM6020_StdID_CONTROL_VOLTAGE1;
uint16_t pitch_motor_control_stdid = GM6020_StdID_CONTROL_VOLTAGE1;
uint16_t shooter1_control_stdid = M3508_StdID_CONTROL1;
uint16_t shooter2_control_stdid = M3508_StdID_CONTROL2;

extern uint8_t tx_data1[8];
extern uint8_t tx_data2[8];

uint8_t rx_data[8];

IMU imu(0.0004);
RC rc;

float YawFeedForward(float angle_imu){
    return 0;
}
float PitchFeedForward(float angle_imu){
    return (-7e-5f * angle_imu * angle_imu - 0.0032f * angle_imu + 0.1959f);
}


//motor yaw_motor(motor::GM6020, yaw_motor_control_stdid, YAW_MOTOR_ID,
//                PID(0.006,0,0,0,GM6020_MAXCURRENT),
//                PID(50,1.5, 380,10,600), false ,0, &YawFeedForward);

// ecd作为fdb
//motor pitch_motor(Motor::GM6020, GM6020_RATIO, GM6020_MAXCURRENT,pitch_motor_control_stdid,PITCH_MOTOR_ID,
//                  PID(0.0045,0,0,0,0,GM6020_MAXCURRENT),
//                  PID(45,1.2,200,0.2,25,600),1 , 217.36174);

motor pitch_motor(motor::GM6020, pitch_motor_control_stdid,PITCH_MOTOR_ID,
                  PID(0.0045,0,0,0,GM6020_MAXCURRENT),
                  PID(90,0.1,45
                      ,50,600),true , 217.36174, &PitchFeedForward);

//motor shooter_motor1(Motor::M3508, shooter1_control_stdid,SHOOTER1_MOTOR_ID,
//                     PID(0,0,0,0,0,M3508_MAXCURRENT),
//                     PID(0,0,0,0,0,0),0 ,0);

//motor shooter_motor2(Motor::M3508, shooter2_control_stdid, SHOOTER2_MOTOR_ID,
//                     PID(0,0,0,0,0,M3508_MAXCURRENT),
//                     PID(0,0,0,0,0,0),0 , 0);

void RCHandle();
void IMUHandle();
void MotorHandle();

CAN_RxHeaderTypeDef rx_header;

void ControlLoop(){
    HAL_IWDG_Refresh(&hiwdg);
    // 将rc的 add_on量加给 target
    RCHandle();
    // IMU 解算 & 赋值给电机
    IMUHandle();
    // can 控制电机
    MotorHandle();
}

void MotorHandle(){
    pitch_motor.Handle();
   //todo yaw_motor.Handle();

    CAN_TxHeaderTypeDef TxHeader1 = {(uint16_t)(GM6020_StdID_CONTROL_VOLTAGE1), 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE};
    HAL_CAN_AddTxMessage(&hcan1, &TxHeader1, tx_data1, CAN_FilterFIFO0);

//  CAN_TxHeaderTypeDef TxHeader2 = {(uint16_t)(GM6020_StdID_CONTROL_VOLTAGE2), 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE};
//  HAL_CAN_AddTxMessage(&hcan1, &TxHeader2, tx_data2, CAN_FilterFIFO0);
}

//
void RCHandle(){
    // 右下关闭所有电机
//  if(rc.switch_.r == RC::DOWN){
//    yaw_motor.Stop();
//    pitch_motor.Stop();
//    shooter_motor1.Stop();
//    shooter_motor2.Stop();
//  }
    // 右中/上控制电机 左拨杆上下pitch，左右yaw
//  else{
    /// yaw_motor.RCControl(rc.yaw_gain_ * rc.channel_.l_row);
    //pitch_motor.RCControl(rc.pitch_gain_ * rc.channel_.l_col);
//  }

}

void IMUHandle(){
    imu.IMUCalculate();
    pitch_motor.SetIMUAngle(imu.IMUPitch());
    ///yaw_motor.SetIMUAngle(imu.IMUYaw());
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if(htim == &htim6){
        ControlLoop();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if (huart == &huart3){
        rc.ReadDbus();
        rc.ParseDbus();
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    if (hcan == &hcan1){
        //uint8_t rx_data[8];
        HAL_CAN_GetRxMessage(&hcan1, CAN_FilterFIFO0, &rx_header, rx_data);

        if (rx_header.StdId == GM6020_StdID_FEEDBACK + YAW_MOTOR_ID); ///yaw_motor.CanRxMsgCallback(rx_data)
        else if (rx_header.StdId == GM6020_StdID_FEEDBACK + PITCH_MOTOR_ID) pitch_motor.CanRxMsgCallback(rx_data);
//    else if (rx_header.StdId == shooter1_feedback_stdid) shooter_motor1.CanRxMsgCallback(rx_data);
//    else if (rx_header.StdId == shooter2_feedback_stdid) shooter_motor2.CanRxMsgCallback(rx_data);
    }
}