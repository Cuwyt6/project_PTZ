//
// Created by lenovo on 24-11-29.
//
#include "RC.h"
#include "tool.h"

RC::RC():yaw_gain_(10.0), pitch_gain_(10.0){}

void RC::ReadDbus() {
    HAL_UART_Receive_DMA(&huart3, this->rx_buf_, RC_RX_BUF_SIZE);
    for (int i = 0; i < sizeof(this->rx_buf_); i++){
        this->rx_data_[i] = this->rx_buf_[i];
    }
}

void RC::ParseDbus() {
    // RCChannel
    uint16_t r_row_raw = (((int16_t)rx_data_[1] << 8) | ((int16_t)rx_data_[0])) & 0x07FF;
    uint16_t r_col_raw = ((int16_t)rx_data_[2] << 5 | (int16_t)rx_data_[1] >> 3) & 0x07FF;
    uint16_t l_row_raw = (((int16_t)rx_data_[4] << 10) | ((int16_t)rx_data_[3] << 2) | ((int16_t)rx_data_[2] >> 6)) & 0x07FF;
    uint16_t l_col_raw = (((int16_t)rx_data_[5] << 7) | ((int16_t)rx_data_[4] >> 1) | ((int16_t)rx_data_[3] >> 3)) & 0x07FF;
    this->channel_.r_row = LinearMap((int)r_row_raw, 1024, 1684, 0.0f, 1.0f);
    this->channel_.r_col = LinearMap((int)r_col_raw, 1024, 1684, 0.0f, 1.0f);
    this->channel_.l_row = LinearMap((int)l_row_raw, 1024, 1684, 0.0f, 1.0f);
    this->channel_.l_col = LinearMap((int)l_col_raw, 1024, 1684, 0.0f, 1.0f);
    // RCSwitch
    uint8_t s1_raw = (rx_data_[5] >> 6) & 0x0003;
    uint8_t s2_raw = (rx_data_[5] >> 4) & 0x0003;
    switch (s2_raw){
        case 1:
            this->switch_.r = UP;
            break;
        case 2:
            this->switch_.r = DOWN;
            break;
        case 3:
            this->switch_.r = MID;
            break;
    }
    switch (s1_raw){
        case 1:
            this->switch_.l = UP;
            break;
        case 2:
            this->switch_.l = DOWN;
            break;
        case 3:
            this->switch_.l = MID;
            break;
    }
}