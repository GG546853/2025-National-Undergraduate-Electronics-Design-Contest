#include "main.h"
#include "uart_me.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "motor.h"

DecodeState currentState = STATE_WAIT_HEADER;
uint8_t rx_cmd = 0;
uint8_t rx_len = 0;
uint8_t rx_data_buffer[128];
uint8_t rx_data_idx = 0;

void Handle_Command(uint8_t cmd, uint8_t* data);


int16_t BytesToInt16(uint8_t high, uint8_t low) {
    return (int16_t)((high << 8) | low);
}

void Process_Vision_Packet() {
    uint8_t byte;
    
    while (CDC_Read_Byte(&byte)) {
        switch (currentState) {
            case STATE_WAIT_HEADER:
                if (byte == 0xAA) {
                    currentState = STATE_WAIT_CMD;
                }
                break;

            case STATE_WAIT_CMD:
                rx_cmd = byte;
                currentState = STATE_WAIT_LEN;
                break;

            case STATE_WAIT_LEN:
                rx_len = byte;
                rx_data_idx = 0;
                if (rx_len == 0) {
                    Handle_Command(rx_cmd, NULL); 
                    currentState = STATE_WAIT_HEADER;
                } else {
                    currentState = STATE_WAIT_DATA;
                }
                break;

            case STATE_WAIT_DATA:
                rx_data_buffer[rx_data_idx++] = byte;
                if (rx_data_idx >= rx_len) {
                    Handle_Command(rx_cmd, rx_data_buffer);
                    currentState = STATE_WAIT_HEADER;
                }
                break;
        }
    }
}

void Handle_Command(uint8_t cmd, uint8_t* data) {
    if (cmd == 0x01) { 
        uint8_t state = data[0]; 
        if(state == 0xF1) {
            // HAL_GPIO_WritePin(..., ..., GPIO_PIN_SET);
        } else {
        }
    }
    else if (cmd == 0x02) {
        vision_data.cur_x = BytesToInt16(data[0], data[1]);
				On_Vision_Data_Received(vision_data.cur_x);
        vision_data.cur_y = BytesToInt16(data[2], data[3]);
			  vision_data.is_tracking = 1;
        vision_data.last_update_tick = HAL_GetTick();
				
    }
    else if (cmd == 0x03) {
        vision_data.dist_cm = BytesToInt16(data[0], data[1]);
    }
		else if (cmd == 0x08) {
    vision_data.vel_x_pps = BytesToInt16(data[0], data[1]);
    vision_data.vel_y_pps = BytesToInt16(data[2], data[3]);
		}
}

