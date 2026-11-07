#ifndef __UART_ME_H__
#define __UART_ME_H__

#include "main.h"

typedef enum {
    STATE_WAIT_HEADER,
    STATE_WAIT_CMD,
    STATE_WAIT_LEN,
    STATE_WAIT_DATA
} DecodeState;

void Process_Vision_Packet();
void Handle_Command(uint8_t cmd, uint8_t* data);

#endif