#ifndef __HAL_HARD_I2C_H
#define __HAL_HARD_I2C_H

#include "main.h"
#include <stdint.h>

int32_t HAL_Hardware_I2C_Write(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen);
int32_t HAL_Hardware_I2C_Read(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen);

#endif