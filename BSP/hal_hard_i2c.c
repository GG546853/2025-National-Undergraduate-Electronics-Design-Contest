#include "hal_hard_i2c.h"

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c3;

int32_t HAL_Hardware_I2C_Write(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen)
{
    HAL_StatusTypeDef status;

    // --- ???? ---
    // ?: uint8_t device_address = (ucAddr << 1);
    // ??:
    uint8_t device_address = ucAddr;

    status = HAL_I2C_Mem_Write(&hi2c3, device_address, (uint16_t)ucReg, I2C_MEMADD_SIZE_8BIT, p_ucVal, uiLen, 100);

    if (status == HAL_OK)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int32_t HAL_Hardware_I2C_Read(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen)
{
    HAL_StatusTypeDef status;

    // --- ???? ---
    // ?: uint8_t device_address = (ucAddr << 1);
    // ??:
    uint8_t device_address = ucAddr;
    
    status = HAL_I2C_Mem_Read(&hi2c3, device_address, (uint16_t)ucReg, I2C_MEMADD_SIZE_8BIT, p_ucVal, uiLen, 100);

    if (status == HAL_OK)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}