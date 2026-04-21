#ifndef __VL53L0X_H
#define __VL53L0X_H

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_def.h"
#include "stm32l4xx_hal_i2c.h"
#include <stdint.h>

#define VL53L0X_ADDR (0x29 << 1)

#define REG_SYSRANGE_START                              0x00
#define REG_SYSTEM_INTERRUPT_CONFIG_GPIO                0x0A // <--- TEGO BRAKOWAŁO
#define REG_RESULT_RANGE_STATUS                         0x14
#define REG_RESULT_INTERRUPT_STATUS_GPIO                0x13
#define REG_SYSTEM_INTERRUPT_CLEAR                      0x0B
#define REG_RESULT_RANGE_VAL                            0x1E

HAL_StatusTypeDef VL53L0X_Init(I2C_HandleTypeDef *hi2c);
uint16_t VL53L0X_ReadDistance(I2C_HandleTypeDef *hi2c);
void VL53L0X_StartContinous(I2C_HandleTypeDef *hi2c);

#endif