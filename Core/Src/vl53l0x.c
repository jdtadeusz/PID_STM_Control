#include "vl53l0x.h"
#include "i2c.h"
#include "stm32l4xx_hal_def.h"
#include "stm32l4xx_hal_i2c.h"
#include <stdint.h>

static uint16_t readings[5];
static uint8_t read_index = 0;

HAL_StatusTypeDef VL53L0X_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t val;

    // Verify Device ID
    if (HAL_I2C_Mem_Read(hi2c, VL53L0X_ADDR, 0xC0, 1, &val, 1, 100) != HAL_OK) return HAL_ERROR;

    // "Magic Initialization" sequence (mandated by STMicroelectronics documentation)
    uint8_t init_seq[][2] = {
        {0x88, 0x00}, {0x80, 0x01}, {0xFF, 0x01}, {0x00, 0x00},
        {0x91, 0x3C}, {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00}
    };

    for(int i = 0; i < 8; i++) {
        HAL_I2C_Mem_Write(hi2c, VL53L0X_ADDR, init_seq[i][0], 1, &init_seq[i][1], 1, 100);
    }

    // Configure interrupts and measurement mode
    val = 0x04; // New measurement ready status flag
    HAL_I2C_Mem_Write(hi2c, VL53L0X_ADDR, REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 1, &val, 1, 100);
    
    val = 0x01; // Clear any stale/pending interrupts
    HAL_I2C_Mem_Write(hi2c, VL53L0X_ADDR, REG_SYSTEM_INTERRUPT_CLEAR, 1, &val, 1, 100);

    uint8_t timing_budget[] = {0xFF, 0x01, 0x00, 0x01, 0xFF, 0x00, 0x80};
    HAL_I2C_Mem_Write(hi2c, VL53L0X_ADDR, 0x01, 1, timing_budget, 7, 100);
    
    return HAL_OK;
}

uint16_t VL53L0X_ReadDistance(I2C_HandleTypeDef *hi2c) {
    uint8_t data[2];
    uint16_t dist = 0;

    // Check if data conversion is complete
    volatile uint8_t status_reg;
    HAL_I2C_Mem_Read(hi2c, VL53L0X_ADDR, REG_RESULT_RANGE_STATUS, 1, &status_reg, 1, 10);

    if (!(status_reg & 0x01)) return 0; // If bit 0 is cleared, data is not ready yet

    if (HAL_I2C_Mem_Read(hi2c, VL53L0X_ADDR, REG_RESULT_RANGE_VAL, 1, data, 2, 50) == HAL_OK) {
        dist = (uint16_t)((data[0] << 8) | data[1]);
    }

    // Clear interrupt flag
    uint8_t clear_bit = 0x01;
    HAL_I2C_Mem_Write(hi2c, VL53L0X_ADDR, REG_SYSTEM_INTERRUPT_CLEAR, 1, &clear_bit, 1, 10);

    return dist;
}

void VL53L0X_StartContinous(I2C_HandleTypeDef *hi2c) {
    uint8_t start_cmd = 0x02; // 0x02 selects Continuous Measurement Mode
    HAL_I2C_Mem_Write(hi2c, VL53L0X_ADDR, 0x00, 1, &start_cmd, 1, 100);
}

uint16_t VL53L0X_GetDistance(I2C_HandleTypeDef *hi2c) {
    uint8_t start_cmd = 0x01;
    uint8_t status = 0;
    uint8_t data[2];

    // Trigger Single Shot measurement
    if(HAL_I2C_Mem_Write(hi2c, VL53L0X_ADDR, 0x00, 1, &start_cmd, 1, 100) != HAL_OK) return 0;

    // Wait for the sensor to accept the command
    uint32_t timeout = HAL_GetTick();
    while (HAL_GetTick() - timeout < 50) {
        HAL_I2C_Mem_Read(hi2c, VL53L0X_ADDR, 0x00, 1, &status, 1, 50);
        if (!(status & 0x01)) break; 
    }

    // Wait for Interrupt Status flag
    timeout = HAL_GetTick();
    while (HAL_GetTick() - timeout < 100) {
        HAL_I2C_Mem_Read(hi2c, VL53L0X_ADDR, 0x13, 1, &status, 1, 100);
        if (status & 0x07) break; // Bit 0, 1, or 2 asserts data readiness
    }

    // Fetch 2 bytes of distance data starting from RESULT_RANGE_STATUS (0x14) + 10-byte offset
    if (HAL_I2C_Mem_Read(hi2c, VL53L0X_ADDR, 0x1E, 1, data, 2, 100) == HAL_OK) {
        uint16_t raw_dist = (uint16_t)((data[0] << 8) | data[1]);

        // Clear interrupt flag
        uint8_t clear = 0x01;
        HAL_I2C_Mem_Write(hi2c, VL53L0X_ADDR, 0x0B, 1, &clear, 1, 100);

        if (raw_dist > 40 && raw_dist < 700) {
            return raw_dist;
        }
    }

    return 0;
}

uint16_t VL53L0X_ApplyFilter(uint16_t new_val) {
    readings[read_index] = new_val;
    read_index = (read_index + 1) % 5;
    uint32_t sum = 0;
    for(int i=0; i < 5; i++) sum += readings[i];
    
    return (uint16_t)(sum / 5);
}

uint16_t VL53L0X_ReadContinuousFast(I2C_HandleTypeDef *hi2c) {
    uint8_t status = 0;
    uint8_t data[2];

    // Poll data readiness flag
    if (HAL_I2C_Mem_Read(hi2c, VL53L0X_ADDR, 0x13, 1, &status, 1, 2) != HAL_OK) return 0;
    
    // Fall through and exit if the sensor is busy (not ready)
    if ((status & 0x07) == 0) return 0; 

    // Retrieve measurement conversion results
    if (HAL_I2C_Mem_Read(hi2c, VL53L0X_ADDR, 0x1E, 1, data, 2, 2) == HAL_OK) {
        uint16_t dist = (uint16_t)((data[0] << 8) | data[1]);

        // Clear interrupt flag
        uint8_t clear = 0x01;
        HAL_I2C_Mem_Write(hi2c, VL53L0X_ADDR, 0x0B, 1, &clear, 1, 2);

        return dist;
    }
    return 0;
}