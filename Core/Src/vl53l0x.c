#include "vl53l0x.h"
#include "stm32l4xx_hal_def.h"
#include "stm32l4xx_hal_i2c.h"
#include <stdint.h>

HAL_StatusTypeDef VL53L0X_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t val;

    // Sprawdź ID 
    if (HAL_I2C_Mem_Read(hi2c, VL53L0X_ADDR, 0xC0, 1, &val, 1, 100) != HAL_OK) return HAL_ERROR;

    // Sekwencja "Magic Initialization" (wymagana przez dokumentację ST)
    uint8_t init_seq[][2] = {
        {0x88, 0x00}, {0x80, 0x01}, {0xFF, 0x01}, {0x00, 0x00},
        {0x91, 0x3C}, {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00}
    };

    for(int i = 0; i < 8; i++) {
        HAL_I2C_Mem_Write(hi2c, VL53L0X_ADDR, init_seq[i][0], 1, &init_seq[i][1], 1, 100);
    }

    // Konfiguracja przerwań i trybu pomiaru
    val = 0x04; // Nowy pomiar gotowy
    HAL_I2C_Mem_Write(hi2c, VL53L0X_ADDR, REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 1, &val, 1, 100);
    
    val = 0x01; // Czyścimy ewentualne zaległe przerwania
    HAL_I2C_Mem_Write(hi2c, VL53L0X_ADDR, REG_SYSTEM_INTERRUPT_CLEAR, 1, &val, 1, 100);

    uint8_t timing_budget[] = {0xFF, 0x01, 0x00, 0x01, 0xFF, 0x00, 0x80};
    HAL_I2C_Mem_Write(hi2c, VL53L0X_ADDR, 0x01, 1, timing_budget, 7, 100);
    
    return HAL_OK;
}

uint16_t VL53L0X_ReadDistance(I2C_HandleTypeDef *hi2c) {
    uint8_t data[2];
    uint16_t dist = 0;

    // Sprawdzanie czy dane są gotowe
    volatile uint8_t status_reg;
    HAL_I2C_Mem_Read(hi2c, VL53L0X_ADDR, REG_RESULT_RANGE_STATUS, 1, &status_reg, 1, 10);

    if (!(status_reg & 0x01)) return 0; // Jeśli bit 0 jest czysty, dane nie są gotowe


    if (HAL_I2C_Mem_Read(hi2c, VL53L0X_ADDR, REG_RESULT_RANGE_VAL, 1, data, 1, 50) == HAL_OK) {
        dist = (uint16_t)((data[0] << 8) | data[1]);
    }

    // Czyszczenie przerwania
    uint8_t clear_bit = 0x01;
    HAL_I2C_Mem_Write(hi2c, VL53L0X_ADDR, REG_SYSTEM_INTERRUPT_CLEAR, 1, &clear_bit, 1, 10);


    return dist;
}

void VL53L0X_StartContinous(I2C_HandleTypeDef *hi2c) {
    uint8_t start_cmd = 0x02; // tryb ciągły
    HAL_I2C_Master_Transmit(hi2c, VL53L0X_ADDR, &start_cmd, 1, 100);
}