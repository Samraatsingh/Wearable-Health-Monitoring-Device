/*
 * max30102.h
 *
 *  Created on: Jul 23, 2025
 *      Author: ADE1HYD
 */

#include "stm32f4xx_hal.h" // Change to your specific STM32 series
#include "stdbool.h"
#include "FreeRTOS.h"
#include "queue.h"

// Data structure to hold a single sample of IR and Red data
typedef struct {
    uint16_t ir_value;
    uint16_t red_value;
} max30100_data_t;

/**
 * @brief Initializes the MAX30100 sensor and the FreeRTOS components.
 * @param hi2c Pointer to the I2C handle.
 * @return true on success, false on failure.
 */
bool max30100_init(I2C_HandleTypeDef *hi2c);

/**
 * @brief Retrieves a data sample from the sensor queue.
 * @param data Pointer to a max30100_data_t struct to store the data.
 * @param timeout_ms The maximum time to wait for data.
 * @return true if data was received, false on timeout.
 */
bool max30100_get_data(max30100_data_t* data, uint32_t timeout_ms);

/**
 * @brief This function should be called from the HAL_GPIO_EXTI_Callback
 * when an interrupt from the sensor's INT pin is detected.
 */
void max30100_Interrupt_Callback(void);
