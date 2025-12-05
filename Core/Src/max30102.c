/*
 * max30102.c
 *
 *  Created on: Jul 23, 2025
 *      Author: ADE1HYD
 */


#include "max30102.h"
#include "semphr.h"

// I2C Address (7-bit address, left-shifted)
#define MAX30100_I2C_ADDR   (0x57 << 1)

// Register Addresses (Verified with MAX30100 Datasheet)
#define REG_INT_STATUS      0x00
#define REG_INT_ENABLE      0x01
#define REG_FIFO_WR_PTR     0x02
#define REG_OVF_COUNTER     0x03
#define REG_FIFO_RD_PTR     0x04
#define REG_FIFO_DATA       0x05
#define REG_MODE_CONFIG     0x06
#define REG_SPO2_CONFIG     0x07
#define REG_LED_CONFIG      0x09
#define REG_PART_ID         0xFF

// --- Private Variables ---
static I2C_HandleTypeDef *g_hi2c;
static QueueHandle_t g_data_queue;
static SemaphoreHandle_t g_data_ready_sem;

// --- Private Function Prototypes ---
static void max30100_task(void* argument);

bool max30100_init(I2C_HandleTypeDef *hi2c) {
    g_hi2c = hi2c;
    uint8_t reg_val;

    // Create FreeRTOS objects
    g_data_queue = xQueueCreate(16, sizeof(max30100_data_t)); // FIFO is 16 samples deep [cite: 427]
    g_data_ready_sem = xSemaphoreCreateBinary();
    if (g_data_queue == NULL || g_data_ready_sem == NULL) {
        return false;
    }

    // Check if the sensor is present by reading the Part ID
    // The datasheet Part ID is 0x11, but many clones use 0x15. We accept either.
    HAL_I2C_Mem_Read(g_hi2c, MAX30100_I2C_ADDR, REG_PART_ID, 1, &reg_val, 1, 100);
    if (reg_val != 0x11 && reg_val != 0x15) {
        return false;
    }

    // --- Configure the Sensor ---
    // 1. Reset the sensor
    reg_val = 0x40; // Set RESET bit [cite: 557]
    HAL_I2C_Mem_Write(g_hi2c, MAX30100_I2C_ADDR, REG_MODE_CONFIG, 1, &reg_val, 1, 100);
    HAL_Delay(100);

    // 2. Configure Interrupts: Enable "FIFO Almost Full" interrupt
    reg_val = 0x80; // A_FULL_EN [cite: 384]
    HAL_I2C_Mem_Write(g_hi2c, MAX30100_I2C_ADDR, REG_INT_ENABLE, 1, &reg_val, 1, 100);

    // 3. Configure SpO2: ADC range=16384nA, Sample Rate=100Hz, Pulse Width=400us (14-bit)
    reg_val = (0x02 << 5) | (0x01 << 2) | 0x01; // SPO2_ADC_RGE, SPO2_SR, LED_PW [cite: 576, 581, 589]
    HAL_I2C_Mem_Write(g_hi2c, MAX30100_I2C_ADDR, REG_SPO2_CONFIG, 1, &reg_val, 1, 100);

    // 4. Configure LED Pulse Amplitude (current)
    // Values are a starting point and may need tuning.
    reg_val = (0x02 << 4) | 0x02; // RED_PA=7.6mA, IR_PA=7.6mA [cite: 594, 596, 599]
    HAL_I2C_Mem_Write(g_hi2c, MAX30100_I2C_ADDR, REG_LED_CONFIG, 1, &reg_val, 1, 100);

    // 5. Configure Mode: Set to SpO2 mode
    reg_val = 0x03; // MODE=SpO2 enabled [cite: 566]
    HAL_I2C_Mem_Write(g_hi2c, MAX30100_I2C_ADDR, REG_MODE_CONFIG, 1, &reg_val, 1, 100);

    // 6. Clear FIFO by writing 0 to all pointers
    reg_val = 0x00;
    HAL_I2C_Mem_Write(g_hi2c, MAX30100_I2C_ADDR, REG_FIFO_WR_PTR, 1, &reg_val, 1, 100);
    HAL_I2C_Mem_Write(g_hi2c, MAX30100_I2C_ADDR, REG_OVF_COUNTER, 1, &reg_val, 1, 100);
    HAL_I2C_Mem_Write(g_hi2c, MAX30100_I2C_ADDR, REG_FIFO_RD_PTR, 1, &reg_val, 1, 100);

    // Create the dedicated FreeRTOS task for reading sensor data
    xTaskCreate(max30100_task, "MAX30100_Task", 256, NULL, 1, NULL);

    return true;
}

static void max30100_task(void* argument) {
    uint8_t fifo_data[4]; // One sample is 4 bytes (2 bytes IR, 2 bytes Red)
    max30100_data_t sample;

    for (;;) {
        // Block and wait for the interrupt to signal that data is ready
        if (xSemaphoreTake(g_data_ready_sem, portMAX_DELAY) == pdTRUE) {

            // Read the 4 bytes of FIFO data (one sample of IR and Red)
            if (HAL_I2C_Mem_Read(g_hi2c, MAX30100_I2C_ADDR, REG_FIFO_DATA, 1, fifo_data, 4, 100) == HAL_OK) {

                // Combine the bytes into 16-bit values
                sample.ir_value = (uint16_t)(fifo_data[0] << 8) | fifo_data[1];
                sample.red_value = (uint16_t)(fifo_data[2] << 8) | fifo_data[3];

                // Send the processed sample to the queue
                xQueueSend(g_data_queue, &sample, 0);
            }
        }
    }
}

void max30100_Interrupt_Callback(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(g_data_ready_sem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

bool max30100_get_data(max30100_data_t* data, uint32_t timeout_ms) {
    if (xQueueReceive(g_data_queue, data, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        return true;
    }
    return false;
}
