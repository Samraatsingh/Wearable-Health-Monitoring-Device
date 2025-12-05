/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "m95640_driver.h"
#include "max30102.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>
#include "hc05.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// State machine for our ThingSpeak client application
typedef enum {
    STATE_CLIENT_INIT,
    STATE_CLIENT_CONNECT_WIFI,
    STATE_CLIENT_START_CONNECTION,
    STATE_CLIENT_SEND_DATA,
    STATE_CLIENT_CLOSE_CONNECTION,
    STATE_CLIENT_WAIT,
    STATE_CLIENT_ERROR
} AppState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define EEPROM_ADDRESS  0x0000

// --- WiFi Credentials ---
#define WIFI_SSID       "network7"
#define WIFI_PASSWORD   "1029384757"

// --- ThingSpeak Configuration ---
#define THINGSPEAK_API_KEY "VM24KC8S1EDVA8AV" // IMPORTANT: REPLACE WITH YOUR KEY
#define THINGSPEAK_HOST "api.thingspeak.com"
#define THINGSPEAK_PORT 80
#define UPDATE_INTERVAL_MS 20000 // ThingSpeak free plan allows updates every 15s. 20s is safer.

// --- ESP32 Communication Buffer ---
#define RX_BUFFER_SIZE 1024
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
bool isTrue = false;
char g_read_string[5];
uint16_t ir_data[16], red_data[16];

// --- ESP32 AT Command Client Variables ---
uint8_t uart_rx_buffer[1]; // Single byte buffer for UART IT
uint8_t app_rx_buffer[RX_BUFFER_SIZE];  // Application buffer for processing responses
volatile uint16_t app_rx_write_pos = 0;
volatile AppState app_state = STATE_CLIENT_INIT;


// MAX30100 Register addresses and functions (as provided by you)
#define MAX30100_I2C_ADDRESS (0x57 << 1)
#define REG_INT_STATUS      0x00
#define REG_INT_ENABLE      0x01
#define REG_FIFO_WR_PTR     0x02
#define REG_OVF_COUNTER     0x03
#define REG_FIFO_RD_PTR     0x04
#define REG_FIFO_DATA       0x05
#define REG_MODE_CONFIG     0x06
#define REG_SPO2_CONFIG     0x07
#define REG_LED_CONFIG      0x09

HAL_StatusTypeDef MAX30100_WriteReg(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    return HAL_I2C_Master_Transmit(&hi2c1, MAX30100_I2C_ADDRESS, data, 2, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MAX30100_ReadReg(uint8_t reg, uint8_t *data, uint16_t len) {
    return HAL_I2C_Mem_Read(&hi2c1, MAX30100_I2C_ADDRESS, reg, 1, data, len, HAL_MAX_DELAY);
}

void MAX30100_Init(void) {
    MAX30100_WriteReg(REG_MODE_CONFIG, 0x40);
    HAL_Delay(100);
    MAX30100_WriteReg(REG_FIFO_WR_PTR, 0x00);
    MAX30100_WriteReg(REG_OVF_COUNTER, 0x00);
    MAX30100_WriteReg(REG_FIFO_RD_PTR, 0x00);
    MAX30100_WriteReg(REG_INT_ENABLE, 0x10);
    MAX30100_WriteReg(REG_MODE_CONFIG, 0x03);
    MAX30100_WriteReg(REG_SPO2_CONFIG, 0x41);
    MAX30100_WriteReg(REG_LED_CONFIG, 0x33);
}

void MAX30100_ReadFIFO(uint16_t *ir, uint16_t *red, uint8_t num_samples) {
    uint8_t fifo_data[4 * num_samples];
    uint8_t wr_ptr, rd_ptr;
    MAX30100_ReadReg(REG_FIFO_WR_PTR, &wr_ptr, 1);
    MAX30100_ReadReg(REG_FIFO_RD_PTR, &rd_ptr, 1);
    int8_t samples_to_read = wr_ptr - rd_ptr;
    if (samples_to_read < 0) samples_to_read += 32;
    if (samples_to_read > num_samples) samples_to_read = num_samples;
    if (samples_to_read > 0) {
        MAX30100_ReadReg(REG_FIFO_DATA, fifo_data, 4 * samples_to_read);
        for (uint8_t i = 0; i < samples_to_read; i++) {
            ir[i] = (fifo_data[i * 4] << 8) | fifo_data[i * 4 + 1];
            red[i] = (fifo_data[i * 4 + 2] << 8) | fifo_data[i * 4 + 3];
        }
    }
}

// --- ESP32 Helper Functions ---
void clear_app_buffer(void) {
    memset(app_rx_buffer, 0, RX_BUFFER_SIZE);
    app_rx_write_pos = 0;
}

void esp32_send_command(const char* cmd) {
    clear_app_buffer();
    HAL_UART_Transmit(&huart3, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
}

bool esp32_wait_for_response(const char* expected_response, uint32_t timeout_ms) {
    uint32_t start_time = HAL_GetTick();
    while ((HAL_GetTick() - start_time) < timeout_ms) {
        if (strstr((char*)app_rx_buffer, expected_response) != NULL) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return false;
}

int getRandom_0_to_50(void) {
    // rand() % 51 will generate a number between 0 and 50.
    return rand() % 51;
}

int getRandom_90_to_110(void) {
    // 1. Calculate the range size: 110 - 90 + 1 = 21
    // 2. Generate a random number from 0 to 20: rand() % 21
    // 3. Add the minimum value (90) to shift the range.
    return (rand() % 21) + 90;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void thingSpeakTask(void *param);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  SpiEepromInit_HAL();
  hc05_init(&huart2);
  srand(time(NULL));
//  MAX30100_Init();

  // Start listening for data from ESP32 using interrupts.
  HAL_UART_Receive_IT(&huart3, uart_rx_buffer, 1);

  // Create the FreeRTOS task for sending data to ThingSpeak.
  xTaskCreate(thingSpeakTask, "ThingSpeakTask", 2048, NULL, 1, NULL);

  // Start the FreeRTOS scheduler.
  vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// This is the main task for sending data to ThingSpeak
void thingSpeakTask(void *param)
{
    char command_buffer[128];
    char http_request[256];
    char data_payload[128];

    // Initial state setup
    app_state = STATE_CLIENT_INIT;

    while(1)
    {
        switch(app_state)
        {
            case STATE_CLIENT_INIT:
                esp32_send_command("AT+RST\r\n");
                vTaskDelay(pdMS_TO_TICKS(2000));
                esp32_send_command("ATE0\r\n");
                if(esp32_wait_for_response("OK", 2000)) {
                    app_state = STATE_CLIENT_CONNECT_WIFI;
                } else {
                    app_state = STATE_CLIENT_ERROR;
                }
                break;

            case STATE_CLIENT_CONNECT_WIFI:
                esp32_send_command("AT+CWMODE=1\r\n");
                if(!esp32_wait_for_response("OK", 2000)) { app_state = STATE_CLIENT_ERROR; break; }

                sprintf(command_buffer, "AT+CWJAP=\"%s\",\"%s\"\r\n", WIFI_SSID, WIFI_PASSWORD);
                esp32_send_command(command_buffer);
                if(esp32_wait_for_response("WIFI GOT IP", 15000)) {
                    app_state = STATE_CLIENT_WAIT; // Successfully connected, go to wait state
                } else {
                    app_state = STATE_CLIENT_ERROR;
                }
                break;

            case STATE_CLIENT_START_CONNECTION:
                sprintf(command_buffer, "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", THINGSPEAK_HOST, THINGSPEAK_PORT);
                esp32_send_command(command_buffer);
                if(esp32_wait_for_response("CONNECT", 5000)) {
                    app_state = STATE_CLIENT_SEND_DATA;
                } else {
                    app_state = STATE_CLIENT_ERROR; // Failed to connect to server
                }
                break;

            case STATE_CLIENT_SEND_DATA:
            {
                // Step 1: Get sensor data (using constant values for now)
                // MAX30100_ReadFIFO(ir_data, red_data, 1);
                uint16_t ir_value = getRandom_0_to_50();
                uint16_t red_value = getRandom_90_to_110();

                // Step 2: Format the data payload for the request body
                sprintf(data_payload, "api_key=%s&field1=%u&field2=%u", THINGSPEAK_API_KEY, ir_value, red_value);

                // Step 3: Format the full HTTP POST request
                sprintf(http_request,
                        "POST /update HTTP/1.1\r\n"
                        "Host: %s\r\n"
                        "Content-Type: application/x-www-form-urlencoded\r\n"
                        "Content-Length: %d\r\n"
                        "\r\n"
                        "%s",
                        THINGSPEAK_HOST, strlen(data_payload), data_payload);

                // Step 4: Send the request length to ESP32
                sprintf(command_buffer, "AT+CIPSEND=%d\r\n", strlen(http_request));
                esp32_send_command(command_buffer);

                // Step 5: Wait for ">" and send the actual request
                if(esp32_wait_for_response(">", 2000)) {
                    esp32_send_command(http_request);
                    if(esp32_wait_for_response("SEND OK", 5000)) {
                        app_state = STATE_CLIENT_CLOSE_CONNECTION;
                    } else { app_state = STATE_CLIENT_ERROR; }
                } else { app_state = STATE_CLIENT_ERROR; }
            }
            break;

            case STATE_CLIENT_CLOSE_CONNECTION:
                esp32_send_command("AT+CIPCLOSE\r\n");
                // We don't strictly need to wait for the response. Move to wait state.
                app_state = STATE_CLIENT_WAIT;
                break;

            case STATE_CLIENT_WAIT:
                // Wait for the defined interval before sending the next update
                vTaskDelay(pdMS_TO_TICKS(UPDATE_INTERVAL_MS));
                app_state = STATE_CLIENT_START_CONNECTION; // Time to send data again
                break;

            case STATE_CLIENT_ERROR:
                // An error occurred. Toggle an LED and try to re-initialize after a delay.
//                HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
                vTaskDelay(pdMS_TO_TICKS(500));
//                HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
                vTaskDelay(pdMS_TO_TICKS(5000));
                app_state = STATE_CLIENT_INIT; // Try to recover
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// UART Interrupt Service Routine Callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        if (app_rx_write_pos < sizeof(app_rx_buffer) - 1)
        {
            app_rx_buffer[app_rx_write_pos++] = uart_rx_buffer[0];
        }
        else
        {
            // Buffer overflow, clear it and start over
            clear_app_buffer();
            app_rx_buffer[app_rx_write_pos++] = uart_rx_buffer[0];
        }
        // Re-arm the UART interrupt to receive the next byte
        HAL_UART_Receive_IT(&huart3, uart_rx_buffer, 1);
    }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
