# STM32 FreeRTOS IoT Health Monitoring System (ThingSpeak)

An **STM32 + FreeRTOS based IoT application** that reads sensor data, communicates with external peripherals, and uploads data to **ThingSpeak** using an **ESP32 (AT command mode)** over UART.

---

## 🚀 Features

- FreeRTOS-based multitasking architecture
- ESP32 WiFi communication using AT commands
- Cloud data upload to ThingSpeak
- MAX30100 / MAX30102 pulse oximeter integration
- HC-05 Bluetooth module support
- SPI EEPROM (M95640) support
- State-machine-driven IoT client
- UART interrupt-based communication

---

## 🧠 System Architecture

```
Sensor Data (MAX30100 / Random)
          |
          v
     STM32 + FreeRTOS
          |
     UART (AT Commands)
          |
        ESP32
          |
     ThingSpeak Cloud
```

---

## 🧩 Hardware Used

| Component | Description |
|---------|------------|
| STM32 MCU | Main controller (HAL based) |
| ESP32 | WiFi module (AT firmware) |
| MAX30100 / MAX30102 | Pulse oximeter sensor |
| HC-05 | Bluetooth module |
| M95640 | SPI EEPROM |
| UART | ESP32 & Bluetooth |
| I2C | Sensor interface |

---

## 🛠 Software Stack

- STM32 HAL
- FreeRTOS
- ESP32 AT Commands
- ThingSpeak REST API
- UART Interrupts
- I2C / SPI Drivers

---

## 📂 Project Structure

```
├── Core
│   ├── Src
│   │   ├── main.c
│   │   ├── gpio.c
│   │   ├── i2c.c
│   │   ├── spi.c
│   │   ├── usart.c
│   ├── Inc
│
├── Drivers
├── Middlewares
│   └── FreeRTOS
├── LICENSE
└── README.md
```

---

## 🔄 Application State Machine

```c
STATE_CLIENT_INIT
STATE_CLIENT_CONNECT_WIFI
STATE_CLIENT_START_CONNECTION
STATE_CLIENT_SEND_DATA
STATE_CLIENT_CLOSE_CONNECTION
STATE_CLIENT_WAIT
STATE_CLIENT_ERROR
```

---

## 🌐 ThingSpeak Configuration

```c
#define THINGSPEAK_API_KEY "YOUR_API_KEY"
#define THINGSPEAK_HOST "api.thingspeak.com"
#define THINGSPEAK_PORT 80
#define UPDATE_INTERVAL_MS 20000
```

---

## 👨‍💻 Author

**  Vivek  Kumar Singh**
Embedded Systems | STM32 | FreeRTOS | IoT
