################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/croutine.c \
../Core/Src/event_groups.c \
../Core/Src/gpio.c \
../Core/Src/hc05.c \
../Core/Src/heap_4.c \
../Core/Src/i2c.c \
../Core/Src/list.c \
../Core/Src/m95640_driver.c \
../Core/Src/main.c \
../Core/Src/max30102.c \
../Core/Src/port.c \
../Core/Src/queue.c \
../Core/Src/spi.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_hal_timebase_tim.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/stream_buffer.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/tasks.c \
../Core/Src/timers.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/croutine.o \
./Core/Src/event_groups.o \
./Core/Src/gpio.o \
./Core/Src/hc05.o \
./Core/Src/heap_4.o \
./Core/Src/i2c.o \
./Core/Src/list.o \
./Core/Src/m95640_driver.o \
./Core/Src/main.o \
./Core/Src/max30102.o \
./Core/Src/port.o \
./Core/Src/queue.o \
./Core/Src/spi.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_hal_timebase_tim.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/stream_buffer.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/tasks.o \
./Core/Src/timers.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/croutine.d \
./Core/Src/event_groups.d \
./Core/Src/gpio.d \
./Core/Src/hc05.d \
./Core/Src/heap_4.d \
./Core/Src/i2c.d \
./Core/Src/list.d \
./Core/Src/m95640_driver.d \
./Core/Src/main.d \
./Core/Src/max30102.d \
./Core/Src/port.d \
./Core/Src/queue.d \
./Core/Src/spi.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_hal_timebase_tim.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/stream_buffer.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/tasks.d \
./Core/Src/timers.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/croutine.d ./Core/Src/croutine.o ./Core/Src/croutine.su ./Core/Src/event_groups.d ./Core/Src/event_groups.o ./Core/Src/event_groups.su ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/hc05.d ./Core/Src/hc05.o ./Core/Src/hc05.su ./Core/Src/heap_4.d ./Core/Src/heap_4.o ./Core/Src/heap_4.su ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/list.d ./Core/Src/list.o ./Core/Src/list.su ./Core/Src/m95640_driver.d ./Core/Src/m95640_driver.o ./Core/Src/m95640_driver.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/max30102.d ./Core/Src/max30102.o ./Core/Src/max30102.su ./Core/Src/port.d ./Core/Src/port.o ./Core/Src/port.su ./Core/Src/queue.d ./Core/Src/queue.o ./Core/Src/queue.su ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_hal_timebase_tim.d ./Core/Src/stm32f4xx_hal_timebase_tim.o ./Core/Src/stm32f4xx_hal_timebase_tim.su ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/stream_buffer.d ./Core/Src/stream_buffer.o ./Core/Src/stream_buffer.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/tasks.d ./Core/Src/tasks.o ./Core/Src/tasks.su ./Core/Src/timers.d ./Core/Src/timers.o ./Core/Src/timers.su ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

