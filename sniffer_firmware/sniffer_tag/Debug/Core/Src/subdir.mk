################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Application.c \
../Core/Src/config_options.c \
../Core/Src/deca_device.c \
../Core/Src/ds_twr_initiator.c \
../Core/Src/ds_twr_responder.c \
../Core/Src/hmi_uart.c \
../Core/Src/key.c \
../Core/Src/main.c \
../Core/Src/menu.c \
../Core/Src/shared_functions.c \
../Core/Src/simple_rx.c \
../Core/Src/simple_tx.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c \
../Core/Src/uwb.c 

OBJS += \
./Core/Src/Application.o \
./Core/Src/config_options.o \
./Core/Src/deca_device.o \
./Core/Src/ds_twr_initiator.o \
./Core/Src/ds_twr_responder.o \
./Core/Src/hmi_uart.o \
./Core/Src/key.o \
./Core/Src/main.o \
./Core/Src/menu.o \
./Core/Src/shared_functions.o \
./Core/Src/simple_rx.o \
./Core/Src/simple_tx.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o \
./Core/Src/uwb.o 

C_DEPS += \
./Core/Src/Application.d \
./Core/Src/config_options.d \
./Core/Src/deca_device.d \
./Core/Src/ds_twr_initiator.d \
./Core/Src/ds_twr_responder.d \
./Core/Src/hmi_uart.d \
./Core/Src/key.d \
./Core/Src/main.d \
./Core/Src/menu.d \
./Core/Src/shared_functions.d \
./Core/Src/simple_rx.d \
./Core/Src/simple_tx.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d \
./Core/Src/uwb.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Application.cyclo ./Core/Src/Application.d ./Core/Src/Application.o ./Core/Src/Application.su ./Core/Src/config_options.cyclo ./Core/Src/config_options.d ./Core/Src/config_options.o ./Core/Src/config_options.su ./Core/Src/deca_device.cyclo ./Core/Src/deca_device.d ./Core/Src/deca_device.o ./Core/Src/deca_device.su ./Core/Src/ds_twr_initiator.cyclo ./Core/Src/ds_twr_initiator.d ./Core/Src/ds_twr_initiator.o ./Core/Src/ds_twr_initiator.su ./Core/Src/ds_twr_responder.cyclo ./Core/Src/ds_twr_responder.d ./Core/Src/ds_twr_responder.o ./Core/Src/ds_twr_responder.su ./Core/Src/hmi_uart.cyclo ./Core/Src/hmi_uart.d ./Core/Src/hmi_uart.o ./Core/Src/hmi_uart.su ./Core/Src/key.cyclo ./Core/Src/key.d ./Core/Src/key.o ./Core/Src/key.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/menu.cyclo ./Core/Src/menu.d ./Core/Src/menu.o ./Core/Src/menu.su ./Core/Src/shared_functions.cyclo ./Core/Src/shared_functions.d ./Core/Src/shared_functions.o ./Core/Src/shared_functions.su ./Core/Src/simple_rx.cyclo ./Core/Src/simple_rx.d ./Core/Src/simple_rx.o ./Core/Src/simple_rx.su ./Core/Src/simple_tx.cyclo ./Core/Src/simple_tx.d ./Core/Src/simple_tx.o ./Core/Src/simple_tx.su ./Core/Src/stm32f1xx_hal_msp.cyclo ./Core/Src/stm32f1xx_hal_msp.d ./Core/Src/stm32f1xx_hal_msp.o ./Core/Src/stm32f1xx_hal_msp.su ./Core/Src/stm32f1xx_it.cyclo ./Core/Src/stm32f1xx_it.d ./Core/Src/stm32f1xx_it.o ./Core/Src/stm32f1xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f1xx.cyclo ./Core/Src/system_stm32f1xx.d ./Core/Src/system_stm32f1xx.o ./Core/Src/system_stm32f1xx.su ./Core/Src/uwb.cyclo ./Core/Src/uwb.d ./Core/Src/uwb.o ./Core/Src/uwb.su

.PHONY: clean-Core-2f-Src

