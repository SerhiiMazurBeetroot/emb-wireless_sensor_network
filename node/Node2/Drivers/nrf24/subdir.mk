################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/nrf24/nrf24.c 

OBJS += \
./Drivers/nrf24/nrf24.o 

C_DEPS += \
./Drivers/nrf24/nrf24.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/nrf24/%.o Drivers/nrf24/%.su Drivers/nrf24/%.cyclo: ../Drivers/nrf24/%.c Drivers/nrf24/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DPKT_NODE_ID=0x02 -DNODE_USE_AHT20=1 -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/node/Drivers/nrf24" -I../Drivers/bme280 -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/nrf24 -I../Drivers/aht20 -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-nrf24

clean-Drivers-2f-nrf24:
	-$(RM) ./Drivers/nrf24/nrf24.cyclo ./Drivers/nrf24/nrf24.d ./Drivers/nrf24/nrf24.o ./Drivers/nrf24/nrf24.su

.PHONY: clean-Drivers-2f-nrf24

