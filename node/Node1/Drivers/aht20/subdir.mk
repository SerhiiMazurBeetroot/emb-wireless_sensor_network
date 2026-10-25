################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/aht20/aht20.c 

OBJS += \
./Drivers/aht20/aht20.o 

C_DEPS += \
./Drivers/aht20/aht20.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/aht20/%.o Drivers/aht20/%.su Drivers/aht20/%.cyclo: ../Drivers/aht20/%.c Drivers/aht20/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DNODE_USE_AHT20=0 -DPKT_NODE_ID=0x01 -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/node/Drivers/nrf24" -I../Drivers/bme280 -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/nrf24 -I../Drivers/aht20 -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-aht20

clean-Drivers-2f-aht20:
	-$(RM) ./Drivers/aht20/aht20.cyclo ./Drivers/aht20/aht20.d ./Drivers/aht20/aht20.o ./Drivers/aht20/aht20.su

.PHONY: clean-Drivers-2f-aht20

