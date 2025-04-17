################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ThirdParty/SEGGER/OS/SEGGER_SYSVIEW_FreeRTOS.c 

C_DEPS += \
./ThirdParty/SEGGER/OS/SEGGER_SYSVIEW_FreeRTOS.d 

OBJS += \
./ThirdParty/SEGGER/OS/SEGGER_SYSVIEW_FreeRTOS.o 


# Each subdirectory must supply rules for building sources it contributes
ThirdParty/SEGGER/OS/%.o ThirdParty/SEGGER/OS/%.su ThirdParty/SEGGER/OS/%.cyclo: ../ThirdParty/SEGGER/OS/%.c ThirdParty/SEGGER/OS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/ejverat/LDATA/Projects/stm32/RTOS_WS/stm32_cli_led_rtc/ThirdParty/SEGGER/Config" -I"/home/ejverat/LDATA/Projects/stm32/RTOS_WS/stm32_cli_led_rtc/ThirdParty/SEGGER/OS" -I"/home/ejverat/LDATA/Projects/stm32/RTOS_WS/stm32_cli_led_rtc/ThirdParty/SEGGER/SEGGER" -I"/home/ejverat/LDATA/Projects/stm32/RTOS_WS/stm32_cli_led_rtc/ThirdParty/FreeRTOS" -I"/home/ejverat/LDATA/Projects/stm32/RTOS_WS/stm32_cli_led_rtc/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"/home/ejverat/LDATA/Projects/stm32/RTOS_WS/stm32_cli_led_rtc/ThirdParty/FreeRTOS/include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ThirdParty-2f-SEGGER-2f-OS

clean-ThirdParty-2f-SEGGER-2f-OS:
	-$(RM) ./ThirdParty/SEGGER/OS/SEGGER_SYSVIEW_FreeRTOS.cyclo ./ThirdParty/SEGGER/OS/SEGGER_SYSVIEW_FreeRTOS.d ./ThirdParty/SEGGER/OS/SEGGER_SYSVIEW_FreeRTOS.o ./ThirdParty/SEGGER/OS/SEGGER_SYSVIEW_FreeRTOS.su

.PHONY: clean-ThirdParty-2f-SEGGER-2f-OS

