################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
C:/Users/Gilles\ Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/src/platform/f7-disco-gcc/mcu/hal/uart.cpp 

OBJS += \
./platform/f7-disco-gcc/mcu/hal/uart.o 

CPP_DEPS += \
./platform/f7-disco-gcc/mcu/hal/uart.d 


# Each subdirectory must supply rules for building sources it contributes
platform/f7-disco-gcc/mcu/hal/uart.o: C:/Users/Gilles\ Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/src/platform/f7-disco-gcc/mcu/hal/uart.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: MCU G++ Compiler'
	@echo $(PWD)
	arm-none-eabi-g++ -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F746xx -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Inc" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/config" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/app" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw/ugfx" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw/ugfx/boards/base/STM32F746-Discovery" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw/ugfx/drivers/gdisp/STM32LTDC" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw/ugfx/src/gdisp/mcufont" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/ui-gen" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/platform/f7-disco-gcc/mcu" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/platform/f7-disco-gcc" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/xf/port" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/xf/include" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/config" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Drivers/STM32F7xx_HAL_Driver/Inc" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Drivers/CMSIS/Include" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fno-exceptions -fno-rtti -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


