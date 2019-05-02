################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/adc.c \
../Src/dma2d.c \
../Src/fmc.c \
../Src/freertos.c \
../Src/gpio.c \
../Src/ltdc.c \
../Src/main.c \
../Src/stm32f7xx_hal_msp.c \
../Src/stm32f7xx_hal_timebase_TIM.c \
../Src/stm32f7xx_it.c \
../Src/system_stm32f7xx.c \
../Src/tim.c \
../Src/usart.c 

CPP_SRCS += \
../Src/isrs.cpp 

OBJS += \
./Src/adc.o \
./Src/dma2d.o \
./Src/fmc.o \
./Src/freertos.o \
./Src/gpio.o \
./Src/isrs.o \
./Src/ltdc.o \
./Src/main.o \
./Src/stm32f7xx_hal_msp.o \
./Src/stm32f7xx_hal_timebase_TIM.o \
./Src/stm32f7xx_it.o \
./Src/system_stm32f7xx.o \
./Src/tim.o \
./Src/usart.o 

C_DEPS += \
./Src/adc.d \
./Src/dma2d.d \
./Src/fmc.d \
./Src/freertos.d \
./Src/gpio.d \
./Src/ltdc.d \
./Src/main.d \
./Src/stm32f7xx_hal_msp.d \
./Src/stm32f7xx_hal_timebase_TIM.d \
./Src/stm32f7xx_it.d \
./Src/system_stm32f7xx.d \
./Src/tim.d \
./Src/usart.d 

CPP_DEPS += \
./Src/isrs.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F746xx -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Inc" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw/ugfx" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw/ugfx/boards/base/STM32F746-Discovery" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw/ugfx/drivers/gdisp/STM32LTDC" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw/ugfx/src/gdisp/mcufont" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/ui-gen" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/platform/f7-disco-gcc/mcu" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/platform/f7-disco-gcc" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/xf/port" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/xf/include" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/config" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/config" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/app" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Middlewares/Third_Party/FreeRTOS/Source" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Drivers/STM32F7xx_HAL_Driver/Inc" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Drivers/CMSIS/Include" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Src/%.o: ../Src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: MCU G++ Compiler'
	@echo $(PWD)
	arm-none-eabi-g++ -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F746xx -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Inc" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/config" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/app" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw/ugfx" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw/ugfx/boards/base/STM32F746-Discovery" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw/ugfx/drivers/gdisp/STM32LTDC" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw/ugfx/src/gdisp/mcufont" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/ui-gen" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/platform/f7-disco-gcc/mcu" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/platform/f7-disco-gcc" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/xf/port" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/xf/include" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/config" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Drivers/STM32F7xx_HAL_Driver/Inc" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Drivers/CMSIS/Include" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fno-exceptions -fno-rtti -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


