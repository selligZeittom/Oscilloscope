################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Gilles\ Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/src/mdw/ugfx/src/gmisc/gmisc.c \
C:/Users/Gilles\ Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/src/mdw/ugfx/src/gmisc/gmisc_arrayops.c \
C:/Users/Gilles\ Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/src/mdw/ugfx/src/gmisc/gmisc_hittest.c \
C:/Users/Gilles\ Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/src/mdw/ugfx/src/gmisc/gmisc_matrix2d.c \
C:/Users/Gilles\ Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/src/mdw/ugfx/src/gmisc/gmisc_trig.c 

OBJS += \
./mdw/ugfx/src/gmisc/gmisc.o \
./mdw/ugfx/src/gmisc/gmisc_arrayops.o \
./mdw/ugfx/src/gmisc/gmisc_hittest.o \
./mdw/ugfx/src/gmisc/gmisc_matrix2d.o \
./mdw/ugfx/src/gmisc/gmisc_trig.o 

C_DEPS += \
./mdw/ugfx/src/gmisc/gmisc.d \
./mdw/ugfx/src/gmisc/gmisc_arrayops.d \
./mdw/ugfx/src/gmisc/gmisc_hittest.d \
./mdw/ugfx/src/gmisc/gmisc_matrix2d.d \
./mdw/ugfx/src/gmisc/gmisc_trig.d 


# Each subdirectory must supply rules for building sources it contributes
mdw/ugfx/src/gmisc/gmisc.o: C:/Users/Gilles\ Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/src/mdw/ugfx/src/gmisc/gmisc.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F746xx -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Inc" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw/ugfx" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw/ugfx/boards/base/STM32F746-Discovery" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw/ugfx/drivers/gdisp/STM32LTDC" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw/ugfx/src/gdisp/mcufont" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/ui-gen" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/platform/f7-disco-gcc/mcu" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/platform/f7-disco-gcc" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/xf/port" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/xf/include" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/config" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Drivers/STM32F7xx_HAL_Driver/Inc" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

mdw/ugfx/src/gmisc/gmisc_arrayops.o: C:/Users/Gilles\ Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/src/mdw/ugfx/src/gmisc/gmisc_arrayops.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F746xx -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Inc" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw/ugfx" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw/ugfx/boards/base/STM32F746-Discovery" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw/ugfx/drivers/gdisp/STM32LTDC" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw/ugfx/src/gdisp/mcufont" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/ui-gen" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/platform/f7-disco-gcc/mcu" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/platform/f7-disco-gcc" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/xf/port" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/xf/include" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/config" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Drivers/STM32F7xx_HAL_Driver/Inc" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

mdw/ugfx/src/gmisc/gmisc_hittest.o: C:/Users/Gilles\ Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/src/mdw/ugfx/src/gmisc/gmisc_hittest.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F746xx -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Inc" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw/ugfx" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw/ugfx/boards/base/STM32F746-Discovery" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw/ugfx/drivers/gdisp/STM32LTDC" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw/ugfx/src/gdisp/mcufont" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/ui-gen" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/platform/f7-disco-gcc/mcu" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/platform/f7-disco-gcc" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/xf/port" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/xf/include" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/config" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Drivers/STM32F7xx_HAL_Driver/Inc" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

mdw/ugfx/src/gmisc/gmisc_matrix2d.o: C:/Users/Gilles\ Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/src/mdw/ugfx/src/gmisc/gmisc_matrix2d.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F746xx -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Inc" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw/ugfx" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw/ugfx/boards/base/STM32F746-Discovery" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw/ugfx/drivers/gdisp/STM32LTDC" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw/ugfx/src/gdisp/mcufont" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/ui-gen" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/platform/f7-disco-gcc/mcu" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/platform/f7-disco-gcc" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/xf/port" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/xf/include" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/config" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Drivers/STM32F7xx_HAL_Driver/Inc" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

mdw/ugfx/src/gmisc/gmisc_trig.o: C:/Users/Gilles\ Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/src/mdw/ugfx/src/gmisc/gmisc_trig.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F746xx -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Inc" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw/ugfx" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw/ugfx/boards/base/STM32F746-Discovery" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw/ugfx/drivers/gdisp/STM32LTDC" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw/ugfx/src/gdisp/mcufont" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/ui-gen" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/platform/f7-disco-gcc/mcu" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/platform/f7-disco-gcc" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/mdw" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/xf/port" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/xf/include" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src/config" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/../src" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Drivers/STM32F7xx_HAL_Driver/Inc" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"C:/Users/Gilles Mottiez/Documents/HES/I5_6/PTR/RealTimeEmbedded/Oscilloscope/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


