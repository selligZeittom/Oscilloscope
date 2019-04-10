#include "../Drivers/STM32F7xx_HAL_Driver/Inc/stm32f7xx_hal.h"

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	volatile uint32_t value = HAL_ADC_GetValue(hadc);
}
