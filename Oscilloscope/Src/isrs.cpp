#include "../Drivers/STM32F7xx_HAL_Driver/Inc/stm32f7xx_hal.h"
#include "app/factory.h"

extern "C" {
	uint16_t adcValuesBuffer[50];
	uint32_t indexArray = 0;
}

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9);
	volatile uint32_t value = HAL_ADC_GetValue(hadc);
	adcValuesBuffer[indexArray++] = value; //store read value inside array
}

extern "C" void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef* htim) {

}
