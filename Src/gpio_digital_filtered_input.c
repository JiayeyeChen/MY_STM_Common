#include "gpio_digital_filtered_input.h"


uint8_t GPIO_Digital_Filtered_Input(GPIOStruct* hgpio, uint32_t debounce_time)
{
	// read the state of the switch into a local variable (HIGH, when the button is not pressed)
	hgpio->curRead = HAL_GPIO_ReadPin(hgpio->gpioPort, hgpio->gpioPin);
	//check if the button was pressed
	if (hgpio->curRead != hgpio->preRead)
		// reset the debouncing timer
		hgpio->lastDebounceTime = HAL_GetTick();
	// whatever the reading is at, it's been there for longer than the debounce delay, so the current value is safe
	if (((HAL_GetTick() - hgpio->lastDebounceTime) > debounce_time) && (hgpio->curRead != hgpio->state))
	{
		hgpio->state = hgpio->curRead;
		
		if (hgpio->state == GPIO_PIN_RESET) //Effective only when Pin is RESET
		{
			return 1;
		}
	}
	// Update the last button read
	hgpio->preRead = hgpio->curRead;
	return 0;
}
