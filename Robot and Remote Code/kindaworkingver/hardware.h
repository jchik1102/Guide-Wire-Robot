#ifndef HARDWARE_H
#define HARDWARE_H

#include <stdint.h>
#include <stdbool.h>
#include "../Common/Include/stm32l051xx.h"

#define F_CPU 32000000L

#define PINMASK(pin) (1u << (pin))

extern volatile uint32_t msTicks;

void SysTick_Init(void);
void waitms(int len);
void gpio_pin_mode_output(GPIO_TypeDef *port, uint8_t pin);
void gpio_write_low(GPIO_TypeDef *port, uint8_t pin);
void gpio_write_high(GPIO_TypeDef *port, uint8_t pin);

#endif