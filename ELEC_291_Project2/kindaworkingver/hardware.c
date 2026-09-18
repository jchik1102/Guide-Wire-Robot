#include "hardware.h"

volatile uint32_t msTicks = 0;

void SysTick_Handler(void)
{
    msTicks++;
}

void SysTick_Init(void)
{
    SysTick->LOAD = (F_CPU / 1000L) - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

void waitms(int len)
{
    uint32_t start = msTicks;
    while ((msTicks - start) < (uint32_t)len) {}
}

void gpio_pin_mode_output(GPIO_TypeDef *port, uint8_t pin)
{
    port->MODER = (port->MODER & ~(3u << (pin * 2u))) | (1u << (pin * 2u));
}

void gpio_write_low(GPIO_TypeDef *port, uint8_t pin)
{
    port->BRR = PINMASK(pin);
}

void gpio_write_high(GPIO_TypeDef *port, uint8_t pin)
{
    port->BSRR = PINMASK(pin);
}