#include "hardware.h"

static void wait_1ms(void)
{
    SysTick->LOAD = (F_CPU / 1000L) - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    while ((SysTick->CTRL & (1u << 16)) == 0)
    {
    }
    SysTick->CTRL = 0;
}

void waitms(int len)
{
    while (len-- > 0)
    {
        wait_1ms();
    }
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