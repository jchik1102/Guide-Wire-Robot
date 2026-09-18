#include "ir_motor.h"

// ---- IR receiver state (shared with ISR) ----
static volatile uint16_t StartTime = 0;
static volatile uint16_t MeasuredWidth = 0;
static volatile int STATE = 0;        // 0 = IDLE, 1 = RECEIVING BITS
static volatile int BitCount = 0;
static volatile int command = 0;
static volatile int NewData_flag = 0;


// =============================================================================
//  Init: PA4 LED, PA11-14 motors, PA15 TIM2 CH1 input capture
// =============================================================================

void IR_Motor_Init(void)
{
    RCC->IOPENR |= BIT0;   // GPIOA clock
    RCC->APB1ENR |= BIT0;  // TIM2 clock

    // PA4 as output (IR status LED)
    gpio_pin_mode_output(GPIOA, 4u);
    GPIOA->ODR &= ~IR_LED;

    // PA11-PA14 as outputs (motor pins)
    gpio_pin_mode_output(GPIOA, 11u);
    gpio_pin_mode_output(GPIOA, 12u);
    gpio_pin_mode_output(GPIOA, 13u);
    gpio_pin_mode_output(GPIOA, 14u);
    GPIOA->ODR &= ~(LM1 | LM2 | RM1 | RM2);

    // PA15 as AF2 for TIM2_CH1 (IR receiver input)
    GPIOA->MODER = (GPIOA->MODER & ~(BIT30)) | (BIT31);
    GPIOA->AFR[1] |= (BIT30 | BIT28);

    // TIM2: 1 us per tick, input capture on CH1, both edges
    TIM2->PSC = 31;
    TIM2->ARR = 0xFFFF;
    TIM2->CCMR1 |= BIT0;
    TIM2->CCER |= (BIT1 | BIT0 | BIT3);
    TIM2->DIER |= BIT1;

    NVIC->ISER[0] |= BIT15;
    TIM2->CR1 |= BIT0;
    __enable_irq();
}


// =============================================================================
//  TIM2 ISR -- IR pulse decoding
// =============================================================================

void TIM2_Handler(void)
{
    if (TIM2->SR & BIT1) {
        uint16_t currentCapture = TIM2->CCR1;

        if (!(GPIOA->IDR & BIT15)) {
            StartTime = currentCapture;
        }
        else {
            MeasuredWidth = currentCapture - StartTime;

            if (MeasuredWidth > 300 && MeasuredWidth < 600) {
                STATE = 1;
                BitCount = 0;
                command = 0;
            }
            else if (STATE == 1) {
                if (MeasuredWidth > 100 && MeasuredWidth < 350) {
                    command = (command << 1);
                    BitCount++;
                }
                else if (MeasuredWidth > 650 && MeasuredWidth < 1100) {
                    command = (command << 1) | 1;
                    BitCount++;
                }
                else {
                    STATE = 0;
                }

                if (BitCount == 4) {
                    NewData_flag = 1;
                    STATE = 0;
                }
            }
        }
        TIM2->SR &= ~BIT1;
    }
}


// =============================================================================
//  Process latest IR command (call from main loop)
// =============================================================================

void process_ir_command(void)
{
    if (!NewData_flag)
        return;

    GPIOA->ODR &= ~(LM1 | LM2 | RM1 | RM2);
    GPIOA->ODR &= ~IR_LED;
    waitms(5);

    switch (command) {
        case IR_CMD_FORWARD:
            GPIOA->ODR |= (LM1 | RM1);
            GPIOA->ODR |= IR_LED;
            break;
        case IR_CMD_LEFT:
            GPIOA->ODR |= (LM2 | RM1);
            break;
        case IR_CMD_RIGHT:
            GPIOA->ODR |= (LM1 | RM2);
            break;
        case IR_CMD_STOP:
            GPIOA->ODR |= IR_LED;
            GPIOA->ODR &= ~IR_LED;
            break;
    }

    NewData_flag = 0;
}