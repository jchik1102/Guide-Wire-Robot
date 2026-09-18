//  Author: Krish Vashist
//  Date: 2026-03-16
//  Description: This file contains the fully TESTED IR receiver code


#include <stdio.h>
#include <stdlib.h>
#include "../Common/Include/stm32l051xx.h"

volatile uint32_t StartTime = 0; 
volatile uint32_t MeasuredWidth = 0; 
volatile int STATE = 0;        // 0-> IDLE, 1-> RECEIVING BITS
volatile int BitCount = 0; 
volatile int command = 0; 
volatile int NewData_flag = 0; 

#define SYSCLK 32000000L
#define TICK_FREQ 1000L

void wait(int ms) {
    for (volatile int i = 0; i < ms * (SYSCLK / TICK_FREQ) / 5; i++);
}

void Hardware_Init(void) {
    RCC->IOPENR |= BIT0;   // GPIOA Clock
    RCC->APB1ENR |= BIT0;  // Timer 2 Clock

    // Configure PA1-PA4 as Outputs for LEDs
    GPIOA->MODER = (GPIOA->MODER & ~(0x3FF)) | 0x155;

    // Set up PA15 for Timer 2 Input Capture (AF2)
    GPIOA->MODER = (GPIOA->MODER & ~(BIT30)) | (BIT31); 
    GPIOA->AFR[1] |= (BIT30 | BIT28); 

    // Timer speed: 1 tick = 1 us
    TIM2->PSC = 31; 
    TIM2->ARR = 0xFFFF; 

    // Input Capture on Channel 1 (Both Edges)
    TIM2->CCMR1 |= BIT0; 
    TIM2->CCER |= (BIT1 | BIT0 | BIT3); 
    TIM2->DIER |= BIT1; 

    NVIC->ISER[0] |= BIT15; 
    TIM2->CR1 |= BIT0; 
    __enable_irq();
}

void TIM2_Handler(void) {
    if (TIM2->SR & BIT1) { 
        uint32_t currentCapture = TIM2->CCR1; 

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

    // BIT1 -> red -> pin 7 -> STOP -> 0000
    // BIT2 -> yellow -> pin 8 -> left -> 0001
    // BIT3 -> White -> pin 9 -> right -> 1000
    // BIT4 -> Green -> pin 10 -> forward -> 1001 


int main(void) {
    Hardware_Init();

    while(1) {    
        if (NewData_flag == 1) {
            switch (command) {
                case 0b1001: // FORWARD
                    GPIOA->ODR |= BIT4; 
                    wait(500); 
                    GPIOA->ODR &= ~BIT4; 
                    break;
                case 0b0001: // LEFT
                    GPIOA->ODR |= BIT2; 
                    wait(500); 
                    GPIOA->ODR &= ~BIT2; 
                    break;
                case 0b1000: // RIGHT
                    GPIOA->ODR |= BIT3; 
                    wait(500); 
                    GPIOA->ODR &= ~BIT3; 
                    break;
                case 0b0000: // STOP
                    GPIOA->ODR |= BIT1; 
                    wait(500); 
                    GPIOA->ODR &= ~BIT1; 
                    break;
            }
            NewData_flag = 0; 
        }
    }
}

