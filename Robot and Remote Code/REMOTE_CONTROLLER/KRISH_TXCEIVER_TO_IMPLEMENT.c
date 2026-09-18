//  Author: Krish Vashist
//  Date: 2026-03-16
//  Description: This file contains the working in progress code for the entire ROBOT setup

#include <stdio.h>
#include <stdlib.h>
#include "../Common/Include/stm32l051xx.h"



// PIN MAPPING (configure these to MODER register)
// PA11 -> LM1 (Pin 21) - left motor forward
// PA12 -> LM2 (Pin 22) - left motor backward
// PA13 -> RM1 (Pin 23) - right motor forward
// PA14 -> RM2 (Pin 24) - right motor backward
// PA15 -> IR receiver input (Pin 25)


// -- MOTOR PIN DEFINITIONS --
#define LM1 BIT11
#define LM2 BIT12
#define RM1 BIT13
#define RM2 BIT14


#define SYSCLK 32000000L
#define TICK_FREQ 1000L


volatile uint16_t StartTime = 0; 
volatile uint16_t MeasuredWidth = 0;
volatile int STATE = 0;        // 0-> IDLE, 1-> RECEIVING BITS
volatile int BitCount = 0; 
volatile int command = 0; 
volatile int NewData_flag = 0; 

// Track time in milliseconds using the SysTick timer
volatile uint32_t msTicks = 0;
void SysTick_Handler(void) {
    msTicks++;
}

void wait(int ms) {
    for (volatile int i = 0; i < ms * (SYSCLK / TICK_FREQ) / 5; i++);
}

void Hardware_Init(void) {
    RCC->IOPENR |= BIT0;   // GPIOA Clock
    RCC->APB1ENR |= BIT0;  // Timer 2 Clock
    // Configure PA1-PA4 as LEDs (0x155) 
    // AND PA11-PA14 as Motor Outputs (0x5500000)

    GPIOA->MODER = (GPIOA->MODER & ~0x3FF003FF) | 0x15500155;


    GPIOA->ODR &= ~(LM1 | LM2 | RM1 | RM2); // clear all motor bits to stop the car -> intially the car must be in stopped state


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

int main(void) {
    Hardware_Init();

    // Configure SysTick to generate an interrupt every 1 ms
    // This allows us to keep track of time without blocking the processor.
    SysTick->LOAD = (SYSCLK / 1000) - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = 7; // CLKSOURCE=CPU clock, TICKINT=enable, ENABLE=enable

    uint32_t last_cmd_time = 0;

    while(1) {    
        if (NewData_flag == 1) {
            last_cmd_time = msTicks; // Record the timestamp of the latest command

            // stop all the motors before giving a new command
            GPIOA->ODR &= ~(LM1 | LM2 | RM1 | RM2); 
            wait (5);
            switch (command) {
                case 0b0010: // FORWARD
                    // LM1 -> left motor forward
                    // RM1 -> right motor forward
                    // so both motors move forward
                    // this causes the car to move forward
                    GPIOA->ODR |= (LM1 | RM1); 
                    GPIOA->ODR |= BIT4;  // LED 4 -> forward
                    // wait(1000);  (wanna avoid any delays)
                    break;
                case 0b0011: // LEFT
                    // LM2 -> left motor backward
                    // RM1 -> right motor forward
                    // so the left motor moves backward and the right motor moves forward
                    // this causes the car to turn left
                    GPIOA->ODR |= (LM2 | RM1); 
                    //wait(1000); 
                    // GPIOA->ODR &= ~(LM2 | RM1);  TURN IT OFF AFTER 1 SECOND
                    break;
                case 0b0100: // RIGHT
                    GPIOA->ODR |= (LM1 | RM2);  // what does this line do again? Ans: 
                    // LM1 -> left motor forward
                    // RM2 -> right motor backward
                    // so the left motor moves forward and the right motor moves backward
                    // this causes the car to turn right
                    //wait(1000); 
                    //GPIOA->ODR &= ~(LM1 | RM2);  TURN IT OFF AFTER 1 SECOND
                    break;
                case 0b0000: // STOP
                    //wait(1000);
                    GPIOA->ODR |= BIT4;  // LED 4 -> forward
                    GPIOA->ODR &= ~(BIT4);
                    break; 
            }
            NewData_flag = 0; 
        }

        if (last_cmd_time != 0 && (msTicks - last_cmd_time > 250)) {
            GPIOA->ODR &= ~(LM1 | LM2 | RM1 | RM2); // Turn off all motors
            GPIOA->ODR &= ~(BIT4);  // Turn off LEDs
            last_cmd_time = 0; // Reset so we don't keep clearing outputs
        }
    }
}
