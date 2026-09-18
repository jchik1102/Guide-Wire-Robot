//  Author: Krish Vashist & Irene Lam
//  Date: 2026-03-25
//  Description: Integrated ROBOT code with IR Control, Hybrid Triangulation, and Multi-Clap Commands

#include <stdio.h>
#include <stdlib.h>
#include "../Common/Include/stm32l051xx.h"

// -- PIN MAPPING --
// PA0, PA1, PA6 -> Audio Mics (ADC Channels 0, 1, 6)
// PA11-PA14     -> MOSFET Motor Drivers 
// PA15          -> IR Receiver 

#define LM1 BIT7
#define LM2 BIT8
#define RM1 BIT13
#define RM2 BIT14
#define CLAP_THRESHOLD 1400 
#define SYSCLK 32000000L

// Global Variables
volatile uint16_t StartTime = 0, MeasuredWidth = 0;
volatile int STATE = 0, BitCount = 0, command = 0, NewData_flag = 0;
volatile uint32_t msTicks = 0;

void SysTick_Handler(void) { msTicks++; }

// --- ADC FUNCTIONS FOR AUDIO ---
void ADC_Init(void) {
    RCC->APB2ENR |= BIT9; // Enable ADC clock

    // Set PA0, PA1, PA6 to ANALOG mode
    GPIOA->MODER |= (BIT0 | BIT1 | BIT2 | BIT3 | BIT12 | BIT13);

    // Quick Calibration
    ADC1->CR |= BIT31;
    while ((ADC1->CR & BIT31) != 0);

    ADC1->CR |= BIT0; // Enable ADC
    while (!(ADC1->ISR & BIT0)); // Wait for ready
}

uint32_t Read_ADC(uint32_t channel) {
    ADC1->CHSELR = (1 << channel);
    ADC1->CR |= BIT2; // Start conversion
    while (!(ADC1->ISR & BIT2)); // Wait for EOC
    return ADC1->DR;
}

void Hardware_Init(void) {
    RCC->IOPENR |= BIT0;   // GPIOA Clock
    RCC->APB1ENR |= BIT0;  // Timer 2 Clock

    // Configure PA11-PA14 as Motor Outputs
    GPIOA->MODER = (GPIOA->MODER & ~0x3CC3C3FF) | 0x14014155;
    GPIOA->ODR &= ~(LM1 | LM2 | RM1 | RM2);

    // IR Receiver Setup (PA15 AF2)
    GPIOA->MODER = (GPIOA->MODER & ~(BIT30)) | (BIT31);
    GPIOA->AFR[1] |= (BIT30 | BIT28);

    TIM2->PSC = 31;
    TIM2->ARR = 0xFFFF;
    TIM2->CCMR1 |= BIT0;
    TIM2->CCER |= (BIT1 | BIT0 | BIT3);
    TIM2->DIER |= BIT1;

    NVIC->ISER[0] |= BIT15;
    TIM2->CR1 |= BIT0;
    ADC_Init(); // Initialize Microphones
    __enable_irq();
}

// IR Decoding Logic
void TIM2_Handler(void) {
    if (TIM2->SR & BIT1) {
        uint16_t currentCapture = TIM2->CCR1;
        if (!(GPIOA->IDR & BIT15)) { StartTime = currentCapture; }
        else {
            MeasuredWidth = currentCapture - StartTime;
            if (MeasuredWidth > 300 && MeasuredWidth < 600) { STATE = 1; BitCount = 0; command = 0; }
            else if (STATE == 1) {
                if (MeasuredWidth > 100 && MeasuredWidth < 350) { command = (command << 1); BitCount++; }
                else if (MeasuredWidth > 650 && MeasuredWidth < 1100) { command = (command << 1) | 1; BitCount++; }
                else { STATE = 0; }
                if (BitCount == 4) { NewData_flag = 1; STATE = 0; }
            }
        }
        TIM2->SR &= ~BIT1;
    }
}

int main(void) {
    Hardware_Init();
    SysTick->LOAD = (SYSCLK / 1000) - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = 7;

    uint32_t last_cmd_time = 0;

    // --- Audio Variables ---
    int clap_count = 0;
    uint32_t first_clap_time = 0;
    uint32_t last_clap_time = 0;
    uint32_t saved_left = 0, saved_right = 0, saved_back = 0; // Added for Triangulation snapshot

    while (1) {
        // 1. Check for IR Commands
        if (NewData_flag == 1) {
            last_cmd_time = msTicks;
            GPIOA->ODR &= ~(LM1 | LM2 | RM1 | RM2);
            switch (command) {
            case 0b0010: GPIOA->ODR |= (LM1 | RM1); break; // FORWARD
            case 0b0011: GPIOA->ODR |= (LM2 | RM1); break; // LEFT
            case 0b0100: GPIOA->ODR |= (LM1 | RM2); break; // RIGHT
            case 0b0000: GPIOA->ODR &= ~(LM1 | LM2 | RM1 | RM2); break; // STOP
            }
            NewData_flag = 0;
            clap_count = 0; // Reset clap count if IR remote is used
        }

        // 2. Audio Intelligence: Hybrid Triangulation & Counting
        if (!(GPIOA->ODR & (LM1 | LM2 | RM1 | RM2))) {
          // Replace the old left_mic, right_mic, back_mic reads with this:
			uint32_t left_raw = Read_ADC(0);
			uint32_t right_raw = Read_ADC(1);
			uint32_t back_raw = Read_ADC(6);

			// Subtract the 1.65V baseline (approx 2048) to get absolute volume
			int left_mic = abs((int)left_raw - 2048);
			int right_mic = abs((int)right_raw - 2048);
			int back_mic = abs((int)back_raw - 2048);

            // Did ANY microphone hear a clap?
            int heard_clap = (left_mic > CLAP_THRESHOLD || right_mic > CLAP_THRESHOLD || back_mic > CLAP_THRESHOLD);

            // If we heard a clap AND 200ms have passed since the last one (Debounce to ignore echoes)
            if (heard_clap && (msTicks - last_clap_time > 200)) {
                clap_count++;
                last_clap_time = msTicks;

                // If this is the FIRST clap, start the 1.5-second listening window AND save the volume
                if (clap_count == 1) {
    				first_clap_time = msTicks;
    				saved_left = left_mic; // Save the one that triggered it
    
    // ADD THIS: Micro-delay to let the sound wave reach the other mics
    				for (volatile int i = 0; i < 2500; i++); 

    // Now capture the actual peak volumes for the others
    				saved_right = abs((int)Read_ADC(1)-2048); 
    				saved_back = abs((int)Read_ADC(6)-2048);
				}
            }

            // If we have started counting claps AND our 1.5-second listening window has expired
            if (clap_count > 0 && (msTicks - first_clap_time > 1500)) {

                // Evaluate how many claps we heard
                if (clap_count == 1) {
                    // 1 Clap: Use the saved volumes to Triangulate Direction!
                    if (saved_left > saved_right && saved_left > saved_back) {
                        GPIOA->ODR |= (LM2 | RM1); // Turn Left
                    }
                    else if (saved_right > saved_left && saved_right > saved_back) {
                        GPIOA->ODR |= (LM1 | RM2); // Turn Right
                    }
                    else {
                        GPIOA->ODR |= (LM1 | RM2); // Turn Around (Right Spin)
                    }
                }
                else if (clap_count == 2) {
                    GPIOA->ODR |= (LM1 | RM2); // 2 Claps = RIGHT
                }
                else if (clap_count == 3) {
                    GPIOA->ODR |= (LM2 | RM1); // 3 Claps = LEFT
                }
                else if (clap_count == 4) {
                    GPIOA->ODR |= (LM1 | RM1); // 4+ Claps = FORWARD
                }

                // Set the command timer so it auto-stops later
                last_cmd_time = msTicks;

                // Reset the system to listen for the next round of claps
                clap_count = 0;
            }
        }

        // 3. Auto-Stop Timeout (250ms burst)
        if (last_cmd_time != 0 && (msTicks - last_cmd_time > 1000)) {
            GPIOA->ODR &= ~(LM1 | LM2 | RM1 | RM2);
            last_cmd_time = 0;
            // --- THE FIX: FORCED SILENCE ---
    // Set last_clap_time to right now so the 450ms debounce 
    // prevents the robot from hearing its own motor/relay click.
    last_clap_time = msTicks; 
    clap_count = 0; // Double-ensure count is clean
        }
    }
}
