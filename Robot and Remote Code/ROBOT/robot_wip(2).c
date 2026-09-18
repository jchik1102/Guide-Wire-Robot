//  Author: Krish Vashist
//  Date: 2026-03-20
//  Description: Unified ROBOT code integrating manual IR remote control 
//               and automatic path-following modes

#include <stdio.h>
#include <stdlib.h>
#include "../Common/Include/stm32l051xx.h"


// ----- MOTOR PIN DEFINITIONS -----
#define LM1 BIT11 // PA11 -> left motor forward
#define LM2 BIT12 // PA12 -> left motor backward
#define RM1 BIT13 // PA13 -> right motor forward
#define RM2 BIT14 // PA14 -> right motor backward


#define SYSCLK 32000000L
#define TICK_FREQ 1000L

// --- IR RECEIVER STATE ---
volatile uint16_t StartTime = 0; 
volatile uint16_t MeasuredWidth = 0;
volatile int STATE = 0;        // 0->IDLE, 1->RECEIVING BITS
volatile int BitCount = 0; 
volatile int command = 0; 
volatile int NewData_flag = 0; 


// -- Dual mode state variables ----
volatile int auto_mode = 0; // 0 -> Manual, 1 -> Auto
volatile int active_path = 0; // 0 -> path 1, 1 -> path 2, 2 -> path 3
volatile int intersection_count = 0; // tracks progress along the path (make sure to roll over at the end of the path)

// -- path definitions ---
#define PATH_FORWARD 0
#define PATH_LEFT 1
#define PATH_RIGHT 2
#define PATH_STOP 3

const int paths[3][8] = {
    {PATH_FORWARD, PATH_LEFT, PATH_LEFT, PATH_FORWARD, PATH_RIGHT, PATH_LEFT, PATH_RIGHT, PATH_STOP},    // Path 1
    {PATH_LEFT, PATH_RIGHT, PATH_LEFT, PATH_RIGHT, PATH_FORWARD, PATH_FORWARD, PATH_STOP, PATH_STOP},    // Path 2
    {PATH_RIGHT, PATH_FORWARD, PATH_RIGHT, PATH_LEFT, PATH_RIGHT, PATH_LEFT, PATH_FORWARD, PATH_STOP}    // Path 3
};


// -- helper functions ----

void wait(int ms) {
    for (volatile int i = 0; i < ms * (SYSCLK / TICK_FREQ) / 5; i++);
}

void move_forward() { // both motors move forward
    GPIOA->ODR |= (LM1 | RM1); 
    GPIOA->ODR |= BIT4;  // LED 4 -> forward
}

void turn_left() { // left motor backward and right motor forward 
    GPIOA->ODR |= (LM2 | RM1); 
}

void turn_right() { // left motor forward and right motor backward
    GPIOA->ODR |= (LM1 | RM2);  
}

void stop_motors() { // stop all motors
    GPIOA->ODR &= ~(LM1 | LM2 | RM1 | RM2); 
    GPIOA->ODR &= ~BIT4; // ensure LED is off just in case
}

// --- Intersection Detection ---
// NOTE THAT THIS IS JUST A PLACEHOLDER AND NEED TO BE REPLACED FROM GUIDEWIRE STUFF
int intersection_detected() {
    return 0;
}

// --- Path Following ---
// NOTE THAT THIS IS JUST A PLACEHOLDER AND NEED TO BE REPLACED FROM GUIDEWIRE STUFF
void follow_path() {
    move_forward();
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

    while(1) {    
        // 1. First, check if there is a new IR command exactly as requested
        if (NewData_flag == 1) {
            
            // Mode Toggle overrides everything
            if (command == 0b0101) {
                auto_mode = !auto_mode;
                
                // As requested, when entering manual mode, STOP the car immediately 
                // until the user moves the joystick.
                stop_motors(); 
            }
            
            // Path Selections (Activates Auto Mode)
            else if (command == 0b0110) { active_path = 0; intersection_count = 0; auto_mode = 1; }
            else if (command == 0b0111) { active_path = 1; intersection_count = 0; auto_mode = 1; }
            else if (command == 0b1000) { active_path = 2; intersection_count = 0; auto_mode = 1; }
            
            // Manual Movements (Only process these if already in Manual Mode)
            else if (auto_mode == 0) {
                if (command == 0b0000) { stop_motors(); }
                else if (command == 0b0010) { stop_motors(); move_forward(); }
                else if (command == 0b0011) { stop_motors(); turn_left(); }
                else if (command == 0b0100) { stop_motors(); turn_right(); }
            }
            
            NewData_flag = 0; 
        }

        // 2. State-Based Execution Flow
        if (auto_mode == 0) {
            // ----- MANUAL MODE -----
            // In manual mode, we simply let the motors continue whatever the last IR command
            // told them to do. Since you stop on manual transition, it stays stopped until
            // the joystick transmits FORWARD/LEFT/RIGHT.
        }
        else {
            // ----- AUTOMATIC MODE -----
            follow_path(); 

            // Intersection Logic
            if (intersection_detected()) {
                if (intersection_count < 8) {
                    int action = paths[active_path][intersection_count];

                    if (action == PATH_FORWARD) { move_forward(); }
                    else if (action == PATH_LEFT) { turn_left(); }
                    else if (action == PATH_RIGHT) { turn_right(); }
                    else if (action == PATH_STOP) {
                        stop_motors();
                        auto_mode = 0; // Finished path
                    }

                    intersection_count++;

                    // Wait until we physically clear the intersection
                    // to prevent incrementing the count multiple times
                    while (intersection_detected()) {
                        follow_path(); 
                    }
                } else {
                    stop_motors();
                    auto_mode = 0; 
                }
            }
        }
    }
}