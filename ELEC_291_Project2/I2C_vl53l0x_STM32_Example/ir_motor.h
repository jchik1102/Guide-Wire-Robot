#ifndef IR_MOTOR_H
#define IR_MOTOR_H

#include "hardware.h"

// Motor pins (GPIOA)
#define LM1 BIT11
#define LM2 BIT12
#define RM1 BIT13
#define RM2 BIT14

// IR status LED
#define IR_LED BIT4  // PA4

// IR command codes
#define IR_CMD_STOP    0b0000
#define IR_CMD_FORWARD 0b0010
#define IR_CMD_LEFT    0b0011
#define IR_CMD_RIGHT   0b0100

void IR_Motor_Init(void);
void process_ir_command(void);

#endif