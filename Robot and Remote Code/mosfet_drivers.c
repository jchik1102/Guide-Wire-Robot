/* MOSFET Drivers Assumption
    LM1 - left forward
    LM2 - left backward
    RM1 - right forward
    RM2 - right backward
*/




volatile int* const MOTOR = (int *)0xFF200000; // Base address for motor

#define LM1 0x01 // Bit 0
#define LM2 0x02 // Bit 1
#define RM1 0x04 // Bit 2
#define RM2 0x08 // Bit 3

void left_move_forward() {
    *MOTOR &= ~(LM1 | LM2); // Clear left motor bits
    *MOTOR |= LM1; // Set LM1 high, LM2 low
}

void left_move_backward() {
    *MOTOR &= ~(LM1 | LM2); // Clear left motor bits
    *MOTOR |= LM2; // Set LM2 high, LM1 low
}

void right_move_forward() {
    *MOTOR &= ~(RM1 | RM2); // Clear right motor bits
    *MOTOR |= RM1; // Set RM1 high, RM2 low
}

void right_move_backward() {
    *MOTOR &= ~(RM1 | RM2); // Clear right motor bits
    *MOTOR |= RM2; // Set RM2 high, RM1 low
}

void stop_motors() {
    *MOTOR &= ~(LM1 | LM2 | RM1 | RM2); // Clear all motor bits to stop the car
}

void move_forward() {
    left_move_forward();
    right_move_forward();
}

void turn_left() {
    left_move_backward();
    right_move_forward();
}

void turn_right() {
    left_move_forward();
    right_move_backward();
}

void move_backward() {
    left_move_backward();
    right_move_backward();
}

void stop_robot() {
    stop_motors();
}


