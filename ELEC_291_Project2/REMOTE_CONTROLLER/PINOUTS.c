// --- JOYSTICK PINS ---
#define JOYSTICK_VX  P2_1 
#define JOYSTICK_VY  P2_2
#define nreset		 
#define VDD         3295 // millivolts
#define GND         100  // millivolts

// --- IR SENSOR PIN ---
#define TIMER_2_FREQ 38000L // 38kHz for IR sensor
#define TIMER_OUT_2 P1_6    // IR transmitter signal

// --- LCD PINS ---
#define LCD_RS P1_1
#define LCD_E  P1_2
#define LCD_D4 P1_3
#define LCD_D5 P1_4
#define LCD_D6 P1_5
// NOTE: P1.6 IS SKIPPED. IT IS THE IR TRANSMITTER!
#define LCD_D7 P1_7
