//  Pins: P1.6 (IR Out), P2.2-2.5 (LEDs), P0.2, P0.5, P3.7, P3.2 (Inputs)

#include <EFM8LB1.h>
#include <stdlib.h>
#include <stdio.h>

#define SYSCLK 72000000L 
#define TIMER_0_FREQ 1000L
#define TIMER_1_FREQ 2000L
#define TIMER_2_FREQ 38000L // IR Carrier Frequency
#define TIMER_3_FREQ 4000L
#define TIMER_4_FREQ 5000L
#define TIMER_5_FREQ 6000L
#define PCA_0_FREQ 7000L
#define PCA_1_FREQ 8000L
#define PCA_2_FREQ 9000L
#define PCA_3_FREQ 10000L
#define PCA_4_FREQ 11000L

// Output Pin Mapping
#define TIMER_OUT_0 P2_0
#define TIMER_OUT_1 P1_7
#define TIMER_OUT_2 P1_6 // IR Transmitter Signal
#define TIMER_OUT_3 P1_5
#define TIMER_OUT_4 P1_4
#define TIMER_OUT_5 P1_3
#define PCA_OUT_0   P1_2
#define PCA_OUT_1   P1_1
#define PCA_OUT_2   P1_0
#define PCA_OUT_3   P0_7
#define PCA_OUT_4   P0_6

// LED Feedback Pins (Push-Pull)
#define LED_LEFT    P2_2
#define LED_RIGHT   P2_3
#define LED_FWD     P2_4
#define LED_SWITCH  P2_5

// Input Pin Mapping
#define BTN_LEFT    P0_2
#define BTN_RIGHT   P0_5
#define BTN_FWD     P3_3
#define BTN_SWITCH  P3_2

char _c51_external_startup (void)
{
    // Disable Watchdog
    SFRPAGE = 0x00;
    WDTCN = 0xDE; 
    WDTCN = 0xAD; 
  
    VDM0CN=0x80;       
    RSTSRC=0x02|0x04;  
    
    // Clock Setup (72MHz)
    SFRPAGE = 0x10;
    PFE0CN  = 0x20; 
    SFRPAGE = 0x00;
    CLKSEL = 0x00;
    while ((CLKSEL & 0x80) == 0);
    CLKSEL = 0x03;
    while ((CLKSEL & 0x80) == 0);

    // Pin Configurations
    P0MDOUT |= 0b_1100_0010; // P0.1, P0.6, P0.7
    P1MDOUT |= 0b_1111_1111; // All P1 as Push-Pull
    
    // P2.0 (Timer0) and P2.2-P2.5 (LEDs) as Push-Pull
    P2MDOUT |= 0b_0011_1101; 
    
    XBR0 = 0x00;                       
    XBR1 = 0X00;
    XBR2 = 0x40; // Enable Crossbar and weak pull-ups for inputs
    
    // Timer 0 (16-bit)
    TR0=0; TF0=0;
    CKCON0 |= 0b_0000_0100;
    TMOD &= 0xf0; TMOD |= 0x01;
    TMR0 = 65536L-(SYSCLK/(2*TIMER_0_FREQ));
    ET0=1; TR0=1;

    // Timer 1 (16-bit)
    TR1=0; TF1=0;
    CKCON0 |= 0b_0000_1000;
    TMOD &= 0x0f; TMOD |= 0x10;
    TMR1 = 65536L-(SYSCLK/(2*TIMER_1_FREQ));
    ET1=1; TR1=1;

    // Timer 2 (38kHz Polled)
    TMR2CN0 = 0x00;
    CKCON0 |= 0b_0001_0000;
    TMR2RL = (0x10000L-(SYSCLK/(2*TIMER_2_FREQ)));
    TMR2 = 0xffff;
    ET2 = 0; // Polled, no interrupt
    TR2 = 1;

    // Timer 3
    TMR3CN0 = 0x00;
    CKCON0 |= 0b_0100_0000;
    TMR3RL = (0x10000L-(SYSCLK/(2*TIMER_3_FREQ)));
    TMR3 = 0xffff;
    EIE1 |= 0b_1000_0000;
    TMR3CN0 |= 0b_0000_0100;

    // Timers 4 & 5 (SFR Page 0x10)
    SFRPAGE = 0x10;
    TMR4RL = (0x10000L-(SYSCLK/(2*TIMER_4_FREQ)));
    TMR4 = 0xffff; EIE2 |= 0b_0000_0100; TR4 = 1;
    TMR5RL = (0x10000L-(SYSCLK/(2*TIMER_5_FREQ)));
    TMR5 = 0xffff; EIE2 |= 0b_0000_1000; TR5 = 1;

    // PCA Initialization
    SFRPAGE = 0x0;
    PCA0MD = 0b_0000_1000;
    PCA0CPM0 = PCA0CPM1 = PCA0CPM2 = PCA0CPM3 = PCA0CPM4 = 0b_0100_1001;
    CR = 1; EIE1 |= 0b_0001_0000;
    
    EA = 1; 
    return 0;
}

// ISRs (Standard Toggling)
void Timer0_ISR (void) interrupt INTERRUPT_TIMER0 {
    SFRPAGE=0x0; TMR0=0x10000L-(SYSCLK/(2*TIMER_0_FREQ)); TIMER_OUT_0=!TIMER_OUT_0;
}
void Timer1_ISR (void) interrupt INTERRUPT_TIMER1 {
    SFRPAGE=0x0; TMR1=0x10000L-(SYSCLK/(2*TIMER_1_FREQ)); TIMER_OUT_1=!TIMER_OUT_1;
}
void Timer3_ISR (void) interrupt INTERRUPT_TIMER3 {
    SFRPAGE=0x0; TMR3CN0&=0b_0011_1111; TIMER_OUT_3=!TIMER_OUT_3;
}
void Timer4_ISR (void) interrupt INTERRUPT_TIMER4 {
    SFRPAGE=0x10; TF4H = 0; TIMER_OUT_4=!TIMER_OUT_4;
}
void Timer5_ISR (void) interrupt INTERRUPT_TIMER5 {
    SFRPAGE=0x10; TF5H = 0; TIMER_OUT_5=!TIMER_OUT_5;
}

void PCA_ISR (void) interrupt INTERRUPT_PCA0 {
    unsigned int j;
    SFRPAGE=0x0;
    if (CCF0) { j=(PCA0CPH0*0x100+PCA0CPL0)+(SYSCLK/(2L*PCA_0_FREQ)); PCA0CPL0=j%0x100; PCA0CPH0=j/0x100; CCF0=0; PCA_OUT_0=!PCA_OUT_0; }
    if (CCF1) { j=(PCA0CPH1*0x100+PCA0CPL1)+(SYSCLK/(2L*PCA_1_FREQ)); PCA0CPL1=j%0x100; PCA0CPH1=j/0x100; CCF1=0; PCA_OUT_1=!PCA_OUT_1; }
    if (CCF2) { j=(PCA0CPH2*0x100+PCA0CPL2)+(SYSCLK/(2L*PCA_2_FREQ)); PCA0CPL2=j%0x100; PCA0CPH2=j/0x100; CCF2=0; PCA_OUT_2=!PCA_OUT_2; }
    if (CCF3) { j=(PCA0CPH3*0x100+PCA0CPL3)+(SYSCLK/(2L*PCA_3_FREQ)); PCA0CPL3=j%0x100; PCA0CPH3=j/0x100; CCF3=0; PCA_OUT_3=!PCA_OUT_3; }
    if (CCF4) { j=(PCA0CPH4*0x100+PCA0CPL4)+(SYSCLK/(2L*PCA_4_FREQ)); PCA0CPL4=j%0x100; PCA0CPH4=j/0x100; CCF4=0; PCA_OUT_4=!PCA_OUT_4; }
    CF=0;
}

// --- IR PROTOCOL FUNCTIONS ---

void wait_cycles(unsigned int n, unsigned char burst) {
    unsigned int count = 0;
    SFRPAGE = 0x00; 
    while(count < n) {
        while(!(TMR2CN0 & 0x80)); // Wait for TF2H
        TMR2CN0 &= ~0x80;         // Clear flag
        if (burst == 1) TIMER_OUT_2 = !TIMER_OUT_2;
        else TIMER_OUT_2 = 0;
        count++;
    }
}

void delay_led_visible(void) {
    wait_cycles(15200, 0); // ~200ms delay for the human eye
}

void send_space(void)  { wait_cycles(38, 0); }
void send_header(void) { wait_cycles(32, 1); send_space(); }
void send_zero(void)   { wait_cycles(16, 1); send_space(); }
void send_one(void)    { wait_cycles(64, 1); send_space(); }

// --- DIRECTION FUNCTIONS WITH LED FEEDBACK ---

void send_left(void) {
    LED_LEFT = 1;
    send_header();
    send_zero(); send_zero(); send_one(); send_one();
    delay_led_visible();
    LED_LEFT = 0;
}

void send_right(void) {
    LED_RIGHT = 1;
    send_header();
    send_zero(); send_one(); send_zero(); send_zero();
    delay_led_visible();
    LED_RIGHT = 0;
}

void send_forward(void) {
    LED_FWD = 1;
    send_header();
    send_zero(); send_zero(); send_one(); send_zero();
    delay_led_visible();
    LED_FWD = 0;
}

void send_switch(void) {
    LED_SWITCH = 1;
    send_header();
    send_zero(); send_one(); send_zero(); send_one();
    delay_led_visible();
    LED_SWITCH = 0;
}

void main (void)
{

    _c51_external_startup();
    
  
    
    while(1)
    {
    
        if (BTN_LEFT == 0)   send_left();
        if (BTN_RIGHT == 0)  send_right();
        if (BTN_FWD == 0)    send_forward();
        if (BTN_SWITCH == 0) send_switch();
        
        {
            unsigned long delay;
            for(delay = 0; delay < 100000; delay++); 
        }
    }
}
