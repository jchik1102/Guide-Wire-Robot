#include <stdio.h>
#include <stdlib.h>
#include <EFM8LB1.h>

#define SYSCLK 72000000L
#define BAUDRATE 115200L
#define SARCLK 18000000L

// Joystick Thresholds
#define J_HIGH 3000
#define J_LOW  500

// --- Prototypes ---
void Timer3us(unsigned char us);
void waitms(unsigned int ms);
void InitADC(void);
void InitPinADC(unsigned char portno, unsigned char pin_num);
unsigned int ADC_at_Pin(unsigned char pin);
unsigned int GetMV(unsigned char pin);

// --- Hardware Startup ---
char _c51_external_startup (void)
{
    SFRPAGE = 0x00;
    WDTCN = 0xDE; 
    WDTCN = 0xAD; 
  
    VDM0CN=0x80;       
    RSTSRC=0x02|0x04;  

    // Clock Setup for 72MHz
    SFRPAGE = 0x10;
    PFE0CN  = 0x20; 
    SFRPAGE = 0x00;
    CLKSEL = 0x00;
    while ((CLKSEL & 0x80) == 0);
    CLKSEL = 0x03;
    while ((CLKSEL & 0x80) == 0);
    
    // UART & Crossbar
    P0MDOUT |= 0x10; // Enable UART0 TX as push-pull
    XBR0     = 0x01; // UART0 on P0.4(TX) and P0.5(RX)                     
    XBR1     = 0X00;
    XBR2     = 0x40; // Enable crossbar

    // Configure Uart 0
    SCON0 = 0x10;
    TH1 = 0x100-((SYSCLK/BAUDRATE)/(2L*12L));
    TL1 = TH1;      
    TMOD &= ~0xf0;  
    TMOD |=  0x20;                        
    TR1 = 1; 
    TI = 1;  
    
    return 0;
}

// --- Helper Functions ---
void Timer3us(unsigned char us)
{
    unsigned char i;
    CKCON0 |= 0b_0100_0000; 
    TMR3RL = (-(SYSCLK)/1000000L); 
    TMR3 = TMR3RL;
    TMR3CN0 = 0x04; 
    for (i = 0; i < us; i++)
    {
        while (!(TMR3CN0 & 0x80));
        TMR3CN0 &= ~(0x80);
    }
    TMR3CN0 = 0; 
}

void waitms (unsigned int ms)
{
    unsigned int j;
    unsigned char k;
    for(j=0; j<ms; j++)
        for (k=0; k<4; k++) Timer3us(250);
}

void InitADC (void)
{
    SFRPAGE = 0x00;
    ADEN=0; 
    ADC0CN1 = (0x2 << 6); // 14-bit mode
    ADC0CF0 = ((SYSCLK/SARCLK) << 3); 
    ADC0CF1 = (0x1E << 0); 
    ADC0CF2 = (0x1 << 5) | 0x1F; // VDD reference
    ADC0CN2 = 0x00;
    ADEN=1; 
}

void InitPinADC (unsigned char portno, unsigned char pin_num)
{
    unsigned char mask = 1 << pin_num;
    SFRPAGE = 0x20;
    if (portno == 2) {
        P2MDIN &= (~mask); 
        P2SKIP |= mask; 
    }
    SFRPAGE = 0x00;
}

unsigned int ADC_at_Pin(unsigned char pin)
{
    ADC0MX = pin;   
    ADINT = 0;

    ADBUSY = 1;     
    while (!ADINT); 
    return (ADC0);
}

unsigned int GetMV(unsigned char pin)
{
    return (unsigned int)((ADC_at_Pin(pin) * 3300L) / 16383L);
}

// --- Main Loop ---
void main (void)
{
    unsigned int vx, vy;

    // Match these pins to your GetMV calls below
    InitPinADC(2, 1); // P2.1
    InitPinADC(2, 2); // P2.2
    InitADC();

    waitms(500); // Let UART stabilize
    printf("\x1b[2J\x1b[H");  // Clear screen
    printf("\x1b[?25l");      // Hide cursor
    printf("--- EFM8 Joystick Debugger ---\n\n");

    while(1)
    {
        // Reading P2.1 and P2.2
        vx = GetMV(QFP32_MUX_P2_1);
        vy = GetMV(QFP32_MUX_P2_2);

        // Overwrite the same line with \r
        printf("\rX: %4dmV | Y: %4dmV        ", vx, vy);
        
        waitms(100); 
    }
}