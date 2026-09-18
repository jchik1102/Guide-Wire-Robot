#include <EFM8LB1.h>
#include <stdlib.h>
#include <stdio.h>

#define SYSCLK 72000000L // SYSCLK frequency in Hz
#define SARCLK 18000000L // Max ADC clock is 18MHz

// --- JOYSTICK PINS ---
#define JOYSTICK_VX  P2_1 
#define JOYSTICK_VY  P2_2
#define NRST		 P3_2
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

// ==========================================
// ====== CUSTOM TYPES & GLOBAL VARIABLES === not used in this code 
// ==========================================
typedef enum {
    RESET, // 0 
    AUTO,  // 1
    MANUAL // 2
} LCD_STATE_t;

LCD_STATE_t current_state = RESET;


// ==========================================
// ======== HARDWARE INITIALIZATION =========
// ==========================================
char _c51_external_startup (void)
{
    // Disable Watchdog 
    SFRPAGE = 0x00;
    WDTCN = 0xDE; 
    WDTCN = 0xAD; 
  
    VDM0CN=0x80;       // enable VDD monitor
    RSTSRC=0x02|0x04;  // Enable reset on missing clock detector and VDD
    
    // Set clock to 72MHz
    SFRPAGE = 0x10;
    PFE0CN  = 0x20; 
    SFRPAGE = 0x00;
    CLKSEL = 0x00;
    CLKSEL = 0x00;
    while ((CLKSEL & 0x80) == 0);
    CLKSEL = 0x03;
    CLKSEL = 0x03;
    while ((CLKSEL & 0x80) == 0);

    // Configure the pins 
    P1MDOUT|=0b_1111_1111; // All P1 pins Push-Pull (LCD & IR)
    P2MDOUT|=0b_0000_0000; // P2 pins Open-Drain (Good for ADC)
    
    XBR0 = 0x00;                     
    XBR1 = 0X00;
    XBR2 = 0x40; // Enable crossbar and weak pull-ups
    
    // Initialize Timer 2 (IR SENSOR TIMER ONLY!)
    TMR2CN0=0x00;   
    CKCON0|=0b_0001_0000; 
    TMR2RL=(0x10000L-(SYSCLK/(2*TIMER_2_FREQ))); 
    TMR2=0xffff;   
    ET2=0;         // Interrupts disabled, we poll this manually!
    TR2=1;         

    return 0;
}

void InitADC (void)
{
    SFRPAGE = 0x00;
    ADEN=0; // Disable ADC
    
    ADC0CN1=
        (0x2 << 6) | // <--- REVERTED TO 14-BIT MODE
        (0x0 << 3) |         
        (0x0 << 0) ; 
    
    ADC0CF0=
        ((SYSCLK/SARCLK) << 3) | 
        (0x0 << 2); 
    
    ADC0CF1=
        (0 << 7)   | 
        (0x1E << 0); 
    
    ADC0CN0 = 0x00; 

    ADC0CF2= 
        (0x0 << 7) | // GND reference
        (0x1 << 5) | // VDD reference
        (0x1F << 0); 
    
    ADC0CN2 = 0x00;

    ADEN=1; // Enable ADC
}

void InitPinADC (unsigned char portno, unsigned char pin_num)
{
    unsigned char mask = 1 << pin_num;
    SFRPAGE = 0x20;
    if (portno == 2) {
        P2MDIN &= (~mask); // Set pin as analog input
        P2SKIP |= mask;    // Skip Crossbar decoding
    }
    SFRPAGE = 0x00;
}

unsigned int ADC_at_Pin(unsigned char pin)
{
    SFRPAGE = 0x00; // Ensure we are on ADC page
    ADC0MX = pin;   
    ADINT = 0;
    ADBUSY = 1;     
    while (!ADINT); 
    return (ADC0);
}

unsigned int Millivolts_at_Pin(unsigned char pin)
{
     // <--- REVERTED TO 14-BIT MATH (16383)
     return ((ADC_at_Pin(pin) * 3300L) / 16383L); 
}

// ==========================================
// ========== LCD DRIVER FUNCTIONS ==========
// ==========================================
void lcd_delay_ms(unsigned int ms) {
    volatile unsigned int i, j;
    for(i = 0; i < ms; i++) {
        for(j = 0; j < 20000; j++) {} 
    }
}
void lcd_delay_us(unsigned int us) {
    volatile unsigned int i, j;
    for(i = 0; i < us; i++) {
        for(j = 0; j < 20; j++) {}
    }
}
void lcd_pulse_enable(void) {
    LCD_E = 1;
    lcd_delay_us(50); 
    LCD_E = 0;
    lcd_delay_us(100); 
}
void lcd_send_nibble(unsigned char nibble) {
    LCD_D4 = (nibble & 0x01) ? 1 : 0;
    LCD_D5 = (nibble & 0x02) ? 1 : 0;
    LCD_D6 = (nibble & 0x04) ? 1 : 0;
    LCD_D7 = (nibble & 0x08) ? 1 : 0;
    lcd_pulse_enable();
}
void lcd_command(unsigned char cmd) {
    LCD_RS = 0; 
    lcd_send_nibble(cmd >> 4);   
    lcd_send_nibble(cmd & 0x0F); 
    if (cmd < 4) lcd_delay_ms(5); 
    else         lcd_delay_us(100);
}
void lcd_data(unsigned char data_char) {
    LCD_RS = 1; 
    lcd_send_nibble(data_char >> 4);
    lcd_send_nibble(data_char & 0x0F);
    lcd_delay_us(100);
}
void lcd_init(void) {
    lcd_delay_ms(100); 
    LCD_RS = 0;
    LCD_E = 0;
    lcd_send_nibble(0x03); lcd_delay_ms(10);      
    lcd_send_nibble(0x03); lcd_delay_ms(2);       
    lcd_send_nibble(0x03); lcd_delay_ms(2);
    lcd_send_nibble(0x02); lcd_delay_ms(2);
    lcd_command(0x28); 
    lcd_command(0x08); 
    lcd_command(0x01); lcd_delay_ms(10);  
    lcd_command(0x06); 
    lcd_command(0x0C); 
}
void lcd_print(char* str) {
    while (*str) lcd_data(*str++); 
}
void lcd_set_cursor(unsigned char row, unsigned char col) {
    unsigned char address;
    if (row == 0) address = 0x80 + col; 
    else          address = 0xC0 + col; 
    lcd_command(address);
}

// ==========================================
// ====== IR TRANSMISSION FUNCTIONS =========
// ==========================================
void wait_cycles(unsigned int n, unsigned char burst)
{
    unsigned int count = 0;
    SFRPAGE = 0x00; 
    while(count < n)
    {
        while(!(TMR2CN0 & 0x80)); // Wait for Timer 2 overflow
        TMR2CN0 &= ~0x80;         // Clear flag
        
        if (burst == 1) TIMER_OUT_2 = !TIMER_OUT_2; 
        else            TIMER_OUT_2 = 0;            
        count++;
    }
}
void send_space(void) { wait_cycles(38, 0); }
void send_header(void) { wait_cycles(32, 1); send_space(); }
void send_zero(void) { wait_cycles(16, 1); send_space(); }
void send_one(void) { wait_cycles(64, 1); send_space(); }

void send_forward(void) {
    send_header(); send_one(); send_zero(); send_zero(); send_one(); 
}
void send_left(void) {
    send_header(); send_zero(); send_zero(); send_zero(); send_one(); 
}
void send_right(void) {
    send_header(); send_one(); send_zero(); send_zero(); send_zero(); 
}
void send_stop(void) {
    send_header(); send_zero(); send_zero(); send_zero(); send_zero(); 
}

void load_main_menu_select(void)
{
    lcd_init();
    lcd_set_cursor(0,0);
    lcd_print("UP for auto         ");

    lcd_set_cursor(1,0); 
    lcd_print("DOWN for manual ");
}

void load_path_select(void)
{
    lcd_init();

    lcd_set_cursor(0,0);
    lcd_print("Path 1   Path 2 ");

    lcd_set_cursor(1,0);
    lcd_print("Path 3          ");
}

void play_music(void)
{
}

// ==========================================
// ================= MAIN ===================
// ==========================================
void main (void)
{
    // initialization
    unsigned int vx_mv = 0; 
    unsigned int vy_mv = 0;

    bit auto_mode   = 0;
    bit manual_mode = 0;

    _c51_external_startup();
    
    lcd_init();

    InitPinADC(2, 1); 
    InitPinADC(2, 2); 
    InitADC();
    
    // end of initialization
    
    // JOYSTICK ONLY TEST CODE
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    while(1)
    {
        vx_mv = Millivolts_at_Pin(QFP32_MUX_P2_1);
        vy_mv = Millivolts_at_Pin(QFP32_MUX_P2_2);

        if 		(vx_mv < 200) send_forward();`
        else if (vx_mv > 2800) send_stop();
        
        if 		(vy_mv > 2800) send_left();
        else if (vy_mv < 200) send_right();
        
        else				  
        
        lcd_delay_ms(50);
    }
    // END OF JOYSTICK ONLY TEST CODE
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /*

    
    while (1)
    {
    	if (!NRST) goto reset_label:
    	
    	if (!P3_1) run_path_1();
    	if (!P3_0) run_path_2();
    	if (!P2_6) run_path_3();
    	if (!P3_7) play_music();
    	*/

    
/*
reset_label:
    load_main_menu_select();

    while (auto_mode == 0 && manual_mode == 0) // if none have been selecting, keep reading the voltage and wait
    {
        vx_mv = Millivolts_at_Pin(QFP32_MUX_P2_2);
        if      (vx_mv < VDD/2) auto_mode = 1;
        else if (vx_mv > 1750)  manual_mode = 1;
    }

    // auto or manual mode has been selected

    if (auto_mode) load_path_select();

    if (vx_mv < VDD/2) // if UP (relative to the orientation of the remote)


    while(1)
    {
        if (!NRST) goto reset_label;

        vx_mv = Millivolts_at_Pin(QFP32_MUX_P2_1); 
        vy_mv = Millivolts_at_Pin(QFP32_MUX_P2_2);
        
        if (vx_mv > 3000)
        {
            lcd_set_cursor(0,0);
            lcd_print("going up     ");
        }
        else if (vx_mv < 500)
        {
            lcd_set_cursor(0,0);
            lcd_print("going down    ");
        }
        else if (vx_mv > 1400 && vx_mv < 1800)
        {
            lcd_set_cursor(0,0);
            lcd_print("in the centre  ");
        }

       */
                
    
}