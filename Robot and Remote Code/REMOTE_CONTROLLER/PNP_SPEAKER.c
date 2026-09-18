// BEST VERSION MARCH 26 2026
// GOOD RESET BUTTON (PIN 3.2)
// AUTO MODE PATH SELECTION VIA LCD
// MANUAL MODE 
// JOYSTICK IMPLEMENTED
// IR CONTROLS NEED TO BE TESTED
// PNP EMITTER AT 3V3 - P0.3 is ACTIVE LOW for buzzer
// PASSIVE BUZZER (CEM1302) - uses tone toggling
// 
//
//
//
// BONUS FEATURES TO ADD:
// LAP TIME (SAVING LAP TIMES TOO)
// RECEIVER ON REMOTE END (WAITING FOR KRISH'S CODE)
// INCREASE BEEPING FREQUENCY WHEN GETTING CLOSER TO OBJECT
// DIFFERENT MODES SUCH AS LEARNING MODE TO ADD

#include <EFM8LB1.h>
#include <stdlib.h>
#include <stdio.h>

#define SYSCLK 72000000L 
#define SARCLK 18000000L 
#define TIMER_2_FREQ 38000L

// --- JOYSTICK THRESHOLDS ---
#define VDD 3295 
#define THRESH_LOW  1200
#define THRESH_HIGH 1800

// --- PINS ---
#define NRST        P3_2
#define TIMER_OUT_2 P1_6
#define SPEAKER_OUT P0_3    // PNP base drive - ACTIVE LOW (emitter at 3V3)
#define HAPTIC_OUT  P0_2
#define LCD_RS P1_1
#define LCD_E  P1_2
#define LCD_D4 P1_3
#define LCD_D5 P1_4
#define LCD_D6 P1_5
#define LCD_D7 P1_7

// ==========================================
// ======= FORWARD DECLARATIONS =============
// ==========================================
void lcd_delay_ms(unsigned int ms);
void lcd_delay_us(unsigned int us);

// ==========================================
// ========= BUZZER (PASSIVE CEM1302) =======
// ==========================================
// Passive buzzer needs toggling signal to make tone.
// PNP emitter at 3V3: P0.3=0 -> transistor ON, P0.3=1 -> transistor OFF
// 250us half-period = 2kHz tone
// One alarm "chirp" â€” 4kHz tone for 80ms
void beep_tone(unsigned int ms)
{
    unsigned int i;
    unsigned int cycles = ms * 8; // 8 toggles per ms = ~6kHz (high pitch)
    for (i = 0; i < cycles; i++)
    {
        SPEAKER_OUT = 0;
        lcd_delay_us(56);    // 83us = ~6kHz, much higher pitch
        SPEAKER_OUT = 1;
        lcd_delay_us(56);
    }
    SPEAKER_OUT = 1;
}

void beep_confirm(void) { beep_tone(20); }  // Very short & snappy

void beep_reset(void)
{
    beep_tone(20);
    lcd_delay_ms(40);
    beep_tone(20);
}

// ==========================================
// ======== HARDWARE INITIALIZATION =========
// ==========================================
char _c51_external_startup (void)
{
    SFRPAGE = 0x00;
    WDTCN = 0xDE; 
    WDTCN = 0xAD; 
  
    VDM0CN = 0x80;       
    RSTSRC = 0x02 | 0x04;  
    
    // Set clock to 72MHz
    SFRPAGE = 0x10;
    PFE0CN  = 0x20; 
    SFRPAGE = 0x00;
    CLKSEL = 0x00;
    while ((CLKSEL & 0x80) == 0);
    CLKSEL = 0x03;
    while ((CLKSEL & 0x80) == 0);

    // Configure pins
    P1MDOUT |= 0b_1111_1111; // All P1 push-pull (LCD & IR)
    P0MDOUT |= 0b_0000_1100; // P0.2 (haptic) and P0.3 (speaker) push-pull 3V3 CONFIG

    P0MDOUT &= ~0b_0000_1000; // P0.3 open-drain (clear bit 3, NOT set it) OPEN DRAIN CONFIG

    P2MDOUT |= 0b_0000_0000; // P2 open-drain (ADC)
    P3MDOUT |= 0b_0000_0000; // P3.2 open-drain = input
    P3      |= 0b_0000_0100; // Enable internal pull-up on P3.2
    
    XBR0 = 0x00;                     
    XBR1 = 0x00;
    XBR2 = 0x40; // Enable crossbar
    
    // Default speaker HIGH = transistor OFF = buzzer silent
    SPEAKER_OUT = 1;
    HAPTIC_OUT  = 0;

    // Initialize Timer 2 (38kHz for IR)
    TMR2CN0 = 0x00;   
    CKCON0 |= 0b_0001_0000; 
    TMR2RL = (0x10000L - (SYSCLK / (2 * TIMER_2_FREQ))); 
    TMR2   = 0xffff;   
    ET2 = 0;         
    TR2 = 1;         

    return 0;
}

// ==========================================
// ================= ADC ====================
// ==========================================
void InitADC(void)
{
    SFRPAGE = 0x00;
    ADEN = 0; 
    ADC0CN1 = (0x2 << 6);           // 14-bit mode
    ADC0CF0 = ((SYSCLK/SARCLK) << 3); 
    ADC0CF1 = (0x1E << 0); 
    ADC0CF2 = (0x1 << 5) | 0x1F;   // VDD reference
    ADC0CN0 = 0x00; 
    ADC0CN2 = 0x00;
    ADEN = 1; 
}

void InitPinADC(unsigned char portno, unsigned char pin_num)
{
    unsigned char mask = 1 << pin_num;
    SFRPAGE = 0x20;
    if (portno == 2) {
        P2MDIN &= (~mask); 
        P2SKIP  |= mask;    
    }
    SFRPAGE = 0x00;
}

unsigned int ADC_at_Pin(unsigned char pin)
{
    SFRPAGE = 0x00; 
    ADC0MX = pin;   
    ADINT  = 0;
    ADBUSY = 1;     
    while (!ADINT); 
    return (ADC0);
}

unsigned int Millivolts_at_Pin(unsigned char pin)
{
    return (unsigned int)((ADC_at_Pin(pin) * 3300L) / 16383L); 
}

// ==========================================
// ========== TIMERS & LCD DRIVERS ==========
// ==========================================
void lcd_delay_ms(unsigned int ms)
{
    volatile unsigned int i, j;
    for (i = 0; i < ms; i++) {
        for (j = 0; j < 8000; j++) { 
            if (!NRST) return; // Abort on reset
        } 
    }
}

void lcd_delay_us(unsigned int us)
{
    volatile unsigned int i, j;
    for (i = 0; i < us; i++) {
        for (j = 0; j < 20; j++) {}
    }
}

void lcd_pulse_enable(void) {
    LCD_E = 1; lcd_delay_us(50); 
    LCD_E = 0; lcd_delay_us(100); 
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
    LCD_RS = 0; LCD_E = 0;
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
    while (count < n)
    {
        while (!(TMR2CN0 & 0x80));
        TMR2CN0 &= ~0x80;
        if (burst == 1) TIMER_OUT_2 = !TIMER_OUT_2; 
        else            TIMER_OUT_2 = 0;            
        count++;
    }
}

void delay_led_visible(void) { wait_cycles(15200, 0); }

void send_space(void)   { wait_cycles(38, 0); }
void send_header(void)  { wait_cycles(32, 1); send_space(); }
void send_zero(void)    { wait_cycles(16, 1); send_space(); }
void send_one(void)     { wait_cycles(64, 1); send_space(); }

void send_left(void) {
    send_header();
    send_zero(); send_zero(); send_one(); send_one();
    delay_led_visible(); 
}

void send_right(void) {
    send_header();
    send_zero(); send_one(); send_zero(); send_zero();
    delay_led_visible();
}

void send_forward(void) {
    send_header();
    send_zero(); send_zero(); send_one(); send_zero();
    delay_led_visible();
}

void send_stop(void) {
    send_header();
    send_zero(); send_one(); send_zero(); send_one();
    delay_led_visible();
}

// ==========================================
// ============== UI FUNCTIONS ==============
// ==========================================
void load_main_menu_select(void)
{
    lcd_command(0x01); lcd_delay_ms(5);
    lcd_set_cursor(0,0); lcd_print("UP for auto     ");
    lcd_set_cursor(1,0); lcd_print("DOWN for manual ");
}

void load_path_select(void)
{
    lcd_command(0x01); lcd_delay_ms(5);
    lcd_set_cursor(0,0); lcd_print(" Path 1   Path 2");
    lcd_set_cursor(1,0); lcd_print(" Path 3         ");
}

void update_cursor(int path_number)
{
    lcd_set_cursor(0, 0); lcd_data(' ');
    lcd_set_cursor(0, 9); lcd_data(' ');
    lcd_set_cursor(1, 0); lcd_data(' ');

    if      (path_number == 1) { lcd_set_cursor(0, 0); lcd_data('>'); }
    else if (path_number == 2) { lcd_set_cursor(0, 9); lcd_data('>'); }
    else if (path_number == 3) { lcd_set_cursor(1, 0); lcd_data('>'); }
}

// ==========================================
// ================= MAIN ===================
// ==========================================
void main(void)
{
    unsigned int vx_mv = 0; 
    unsigned int vy_mv = 0;
    bit auto_mode   = 0;
    bit manual_mode = 0;
    int selected_path = 1;

    _c51_external_startup();
    lcd_init();

    InitPinADC(2, 1); // Vx
    InitPinADC(2, 2); // Vy
    InitADC();

reset_label:
    while (!NRST); // Wait for reset button to be released

    // Debounce / settle delay
    {
        volatile unsigned int wait_time, j;
        for (wait_time = 0; wait_time < 5; wait_time++)
            for (j = 0; j < 8000; j++);
    }

    beep_reset(); // Two beeps confirm reset

    auto_mode   = 0;
    manual_mode = 0;
    load_main_menu_select();

    // --- MAIN MENU ---
    while (auto_mode == 0 && manual_mode == 0) 
    {
        vx_mv = Millivolts_at_Pin(QFP32_MUX_P2_1);
        
        if      (vx_mv < THRESH_LOW)  auto_mode   = 1;
        else if (vx_mv > THRESH_HIGH) manual_mode = 1;
    }
    beep_confirm();

    // ==========================================
    // ============= AUTO MODE ==================
    // ==========================================
    if (auto_mode) 
    {
        load_path_select();
        update_cursor(selected_path);
        lcd_delay_ms(200);

        while (1) 
        {
            vx_mv = Millivolts_at_Pin(QFP32_MUX_P2_1);
            vy_mv = Millivolts_at_Pin(QFP32_MUX_P2_2);

            if (vy_mv < THRESH_LOW) // RIGHT
            {
                selected_path++;
                if (selected_path > 3) selected_path = 1;
                update_cursor(selected_path);
                beep_confirm();
                lcd_delay_ms(300); 
            }
            else if (vy_mv > THRESH_HIGH) // LEFT
            {
                selected_path--;
                if (selected_path < 1) selected_path = 3; 
                update_cursor(selected_path);
                beep_confirm();
                lcd_delay_ms(300); 
            }

            if (vx_mv < THRESH_LOW) // UP = confirm
            {
                beep_confirm();
                lcd_command(0x01); lcd_delay_ms(5);
                lcd_set_cursor(0,0); lcd_print("Running Auto...");
                lcd_set_cursor(1,0);

                if (selected_path == 1) lcd_print("Path 1 Selected");
                if (selected_path == 2) lcd_print("Path 2 Selected");
                if (selected_path == 3) lcd_print("Path 3 Selected");

                lcd_delay_ms(1500); 
                break;
            }

            if (!NRST) goto reset_label;
        }

        // --- AUTO MODE IR TRANSMISSION ---
        while (1) {
            // TODO: Add path-specific IR sequences here
            if (!NRST) goto reset_label;
        }
    }

    // ==========================================
    // ============= MANUAL MODE ================
    // ==========================================
    if (manual_mode)
    {
        lcd_command(0x01); lcd_delay_ms(5);
        lcd_set_cursor(0,0); lcd_print("MANUAL MODE");
        
        while (1)
        {
            if (!NRST) goto reset_label;

            vx_mv = Millivolts_at_Pin(QFP32_MUX_P2_1); 
            vy_mv = Millivolts_at_Pin(QFP32_MUX_P2_2);
            
            if (vx_mv < THRESH_LOW)
            {
                lcd_set_cursor(1,0); lcd_print("Sending FWD...  ");
                send_forward();
            }
            else if (vx_mv > THRESH_HIGH)
            {
                lcd_set_cursor(1,0); lcd_print("Sending BWD...  ");
                // send_backward(); // TODO
            }
            else if (vy_mv < THRESH_LOW)
            {
                lcd_set_cursor(1,0); lcd_print("Sending RIGHT...");
                send_right();
            }
            else if (vy_mv > THRESH_HIGH)
            {
                lcd_set_cursor(1,0); lcd_print("Sending LEFT... ");
                send_left();
            }
            else
            {
                lcd_set_cursor(1,0); lcd_print("Stopped         ");
                send_stop();
            }
            
            lcd_delay_ms(50);
        }
    }
}
