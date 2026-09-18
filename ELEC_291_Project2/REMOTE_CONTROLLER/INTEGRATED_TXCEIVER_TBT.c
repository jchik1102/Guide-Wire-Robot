// BEST VERSION MARCH 26 2026
// GOOD RESET BUTTON (PIN 3.2)
// AUTO MODE PATH SELECTION VIA LCD
// MANUAL MODE 
// JOYSTICK IMPLEMENTED
// IR CONTROLS NEED TO BE TESTED
// PNP EMITTER AT 3V3 - P0.3 is ACTIVE LOW for buzzer
// PASSIVE BUZZER (CEM1302) - uses tone toggling
// IR RECEIVE ON P0.4 - Timer 0 free-run, Port Match ISR

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
#define SPEAKER_OUT P0_3
#define HAPTIC_OUT  P0_2
#define IR_IN       P0_4    // IR receiver input
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
// ======= IR RECEIVE STATE VARIABLES =======
// ==========================================
// Timer 0 runs at SYSCLK/12 = 6MHz → 1 tick = 0.167us
// Pulse width thresholds (in ticks):
//   Header burst  ~421us → ~2526 ticks → check 1800–3600
//   Zero burst    ~210us → ~1260 ticks → check  600–2100
//   One burst     ~842us → ~5052 ticks → check 3000–6600

volatile unsigned int  ir_start_time = 0;
volatile unsigned int  ir_width      = 0;
volatile int           ir_state      = 0; // 0=IDLE, 1=RECEIVING
volatile int           ir_bit_count  = 0;
volatile unsigned char ir_command    = 0;
volatile bit           ir_new_data   = 0;

// ==========================================
// ========= BUZZER (PASSIVE CEM1302) =======
// ==========================================
void beep_tone(unsigned int ms)
{
    unsigned int i;
    unsigned int cycles = ms * 8;
    for (i = 0; i < cycles; i++)
    {
        SPEAKER_OUT = 0;
        lcd_delay_us(56);
        SPEAKER_OUT = 1;
        lcd_delay_us(56);
    }
    SPEAKER_OUT = 1;
}

void beep_confirm(void) { beep_tone(20); }

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
    P1MDOUT |=  0b_1111_1111; // All P1 push-pull (LCD & IR TX)
    P0MDOUT &= ~0b_0001_1000; // P0.3 open-drain (speaker), P0.4 open-drain (IR IN)
    P0MDOUT |=  0b_0000_0100; // P0.2 push-pull (haptic)
    P0      |=  0b_0001_1000; // Write 1 to P0.3 and P0.4 for input/pull-up
    P2MDOUT |=  0b_0000_0000; // P2 open-drain (ADC)
    P3MDOUT |=  0b_0000_0000; // P3.2 open-drain = input
    P3      |=  0b_0000_0100; // Enable internal pull-up on P3.2
    
    XBR0 = 0x00;                     
    XBR1 = 0x00;
    XBR2 = 0x40; // Enable crossbar
    
    SPEAKER_OUT = 1; // Buzzer OFF at startup
    HAPTIC_OUT  = 0;

    // ---- Timer 0: Free-running 16-bit counter for IR capture ----
    // SYSCLK/12 = 6MHz, 1 tick = 0.167us — completely independent of Timer 2
    TMOD   &= 0xF0;  // Clear Timer 0 mode bits
    TMOD   |= 0x01;  // Timer 0: Mode 1 (16-bit)
    CKCON0 &= ~0x04; // Timer 0 uses SYSCLK/12
    TH0 = 0x00;
    TL0 = 0x00;
    ET0 = 0;         // No Timer 0 overflow interrupt needed
    TR0 = 1;         // Start Timer 0

    // ---- Timer 2: 38kHz IR TX carrier (UNTOUCHED) ----
    TMR2CN0 = 0x00;   
    CKCON0 |= 0b_0001_0000; 
    TMR2RL = (0x10000L - (SYSCLK / (2 * TIMER_2_FREQ))); 
    TMR2   = 0xffff;   
    ET2 = 0;         
    TR2 = 1;

    return 0;
}

// ==========================================
// ========= IR CAPTURE INIT ================
// ==========================================
void InitIR_Capture(void)
{
    // P0.4 already set as open-drain input in _c51_external_startup
    // Set up Port Match to watch P0.4
    P0MASK = 0b_0001_0000; // Monitor P0.4 only
    P0MAT  = 0b_0001_0000; // Expect HIGH (idle: no IR = receiver output HIGH)

    // Enable Port Match interrupt (EMAT bit in EIE1)
    EIE1  |= 0x02;
    EA     = 1; // Global interrupt enable
}

// ==========================================
// ========= PORT MATCH ISR (P0.4) ==========
// ==========================================
// Fires on every edge of P0.4 (IR receiver output)
// IR receiver: HIGH = no signal, LOW = 38kHz burst detected
//
// Falling edge → burst started  → record start time
// Rising edge  → burst ended    → measure width → decode bit

void PortMatch_ISR(void) interrupt 8
{
    unsigned int current_time;
    unsigned char tl, th;

    // Read Timer 0 (atomic 16-bit read)
    tl = TL0;
    th = TH0;
    current_time = ((unsigned int)th << 8) | tl;

    if (!(P0 & 0b_0001_0000))
    {
        // --- FALLING EDGE: burst started ---
        ir_start_time = current_time;
        P0MAT &= ~0b_0001_0000; // Now expect LOW → catch rising edge next
    }
    else
    {
        // --- RISING EDGE: burst ended ---
        ir_width = current_time - ir_start_time;
        P0MAT |= 0b_0001_0000;  // Now expect HIGH → catch falling edge next

        // --- DECODE ---
        if (ir_width > 1800 && ir_width < 3600)
        {
            // Header detected (~421us burst)
            ir_state     = 1;
            ir_bit_count = 0;
            ir_command   = 0;
        }
        else if (ir_state == 1)
        {
            if (ir_width > 600 && ir_width < 2100)
            {
                // Zero bit (~210us burst)
                ir_command = (ir_command << 1);
                ir_bit_count++;
            }
            else if (ir_width > 3000 && ir_width < 6600)
            {
                // One bit (~842us burst)
                ir_command = (ir_command << 1) | 1;
                ir_bit_count++;
            }
            else
            {
                ir_state = 0; // Invalid width, reset and wait for next header
            }

            if (ir_bit_count == 4)
            {
                ir_new_data = 1; // Signal main loop
                ir_state    = 0;
            }
        }
    }
}

// ==========================================
// ================= ADC ====================
// ==========================================
void InitADC(void)
{
    SFRPAGE = 0x00;
    ADEN = 0; 
    ADC0CN1 = (0x2 << 6);
    ADC0CF0 = ((SYSCLK/SARCLK) << 3); 
    ADC0CF1 = (0x1E << 0); 
    ADC0CF2 = (0x1 << 5) | 0x1F;
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
            if (!NRST) return;
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
void send_space(void)        { wait_cycles(38, 0); }
void send_header(void)       { wait_cycles(32, 1); send_space(); }
void send_zero(void)         { wait_cycles(16, 1); send_space(); }
void send_one(void)          { wait_cycles(64, 1); send_space(); }

void send_left(void)    { send_header(); send_zero(); send_zero(); send_one();  send_one();  delay_led_visible(); }
void send_right(void)   { send_header(); send_zero(); send_one();  send_zero(); send_zero(); delay_led_visible(); }
void send_forward(void) { send_header(); send_zero(); send_zero(); send_one();  send_zero(); delay_led_visible(); }
void send_stop(void)    { send_header(); send_zero(); send_one();  send_zero(); send_one();  delay_led_visible(); }

// ==========================================
// ===== IR COMMAND HANDLER (TEMPLATE) ======
// ==========================================


void handle_ir_command(unsigned char cmd)
{
    switch (cmd)
    {
        case 0b00001001: // CLOSE (1001)
            // BEEP QUICKLY
            lcd_command(0x01); 
            lcd_delay_ms(5);
            lcd_set_cursor(0,0);
            lcd_print("CLOSE");
            break;

        case 0b00001010 : // VERY CLOSE (1010)
            lcd_command(0x01); 
            lcd_delay_ms(5);
            lcd_set_cursor(0,0);
            lcd_print("VERY CLOSE");
            break;

        case 0b00001111: // MOTOR ON/OFF (1111)
            lcd_command(0x01); 
            lcd_delay_ms(5);
            lcd_set_cursor(0,0);
            lcd_print("MOTOR ON/OFF");
            break;

        case 0b00000101: // STOP received
            lcd_command(0x01); 
            lcd_delay_ms(5);
            lcd_set_cursor(0,0);
            lcd_print("0101 RECEIVED");
            break;

        default:
            // Unknown command — ignore
            break;
    }
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
    InitIR_Capture(); // Start IR receive on P0.4

reset_label:
    while (!NRST);

    {
        volatile unsigned int wait_time, j;
        for (wait_time = 0; wait_time < 5; wait_time++)
            for (j = 0; j < 8000; j++);
    }

    beep_reset();

    auto_mode   = 0;
    manual_mode = 0;
    load_main_menu_select();

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

            if (vy_mv < THRESH_LOW)
            {
                selected_path++;
                if (selected_path > 3) selected_path = 1;
                update_cursor(selected_path);
                beep_confirm();
                lcd_delay_ms(300); 
            }
            else if (vy_mv > THRESH_HIGH)
            {
                selected_path--;
                if (selected_path < 1) selected_path = 3; 
                update_cursor(selected_path);
                beep_confirm();
                lcd_delay_ms(300); 
            }

            if (vx_mv < THRESH_LOW)
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
            // Check for any received IR commands
            if (ir_new_data) {
                ir_new_data = 0;
                handle_ir_command(ir_command);
            }
            // TODO: path-specific IR send sequences
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

            // Check for any received IR commands
            if (ir_new_data) {
                ir_new_data = 0;
                handle_ir_command(ir_command);
            }

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
