
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
