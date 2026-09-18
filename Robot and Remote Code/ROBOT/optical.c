// Author: Krish Vashist
// Date: 2026-03-26
// Description: PMW3901 optical flow sensor 
// Tested Processor: STM32L051

#include <stdio.h>
#include <stdlib.h>
#include "../Common/Include/stm32l051xx.h"


// LQFP32 pinout
//                                    ----------
// (OCCUPIED) - 5V              VDD -|1       32|- VSS (OCCUPIED)
//                             PC14 -|2       31|- BOOT0
//                             PC15 -|3       30|- PB7 (I2C1_SDA) (OCCUPIED)
// (OCCUPIED) - BUTTON         NRST -|4       29|- PB6 (I2C1_SCL) (OCCUPIED)
// (OCCUPIED) - 3V             VDDA -|5       28|- PB5 (OCCUPIED) - PMW3901 MOSI
// (OCCUPIED) - MIC1_OUT        PA0 -|6       27|- PB4 (OCCUPIED) - PMW3901 MISO
// (OCCUPIED) - MIC2_OUT        PA1 -|7       26|- PB3 (OCCUPIED) - PMW3901 SCLK
// (OCCUPIED) - GUIDE WIRE      PA2 -|8       25|- PA15 (OCCUPIED) - IR RECEIVER
// (OCCUPIED) - GUIDE WIRE      PA3 -|9       24|- PA14 (OCCUPIED) - MOSFET DRIVERS
// (OCCUPIED) - PMW3901 CS      PA4 -|10      23|- PA13 (OCCUPIED) - MOSFET DRIVERS
// (OCCUPIED) - GUIDE WIRE      PA5 -|11      22|- PA12 (OCCUPIED) - MOSFET DRIVERS
// (OCCUPIED) - MIC3_OUT        PA6 -|12      21|- PA11 (OCCUPIED) - MOSFET DRIVERS
//                              PA7 -|13      20|- PA10 (Reserved for RXD) (OCCUPIED)
// (OCCUPIED) - COLLISION       PB0 -|14      19|- PA9  (Reserved for TXD) (OCCUPIED)
// (OCCUPIED) - COLLISION       PB1 -|15      18|- PA8
// (OCCUPIED)                   VSS -|16      17|- VDD (OCCUPIED)
//                                    ----------


// ----------------------------------------------------------------
// MOTOR PIN DEFINITIONS
// ----------------------------------------------------------------
#define LM1 BIT11
#define LM2 BIT12
#define RM1 BIT13
#define RM2 BIT14

// ----------------------------------------------------------------
// SYSTEM CLOCK
// ----------------------------------------------------------------
#define SYSCLK    32000000L
#define TICK_FREQ 1000L

// ----------------------------------------------------------------
// PMW3901 PIN MACROS
// ----------------------------------------------------------------
#define PMW_CS_LOW()    (GPIOA->ODR &= ~BIT4)
#define PMW_CS_HIGH()   (GPIOA->ODR |=  BIT4)
#define PMW_SCLK_LOW()  (GPIOB->ODR &= ~BIT3)
#define PMW_SCLK_HIGH() (GPIOB->ODR |=  BIT3)
#define PMW_MOSI_LOW()  (GPIOB->ODR &= ~BIT5)
#define PMW_MOSI_HIGH() (GPIOB->ODR |=  BIT5)
#define PMW_MISO_READ() (GPIOB->IDR  &  BIT4)

// ----------------------------------------------------------------
// PMW3901 REGISTER ADDRESSES
// ----------------------------------------------------------------
#define PMW_REG_PRODUCT_ID      0x00
#define PMW_REG_REVISION_ID     0x01
#define PMW_REG_MOTION          0x02
#define PMW_REG_DELTA_X_L       0x03
#define PMW_REG_DELTA_X_H       0x04
#define PMW_REG_DELTA_Y_L       0x05
#define PMW_REG_DELTA_Y_H       0x06
#define PMW_REG_SQUAL           0x07
#define PMW_REG_SHUTTER_LOWER   0x0B
#define PMW_REG_SHUTTER_UPPER   0x0C
#define PMW_REG_MOTION_BURST    0x16
#define PMW_REG_POWER_UP_RESET  0x3A
#define PMW_REG_INV_PRODUCT_ID  0x5F

// Expected identity values (from datasheet)
#define PMW_EXPECTED_PRODUCT    0x49
#define PMW_EXPECTED_REVISION   0x00
#define PMW_EXPECTED_INV        0xB6

// Quality thresholds
#define PMW_SQUAL_MIN           20
#define PMW_SHUTTER_MAX         0x1FFF

// ================================================================
// IR + MOTOR STATE
// ================================================================
volatile uint16_t StartTime     = 0;
volatile uint16_t MeasuredWidth = 0;
volatile int      STATE         = 0;
volatile int      BitCount      = 0;
volatile int      command       = 0;
volatile int      NewData_flag  = 0;

volatile uint32_t msTicks = 0;
void SysTick_Handler(void) {
    msTicks++;
}

// ================================================================
// DELAY FUNCTIONS
// ================================================================
void wait(int ms) {
    for (volatile int i = 0; i < ms * (SYSCLK / TICK_FREQ) / 5; i++);
}

void delay_us(uint32_t us) {
    for (volatile uint32_t i = 0; i < us * 8; i++);
}

// ================================================================
// UART (PA9 = TX)
// ================================================================
void UART_Init(void) {
    GPIOA->MODER &= ~(0x3 << (9 * 2));
    GPIOA->MODER |=  (0x2 << (9 * 2));
    GPIOA->AFR[1] &= ~(0xF << 4);
    GPIOA->AFR[1] |=  (0x4 << 4);

    RCC->APB2ENR |= BIT14;

    USART1->BRR = 277;
    USART1->CR1 = (BIT3 | BIT0);
}

void UART_SendChar(char c) {
    while (!(USART1->ISR & BIT7));
    USART1->TDR = c;
}

void UART_SendString(const char *s) {
    while (*s) UART_SendChar(*s++);
}

void UART_SendHex(uint8_t val) {
    const char hex[] = "0123456789ABCDEF";
    UART_SendChar('0');
    UART_SendChar('x');
    UART_SendChar(hex[val >> 4]);
    UART_SendChar(hex[val & 0x0F]);
}

// ================================================================
// UART NUMBER PRINTING HELPERS
// ================================================================
void UART_SendUint(uint32_t val) {
    char buf[10];
    int idx = 0;
    if (val == 0) { UART_SendChar('0'); return; }
    while (val > 0) { buf[idx++] = '0' + (val % 10); val /= 10; }
    for (int k = idx - 1; k >= 0; k--) UART_SendChar(buf[k]);
}

void UART_SendInt(int32_t val) {
    if (val < 0) { UART_SendChar('-'); UART_SendUint((uint32_t)(-val)); }
    else UART_SendUint((uint32_t)val);
}

void UART_SendFloat1(float f) {
    if (f < 0.0f) { UART_SendChar('-'); f = -f; }
    uint32_t whole = (uint32_t)f;
    uint32_t frac  = (uint32_t)((f - whole) * 1000.0f);
    UART_SendUint(whole);
    UART_SendChar('.');
    // Print leading zeros for 3 digits (e.g. 0.007 not 0.7)
    if (frac < 100) UART_SendChar('0');
    if (frac < 10)  UART_SendChar('0');
    UART_SendUint(frac);
}

// ================================================================
// PMW3901 SPI (bit-bang, Mode 3: CPOL=1 CPHA=1)
// ================================================================
void PMW_SPI_Init(void) {
    RCC->IOPENR |= BIT1;  // GPIOBEN

    // PA4 = CS output
    GPIOA->MODER &= ~(0x3 << (4 * 2));
    GPIOA->MODER |=  (0x1 << (4 * 2));

    // PB3 = SCLK output
    GPIOB->MODER &= ~(0x3 << (3 * 2));
    GPIOB->MODER |=  (0x1 << (3 * 2));

    // PB4 = MISO input, no pull
    GPIOB->MODER &= ~(0x3 << (4 * 2));
    GPIOB->PUPDR &= ~(0x3 << (4 * 2));

    // PB5 = MOSI output
    GPIOB->MODER &= ~(0x3 << (5 * 2));
    GPIOB->MODER |=  (0x1 << (5 * 2));

    PMW_CS_HIGH();
    PMW_SCLK_HIGH();
    PMW_MOSI_LOW();
}

void PMW_SPI_SendByte(uint8_t data) {
    for (int i = 7; i >= 0; i--) {
        if (data & (1 << i)) PMW_MOSI_HIGH();
        else                  PMW_MOSI_LOW();
        PMW_SCLK_LOW();
        delay_us(2);
        PMW_SCLK_HIGH();
        delay_us(2);
    }
}

uint8_t PMW_SPI_RecvByte(void) {
    uint8_t data = 0;
    for (int i = 7; i >= 0; i--) {
        PMW_SCLK_LOW();
        delay_us(2);
        PMW_SCLK_HIGH();
        if (PMW_MISO_READ()) data |= (1 << i);
        delay_us(2);
    }
    return data;
}

uint8_t PMW_ReadReg(uint8_t reg) {
    PMW_CS_LOW();
    delay_us(1);
    PMW_SPI_SendByte(reg & 0x7F);
    delay_us(35);
    uint8_t val = PMW_SPI_RecvByte();
    delay_us(1);
    PMW_CS_HIGH();
    delay_us(20);
    return val;
}

void PMW_WriteReg(uint8_t reg, uint8_t data) {
    PMW_CS_LOW();
    delay_us(1);
    PMW_SPI_SendByte(reg | 0x80);
    PMW_SPI_SendByte(data);
    delay_us(1);
    PMW_CS_HIGH();
    delay_us(50);
}

// ================================================================
// PMW3901 COMPLETE INITIALIZATION
// This is the full PixArt proprietary sequence required to start
// the optical engine. The short sequence from before was NOT enough.
// These register values come from the PMW3901MB datasheet
// "Performance Optimization Registers" section.
// DO NOT modify any values or reorder these writes.
// ================================================================
void PMW_Init(void) {
    // --- Power-up reset ---
    PMW_WriteReg(PMW_REG_POWER_UP_RESET, 0x5A);
    wait(50);

    // Discard startup motion residue (mandatory per datasheet)
    PMW_ReadReg(PMW_REG_MOTION);
    PMW_ReadReg(PMW_REG_DELTA_X_L);
    PMW_ReadReg(PMW_REG_DELTA_X_H);
    PMW_ReadReg(PMW_REG_DELTA_Y_L);
    PMW_ReadReg(PMW_REG_DELTA_Y_H);

    // --- Full performance optimization sequence ---
    // Block 1
    PMW_WriteReg(0x7F, 0x00);
    PMW_WriteReg(0x61, 0xAD);
    PMW_WriteReg(0x7F, 0x03);
    PMW_WriteReg(0x40, 0x00);
    PMW_WriteReg(0x7F, 0x05);
    PMW_WriteReg(0x41, 0xB3);
    PMW_WriteReg(0x43, 0xF1);
    PMW_WriteReg(0x45, 0x14);
    PMW_WriteReg(0x5B, 0x32);
    PMW_WriteReg(0x5F, 0x34);
    PMW_WriteReg(0x7B, 0x08);
    PMW_WriteReg(0x7F, 0x06);
    PMW_WriteReg(0x44, 0x1B);
    PMW_WriteReg(0x40, 0xBF);
    PMW_WriteReg(0x4E, 0x3F);
    PMW_WriteReg(0x7F, 0x08);
    PMW_WriteReg(0x65, 0x20);
    PMW_WriteReg(0x6A, 0x18);
    PMW_WriteReg(0x7F, 0x09);
    PMW_WriteReg(0x4F, 0xAF);
    PMW_WriteReg(0x5F, 0x40);
    PMW_WriteReg(0x48, 0x80);
    PMW_WriteReg(0x49, 0x80);
    PMW_WriteReg(0x57, 0x77);
    PMW_WriteReg(0x60, 0x78);
    PMW_WriteReg(0x61, 0x78);
    PMW_WriteReg(0x62, 0x08);
    PMW_WriteReg(0x63, 0x50);
    PMW_WriteReg(0x7F, 0x0A);
    PMW_WriteReg(0x45, 0x60);
    PMW_WriteReg(0x7F, 0x00);
    PMW_WriteReg(0x4D, 0x11);
    PMW_WriteReg(0x55, 0x80);
    PMW_WriteReg(0x74, 0x1F);
    PMW_WriteReg(0x75, 0x1F);
    PMW_WriteReg(0x4A, 0x78);
    PMW_WriteReg(0x4B, 0x78);
    PMW_WriteReg(0x44, 0x08);
    PMW_WriteReg(0x45, 0x50);
    PMW_WriteReg(0x64, 0xFF);
    PMW_WriteReg(0x65, 0x1F);
    PMW_WriteReg(0x7F, 0x14);
    PMW_WriteReg(0x65, 0x60);
    PMW_WriteReg(0x66, 0x08);
    PMW_WriteReg(0x63, 0x78);
    PMW_WriteReg(0x7F, 0x15);
    PMW_WriteReg(0x48, 0x58);
    PMW_WriteReg(0x7F, 0x07);
    PMW_WriteReg(0x41, 0x0D);
    PMW_WriteReg(0x43, 0x14);
    PMW_WriteReg(0x4B, 0x0E);
    PMW_WriteReg(0x45, 0x0F);
    PMW_WriteReg(0x44, 0x42);
    PMW_WriteReg(0x4C, 0x80);
    PMW_WriteReg(0x7F, 0x10);
    PMW_WriteReg(0x5B, 0x02);
    PMW_WriteReg(0x7F, 0x07);
    PMW_WriteReg(0x40, 0x41);
    PMW_WriteReg(0x70, 0x00);

    wait(10);

    PMW_WriteReg(0x32, 0x44);
    PMW_WriteReg(0x7F, 0x07);
    PMW_WriteReg(0x40, 0x40);
    PMW_WriteReg(0x7F, 0x06);
    PMW_WriteReg(0x62, 0xF0);
    PMW_WriteReg(0x63, 0x00);
    PMW_WriteReg(0x7F, 0x0D);
    PMW_WriteReg(0x48, 0xC0);
    PMW_WriteReg(0x6F, 0xD5);
    PMW_WriteReg(0x7F, 0x00);
    PMW_WriteReg(0x5B, 0xA0);
    PMW_WriteReg(0x4E, 0xA8);
    PMW_WriteReg(0x5A, 0x50);
    PMW_WriteReg(0x40, 0x80);

    // Return to normal register bank
    PMW_WriteReg(0x7F, 0x00);

    // Let the sensor settle after full init
    wait(100);

    // One final motion read to clear any stale data
    PMW_ReadReg(PMW_REG_MOTION);
    PMW_ReadReg(PMW_REG_DELTA_X_L);
    PMW_ReadReg(PMW_REG_DELTA_X_H);
    PMW_ReadReg(PMW_REG_DELTA_Y_L);
    PMW_ReadReg(PMW_REG_DELTA_Y_H);
}

// ================================================================
// PMW3901 MOTION BURST READ
// ================================================================
typedef struct {
    int16_t  delta_x;
    int16_t  delta_y;
    uint8_t  squal;
    uint16_t shutter;
    uint8_t  motion;
} PMW_MotionData;

uint8_t PMW_ReadBurst(PMW_MotionData *m) {
    uint8_t buf[12];

    PMW_CS_LOW();
    delay_us(1);

    PMW_SPI_SendByte(PMW_REG_MOTION_BURST);
    delay_us(35);

    for (int i = 0; i < 12; i++) {
        buf[i] = PMW_SPI_RecvByte();
        delay_us(4);  // small inter-byte delay for reliability
    }

    delay_us(2);
    PMW_CS_HIGH();
    delay_us(2);

    m->motion  = buf[0];
    m->delta_x = (int16_t)((buf[3] << 8) | buf[2]);
    m->delta_y = (int16_t)((buf[5] << 8) | buf[4]);
    m->squal   = buf[6];
    m->shutter = (uint16_t)(((buf[11] & 0x1F) << 8) | buf[10]);

    return (buf[0] & 0x80) ? 1 : 0;
}

// ================================================================
// PMW3901 IDENTITY CHECK
// ================================================================
uint8_t PMW_CheckIdentity(void) {
    UART_SendString("\r\n=== PMW3901 Stage 1: Identity Check ===\r\n");

    uint8_t product_id  = PMW_ReadReg(PMW_REG_PRODUCT_ID);
    uint8_t revision_id = PMW_ReadReg(PMW_REG_REVISION_ID);
    uint8_t inv_product = PMW_ReadReg(PMW_REG_INV_PRODUCT_ID);

    UART_SendString("Product_ID  (0x00): ");
    UART_SendHex(product_id);
    UART_SendString(product_id  == PMW_EXPECTED_PRODUCT  ?
                    "  -> PASS\r\n" : "  -> FAIL (expect 0x49)\r\n");

    UART_SendString("Revision_ID (0x01): ");
    UART_SendHex(revision_id);
    UART_SendString(revision_id == PMW_EXPECTED_REVISION ?
                    "  -> PASS\r\n" : "  -> FAIL (expect 0x00)\r\n");

    UART_SendString("Inv_Product (0x5F): ");
    UART_SendHex(inv_product);
    UART_SendString(inv_product == PMW_EXPECTED_INV      ?
                    "  -> PASS\r\n" : "  -> FAIL (expect 0xB6)\r\n");

    uint8_t pass = (product_id  == PMW_EXPECTED_PRODUCT  &&
                    revision_id == PMW_EXPECTED_REVISION  &&
                    inv_product == PMW_EXPECTED_INV);

    if (pass) {
        UART_SendString(">>> ALL PASS - sensor alive\r\n");
    } else {
        UART_SendString(">>> FAILED\r\n");
        if (product_id == 0xFF)
            UART_SendString("HINT: 0xFF = MISO floating or wrong SPI mode\r\n");
        if (product_id == 0x00)
            UART_SendString("HINT: 0x00 = MOSI/MISO swapped\r\n");
        if (product_id == 0x94)
            UART_SendString("HINT: 0x94 = bit-reversed, LSB/MSB issue\r\n");
    }

    return pass;
}

// ================================================================
// HARDWARE INIT
// ================================================================
void Hardware_Init(void) {
    RCC->IOPENR |= BIT0;   // GPIOA clock
    RCC->APB1ENR |= BIT0;  // TIM2 clock

    GPIOA->MODER = (GPIOA->MODER & ~0x3FF003FF) | 0x15500155;
    GPIOA->ODR &= ~(LM1 | LM2 | RM1 | RM2);

    // PA15 = TIM2 input capture AF2
    GPIOA->MODER = (GPIOA->MODER & ~(BIT30)) | (BIT31);
    GPIOA->AFR[1] |= (BIT30 | BIT28);

    TIM2->PSC  = 31;
    TIM2->ARR  = 0xFFFF;
    TIM2->CCMR1 |= BIT0;
    TIM2->CCER  |= (BIT1 | BIT0 | BIT3);
    TIM2->DIER  |= BIT1;

    NVIC->ISER[0] |= BIT15;
    TIM2->CR1 |= BIT0;
    __enable_irq();
}

// ================================================================
// TIM2 INTERRUPT HANDLER
// ================================================================
void TIM2_Handler(void) {
    if (TIM2->SR & BIT1) {
        uint16_t currentCapture = TIM2->CCR1;

        if (!(GPIOA->IDR & BIT15)) {
            StartTime = currentCapture;
        } else {
            MeasuredWidth = currentCapture - StartTime;

            if (MeasuredWidth > 300 && MeasuredWidth < 600) {
                STATE    = 1;
                BitCount = 0;
                command  = 0;
            } else if (STATE == 1) {
                if (MeasuredWidth > 100 && MeasuredWidth < 350) {
                    command = (command << 1);
                    BitCount++;
                } else if (MeasuredWidth > 650 && MeasuredWidth < 1100) {
                    command = (command << 1) | 1;
                    BitCount++;
                } else {
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

// ================================================================
// MAIN
// ================================================================
int main(void) {
    Hardware_Init();

    SysTick->LOAD = (SYSCLK / 1000) - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL = 7;

    PMW_SPI_Init();
    UART_Init();

    wait(200);

    // --- Stage 1: confirm sensor is alive ---
    uint8_t sensor_ok = PMW_CheckIdentity();

    if (sensor_ok) {
        // --- Full init with complete register sequence ---
        PMW_Init();
        UART_SendString("PMW3901 init complete (full sequence)\r\n");
        // the entire sequence is in the optical.c file

        // Diagnostic: verify SQUAL via single register read
        // Hold sensor ~2cm above textured surface for this test
        wait(200);  // let optical engine settle
        PMW_ReadReg(PMW_REG_MOTION);  // unlatch
        uint8_t test_sq = PMW_ReadReg(PMW_REG_SQUAL);
        UART_SendString("Init SQUAL check: ");
        UART_SendHex(test_sq);
        if (test_sq > 0) {
            UART_SendString("  -> Optical engine RUNNING\r\n");
        } else {
            UART_SendString("  -> Still 0, check surface/distance\r\n");
        }

        UART_SendString("Format: squal | mot | X | Y\r\n");
        UART_SendString("----------------------------------\r\n");
    } else {
        UART_SendString("Sensor init failed - check wiring\r\n");
        UART_SendString("Motor/IR still running\r\n");
    }

    #define MM_PER_COUNT 0.033f
    float x_mm = 0.0f;
    float y_mm = 0.0f;

    uint32_t last_cmd_time = 0;
    uint32_t last_print_time = 0;

    PMW_MotionData m = {0};

    while (1) {
        // --------------------------------------------------------
        // PMW3901 motion reading
        // --------------------------------------------------------
        if (sensor_ok) {
            uint8_t new_data = PMW_ReadBurst(&m);

            if (new_data) {
                uint8_t reliable = (m.squal   >= PMW_SQUAL_MIN) &&
                                   (m.shutter <  PMW_SHUTTER_MAX);
                if (reliable) {
                    x_mm += m.delta_x * MM_PER_COUNT;
                    y_mm += m.delta_y * MM_PER_COUNT;
                }
            }

            if ((msTicks - last_print_time) >= 200) {
                last_print_time = msTicks;

                UART_SendString("squal=");
                UART_SendUint(m.squal);

                UART_SendString("  mot=");
                UART_SendChar((m.motion & 0x80) ? '1' : '0');

                UART_SendString("  X=");
                UART_SendFloat1(x_mm / 10.0f);

                UART_SendString("cm  Y=");
                UART_SendFloat1(y_mm / 10.0f);

                UART_SendString("cm\r\n");
            }
        }

        // --------------------------------------------------------
        // IR remote + motor control
        // NOTE: PA4 is PMW3901 CS — do NOT toggle BIT4 here!
        //       Using PA0 (BIT0) for LED indicator instead.
        // --------------------------------------------------------
        if (NewData_flag == 1) {
            last_cmd_time = msTicks;
            GPIOA->ODR &= ~(LM1 | LM2 | RM1 | RM2);
            wait(5);
            switch (command) {
                case 0b0010:  // Forward
                    GPIOA->ODR |= (LM1 | RM1);
                    GPIOA->ODR |= BIT0;
                    break;
                case 0b0011:  // Turn left
                    GPIOA->ODR |= (LM2 | RM1);
                    break;
                case 0b0100:  // Turn right
                    GPIOA->ODR |= (LM1 | RM2);
                    break;
                case 0b0000:  // Stop
                    GPIOA->ODR &= ~BIT0;
                    break;
            }
            NewData_flag = 0;
        }

        if (last_cmd_time != 0 && (msTicks - last_cmd_time > 250)) {
            GPIOA->ODR &= ~(LM1 | LM2 | RM1 | RM2);
            GPIOA->ODR &= ~BIT0;
            last_cmd_time = 0;
        }
    }
}
