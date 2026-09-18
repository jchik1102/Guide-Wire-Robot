#include <stdio.h>
#include "tof_sensors.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "tof_sensors.h"

// Local helper replacements for missing hardware.c functions
void waitms(int ms)
{
    volatile int i, j;
    for(i = 0; i < ms; i++)
    {
        for(j = 0; j < 6000; j++)
        {
            __asm__("nop");
        }
    }
}

void gpio_pin_mode_output(GPIO_TypeDef *port, uint8_t pin)
{
    port->MODER &= ~(3u << (pin * 2u));
    port->MODER |=  (1u << (pin * 2u));
}

void gpio_write_low(GPIO_TypeDef *port, uint8_t pin)
{
    port->BRR = (1u << pin);
}

void gpio_write_high(GPIO_TypeDef *port, uint8_t pin)
{
    port->BSRR = (1u << pin);
}

// ---- VL53L0X register map (internal) ----
#define REG_SYSRANGE_START                             0x00
#define REG_SYSTEM_SEQUENCE_CONFIG                     0x01
#define REG_SYSTEM_INTERRUPT_CONFIG_GPIO               0x0A
#define REG_SYSTEM_INTERRUPT_CLEAR                     0x0B
#define REG_RESULT_INTERRUPT_STATUS                    0x13
#define REG_RESULT_RANGE_STATUS                        0x14
#define REG_GPIO_HV_MUX_ACTIVE_HIGH                    0x84
#define REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV           0x89
#define REG_I2C_SLAVE_DEVICE_ADDRESS                   0x8A
#define REG_IDENTIFICATION_MODEL_ID                    0xC0

#define VL53L0X_EXPECTED_DEVICE_ID                     0xEE

#define RANGE_SEQUENCE_STEP_DSS                        0x28
#define RANGE_SEQUENCE_STEP_PRE_RANGE                  0x40
#define RANGE_SEQUENCE_STEP_FINAL_RANGE                0x80

typedef enum
{
    CALIBRATION_TYPE_VHV,
    CALIBRATION_TYPE_PHASE
} calibration_type_t;


// =============================================================================
//  XSHUT control (PB0)
// =============================================================================

void xshut_init(void)
{
    RCC->IOPENR |= (1u << 1);

    gpio_pin_mode_output(GPIOB, XSHUT_PIN);
    GPIOB->OTYPER &= ~PINMASK(XSHUT_PIN);
    GPIOB->PUPDR &= ~(3u << (XSHUT_PIN * 2u));

    gpio_write_low(GPIOB, XSHUT_PIN);
}

void xshut_low(void)
{
    gpio_pin_mode_output(GPIOB, XSHUT_PIN);
    gpio_write_low(GPIOB, XSHUT_PIN);
}

void xshut_release(void)
{
    gpio_pin_mode_output(GPIOB, XSHUT_PIN);
    gpio_write_high(GPIOB, XSHUT_PIN);
}


// =============================================================================
//  I2C1 (PB6=SCL, PB7=SDA)
// =============================================================================

void I2C_init(void)
{
    RCC->IOPENR |= (1u << 1);
    RCC->APB1ENR |= (1u << 21);

    GPIOB->MODER = (GPIOB->MODER & ~(3u << (6u * 2u))) | (2u << (6u * 2u));
    GPIOB->AFR[0] = (GPIOB->AFR[0] & ~(0xFu << (6u * 4u))) | (1u << (6u * 4u));

    GPIOB->MODER = (GPIOB->MODER & ~(3u << (7u * 2u))) | (2u << (7u * 2u));
    GPIOB->AFR[0] = (GPIOB->AFR[0] & ~(0xFu << (7u * 4u))) | (1u << (7u * 4u));

    GPIOB->OTYPER |= PINMASK(6u) | PINMASK(7u);
    GPIOB->OSPEEDR |= (1u << (6u * 2u)) | (1u << (7u * 2u));

    I2C1->TIMINGR = (uint32_t)0x70420f13;
}


// =============================================================================
//  I2C low-level read/write
// =============================================================================

static bool i2c_write_reg8(uint8_t addr7, uint8_t reg, uint8_t value)
{
    I2C1->CR1 = I2C_CR1_PE;
    I2C1->CR2 = I2C_CR2_AUTOEND | (2u << 16) | ((uint32_t)addr7 << 1);
    I2C1->CR2 |= I2C_CR2_START;

    while ((I2C1->ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {}
    I2C1->TXDR = reg;
    while ((I2C1->ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {}
    I2C1->TXDR = value;
    while ((I2C1->ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {}

    waitms(1);
    return true;
}

static bool i2c_read_reg8(uint8_t addr7, uint8_t reg, uint8_t *value)
{
    I2C1->CR1 = I2C_CR1_PE;
    I2C1->CR2 = I2C_CR2_AUTOEND | (1u << 16) | ((uint32_t)addr7 << 1);
    I2C1->CR2 |= I2C_CR2_START;

    while ((I2C1->ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {}
    I2C1->TXDR = reg;
    while ((I2C1->ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {}

    waitms(1);

    I2C1->CR1 = I2C_CR1_PE | I2C_CR1_RXIE;
    I2C1->CR2 = I2C_CR2_AUTOEND | (1u << 16) | I2C_CR2_RD_WRN | ((uint32_t)addr7 << 1);
    I2C1->CR2 |= I2C_CR2_START;

    while ((I2C1->ISR & I2C_ISR_RXNE) != I2C_ISR_RXNE) {}
    *value = (uint8_t)I2C1->RXDR;
    waitms(1);
    return true;
}

static bool i2c_read_reg16(uint8_t addr7, uint8_t reg, uint16_t *value)
{
    I2C1->CR1 = I2C_CR1_PE;
    I2C1->CR2 = I2C_CR2_AUTOEND | (1u << 16) | ((uint32_t)addr7 << 1);
    I2C1->CR2 |= I2C_CR2_START;

    while ((I2C1->ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {}
    I2C1->TXDR = reg;
    while ((I2C1->ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {}

    waitms(1);

    I2C1->CR1 = I2C_CR1_PE | I2C_CR1_RXIE;
    I2C1->CR2 = I2C_CR2_AUTOEND | (2u << 16) | I2C_CR2_RD_WRN | ((uint32_t)addr7 << 1);
    I2C1->CR2 |= I2C_CR2_START;

    while ((I2C1->ISR & I2C_ISR_RXNE) != I2C_ISR_RXNE) {}
    *value = ((uint16_t)I2C1->RXDR) << 8;

    while ((I2C1->ISR & I2C_ISR_RXNE) != I2C_ISR_RXNE) {}
    *value |= (uint16_t)I2C1->RXDR;

    waitms(1);
    return true;
}


// =============================================================================
//  VL53L0X driver
// =============================================================================

bool vl53l0x_device_is_booted(VL53L0X_Dev *dev)
{
    uint8_t device_id = 0;
    if (!i2c_read_reg8(dev->addr7, REG_IDENTIFICATION_MODEL_ID, &device_id))
        return false;
    return (device_id == VL53L0X_EXPECTED_DEVICE_ID);
}

static bool vl53l0x_data_init(VL53L0X_Dev *dev)
{
    bool success = false;
    uint8_t vhv_config_scl_sda = 0;

    if (!i2c_read_reg8(dev->addr7, REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, &vhv_config_scl_sda))
        return false;

    vhv_config_scl_sda |= 0x01;
    if (!i2c_write_reg8(dev->addr7, REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, vhv_config_scl_sda))
        return false;

    success = i2c_write_reg8(dev->addr7, 0x88, 0x00);
    success &= i2c_write_reg8(dev->addr7, 0x80, 0x01);
    success &= i2c_write_reg8(dev->addr7, 0xFF, 0x01);
    success &= i2c_write_reg8(dev->addr7, 0x00, 0x00);
    success &= i2c_read_reg8(dev->addr7, 0x91, &dev->stop_variable);
    success &= i2c_write_reg8(dev->addr7, 0x00, 0x01);
    success &= i2c_write_reg8(dev->addr7, 0xFF, 0x00);
    success &= i2c_write_reg8(dev->addr7, 0x80, 0x00);

    return success;
}

static bool vl53l0x_load_default_tuning_settings(VL53L0X_Dev *dev)
{
    bool success;

    success = i2c_write_reg8(dev->addr7, 0xFF, 0x01);
    success &= i2c_write_reg8(dev->addr7, 0x00, 0x00);
    success &= i2c_write_reg8(dev->addr7, 0xFF, 0x00);
    success &= i2c_write_reg8(dev->addr7, 0x09, 0x00);
    success &= i2c_write_reg8(dev->addr7, 0x10, 0x00);
    success &= i2c_write_reg8(dev->addr7, 0x11, 0x00);
    success &= i2c_write_reg8(dev->addr7, 0x24, 0x01);
    success &= i2c_write_reg8(dev->addr7, 0x25, 0xFF);
    success &= i2c_write_reg8(dev->addr7, 0x75, 0x00);
    success &= i2c_write_reg8(dev->addr7, 0xFF, 0x01);
    success &= i2c_write_reg8(dev->addr7, 0x4E, 0x2C);
    success &= i2c_write_reg8(dev->addr7, 0x48, 0x00);
    success &= i2c_write_reg8(dev->addr7, 0x30, 0x20);
    success &= i2c_write_reg8(dev->addr7, 0xFF, 0x00);
    success &= i2c_write_reg8(dev->addr7, 0x30, 0x09);
    success &= i2c_write_reg8(dev->addr7, 0x54, 0x00);
    success &= i2c_write_reg8(dev->addr7, 0x31, 0x04);
    success &= i2c_write_reg8(dev->addr7, 0x32, 0x03);
    success &= i2c_write_reg8(dev->addr7, 0x40, 0x83);
    success &= i2c_write_reg8(dev->addr7, 0x46, 0x25);
    success &= i2c_write_reg8(dev->addr7, 0x60, 0x00);
    success &= i2c_write_reg8(dev->addr7, 0x27, 0x00);
    success &= i2c_write_reg8(dev->addr7, 0x50, 0x06);
    success &= i2c_write_reg8(dev->addr7, 0x51, 0x00);
    success &= i2c_write_reg8(dev->addr7, 0x52, 0x96);
    success &= i2c_write_reg8(dev->addr7, 0x56, 0x08);
    success &= i2c_write_reg8(dev->addr7, 0x57, 0x30);
    success &= i2c_write_reg8(dev->addr7, 0x61, 0x00);
    success &= i2c_write_reg8(dev->addr7, 0x62, 0x00);
    success &= i2c_write_reg8(dev->addr7, 0x64, 0x00);
    success &= i2c_write_reg8(dev->addr7, 0x65, 0x00);
    success &= i2c_write_reg8(dev->addr7, 0x66, 0xA0);
    success &= i2c_write_reg8(dev->addr7, 0xFF, 0x01);
    success &= i2c_write_reg8(dev->addr7, 0x22, 0x32);
    success &= i2c_write_reg8(dev->addr7, 0x47, 0x14);
    success &= i2c_write_reg8(dev->addr7, 0x49, 0xFF);
    success &= i2c_write_reg8(dev->addr7, 0x4A, 0x00);
    success &= i2c_write_reg8(dev->addr7, 0xFF, 0x00);
    success &= i2c_write_reg8(dev->addr7, 0x7A, 0x0A);
    success &= i2c_write_reg8(dev->addr7, 0x7B, 0x00);
    success &= i2c_write_reg8(dev->addr7, 0x78, 0x21);
    success &= i2c_write_reg8(dev->addr7, 0xFF, 0x01);
    success &= i2c_write_reg8(dev->addr7, 0x23, 0x34);
    success &= i2c_write_reg8(dev->addr7, 0x42, 0x00);
    success &= i2c_write_reg8(dev->addr7, 0x44, 0xFF);
    success &= i2c_write_reg8(dev->addr7, 0x45, 0x26);
    success &= i2c_write_reg8(dev->addr7, 0x46, 0x05);
    success &= i2c_write_reg8(dev->addr7, 0x40, 0x40);
    success &= i2c_write_reg8(dev->addr7, 0x0E, 0x06);
    success &= i2c_write_reg8(dev->addr7, 0x20, 0x1A);
    success &= i2c_write_reg8(dev->addr7, 0x43, 0x40);
    success &= i2c_write_reg8(dev->addr7, 0xFF, 0x00);
    success &= i2c_write_reg8(dev->addr7, 0x34, 0x03);
    success &= i2c_write_reg8(dev->addr7, 0x35, 0x44);
    success &= i2c_write_reg8(dev->addr7, 0xFF, 0x01);
    success &= i2c_write_reg8(dev->addr7, 0x31, 0x04);
    success &= i2c_write_reg8(dev->addr7, 0x4B, 0x09);
    success &= i2c_write_reg8(dev->addr7, 0x4C, 0x05);
    success &= i2c_write_reg8(dev->addr7, 0x4D, 0x04);
    success &= i2c_write_reg8(dev->addr7, 0xFF, 0x00);
    success &= i2c_write_reg8(dev->addr7, 0x44, 0x00);
    success &= i2c_write_reg8(dev->addr7, 0x45, 0x20);
    success &= i2c_write_reg8(dev->addr7, 0x47, 0x08);
    success &= i2c_write_reg8(dev->addr7, 0x48, 0x28);
    success &= i2c_write_reg8(dev->addr7, 0x67, 0x00);
    success &= i2c_write_reg8(dev->addr7, 0x70, 0x04);
    success &= i2c_write_reg8(dev->addr7, 0x71, 0x01);
    success &= i2c_write_reg8(dev->addr7, 0x72, 0xFE);
    success &= i2c_write_reg8(dev->addr7, 0x76, 0x00);
    success &= i2c_write_reg8(dev->addr7, 0x77, 0x00);
    success &= i2c_write_reg8(dev->addr7, 0xFF, 0x01);
    success &= i2c_write_reg8(dev->addr7, 0x0D, 0x01);
    success &= i2c_write_reg8(dev->addr7, 0xFF, 0x00);
    success &= i2c_write_reg8(dev->addr7, 0x80, 0x01);
    success &= i2c_write_reg8(dev->addr7, 0x01, 0xF8);
    success &= i2c_write_reg8(dev->addr7, 0xFF, 0x01);
    success &= i2c_write_reg8(dev->addr7, 0x8E, 0x01);
    success &= i2c_write_reg8(dev->addr7, 0x00, 0x01);
    success &= i2c_write_reg8(dev->addr7, 0xFF, 0x00);
    success &= i2c_write_reg8(dev->addr7, 0x80, 0x00);

    return success;
}

static bool vl53l0x_configure_interrupt(VL53L0X_Dev *dev)
{
    uint8_t gpio_hv_mux_active_high = 0;

    if (!i2c_write_reg8(dev->addr7, REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04))
        return false;
    if (!i2c_read_reg8(dev->addr7, REG_GPIO_HV_MUX_ACTIVE_HIGH, &gpio_hv_mux_active_high))
        return false;

    gpio_hv_mux_active_high &= (uint8_t)~0x10;

    if (!i2c_write_reg8(dev->addr7, REG_GPIO_HV_MUX_ACTIVE_HIGH, gpio_hv_mux_active_high))
        return false;
    if (!i2c_write_reg8(dev->addr7, REG_SYSTEM_INTERRUPT_CLEAR, 0x01))
        return false;

    return true;
}

static bool vl53l0x_set_sequence_steps_enabled(VL53L0X_Dev *dev, uint8_t sequence_step)
{
    return i2c_write_reg8(dev->addr7, REG_SYSTEM_SEQUENCE_CONFIG, sequence_step);
}

static bool vl53l0x_static_init(VL53L0X_Dev *dev)
{
    if (!vl53l0x_load_default_tuning_settings(dev))
        return false;
    if (!vl53l0x_configure_interrupt(dev))
        return false;
    if (!vl53l0x_set_sequence_steps_enabled(dev,
        RANGE_SEQUENCE_STEP_DSS + RANGE_SEQUENCE_STEP_PRE_RANGE + RANGE_SEQUENCE_STEP_FINAL_RANGE))
        return false;

    return true;
}

static bool vl53l0x_perform_single_ref_calibration(VL53L0X_Dev *dev, calibration_type_t calib_type)
{
    uint8_t sysrange_start = 0;
    uint8_t sequence_config = 0;
    uint8_t interrupt_status = 0;
    bool success = false;

    switch (calib_type)
    {
        case CALIBRATION_TYPE_VHV:
            sequence_config = 0x01;
            sysrange_start = 0x01 | 0x40;
            break;
        case CALIBRATION_TYPE_PHASE:
            sequence_config = 0x02;
            sysrange_start = 0x01 | 0x00;
            break;
    }

    if (!i2c_write_reg8(dev->addr7, REG_SYSTEM_SEQUENCE_CONFIG, sequence_config))
        return false;
    if (!i2c_write_reg8(dev->addr7, REG_SYSRANGE_START, sysrange_start))
        return false;

    do {
        success = i2c_read_reg8(dev->addr7, REG_RESULT_INTERRUPT_STATUS, &interrupt_status);
    } while (success && ((interrupt_status & 0x07) == 0));

    if (!success)
        return false;
    if (!i2c_write_reg8(dev->addr7, REG_SYSTEM_INTERRUPT_CLEAR, 0x01))
        return false;
    if (!i2c_write_reg8(dev->addr7, REG_SYSRANGE_START, 0x00))
        return false;

    return true;
}

static bool vl53l0x_perform_ref_calibration(VL53L0X_Dev *dev)
{
    if (!vl53l0x_perform_single_ref_calibration(dev, CALIBRATION_TYPE_VHV))
        return false;
    if (!vl53l0x_perform_single_ref_calibration(dev, CALIBRATION_TYPE_PHASE))
        return false;
    if (!vl53l0x_set_sequence_steps_enabled(dev,
        RANGE_SEQUENCE_STEP_DSS + RANGE_SEQUENCE_STEP_PRE_RANGE + RANGE_SEQUENCE_STEP_FINAL_RANGE))
        return false;

    return true;
}

bool vl53l0x_init_device(VL53L0X_Dev *dev)
{
    if (!vl53l0x_device_is_booted(dev))
        return false;
    if (!vl53l0x_data_init(dev))
        return false;
    if (!vl53l0x_static_init(dev))
        return false;
    if (!vl53l0x_perform_ref_calibration(dev))
        return false;

    return true;
}

bool vl53l0x_read_range_single_device(VL53L0X_Dev *dev, uint16_t *range)
{
    uint8_t sysrange_start = 0;
    uint8_t interrupt_status = 0;
    bool success;

    success = i2c_write_reg8(dev->addr7, 0x80, 0x01);
    success &= i2c_write_reg8(dev->addr7, 0xFF, 0x01);
    success &= i2c_write_reg8(dev->addr7, 0x00, 0x00);
    success &= i2c_write_reg8(dev->addr7, 0x91, dev->stop_variable);
    success &= i2c_write_reg8(dev->addr7, 0x00, 0x01);
    success &= i2c_write_reg8(dev->addr7, 0xFF, 0x00);
    success &= i2c_write_reg8(dev->addr7, 0x80, 0x00);

    if (!success) return false;

    if (!i2c_write_reg8(dev->addr7, REG_SYSRANGE_START, 0x01))
        return false;

    do {
        success = i2c_read_reg8(dev->addr7, REG_SYSRANGE_START, &sysrange_start);
    } while (success && (sysrange_start & 0x01));

    if (!success) return false;

    do {
        success = i2c_read_reg8(dev->addr7, REG_RESULT_INTERRUPT_STATUS, &interrupt_status);
    } while (success && ((interrupt_status & 0x07) == 0));

    if (!success) return false;

    if (!i2c_read_reg16(dev->addr7, (uint8_t)(REG_RESULT_RANGE_STATUS + 10), range))
        return false;
    if (!i2c_write_reg8(dev->addr7, REG_SYSTEM_INTERRUPT_CLEAR, 0x01))
        return false;

    if ((*range == 8190u) || (*range == 8191u))
        *range = VL53L0X_OUT_OF_RANGE;

    return true;
}


// =============================================================================
//  TOF safety check
// =============================================================================

bool tof_safety_stop(VL53L0X_Dev *tof, bool tof_ok)
{
    uint16_t range = 0;

    // Front sensor: stop if obstacle too close
    if (tof_ok) {
        if (vl53l0x_read_range_single_device(tof, &range)) {
            if (range < TOF_FRONT_STOP_MM)
                return true;
        }
        // Read failed — treat as safe (don't block motors)
    }

    return false;
}


// =============================================================================
//  Debug
// =============================================================================

void print_ref_registers(VL53L0X_Dev *dev, const char *name)
{
    uint8_t val8 = 0;
    uint16_t val16 = 0;

    printf("%s @ 0x%02X\r\n", name, dev->addr7);

    i2c_read_reg8(dev->addr7, 0xC0, &val8);
    printf("  Reg(0xC0): 0x%02x\r\n", val8);
    i2c_read_reg8(dev->addr7, 0xC1, &val8);
    printf("  Reg(0xC1): 0x%02x\r\n", val8);
    i2c_read_reg8(dev->addr7, 0xC2, &val8);
    printf("  Reg(0xC2): 0x%02x\r\n", val8);
    i2c_read_reg16(dev->addr7, 0x51, &val16);
    printf("  Reg(0x51): 0x%04x\r\n", val16);
    i2c_read_reg16(dev->addr7, 0x61, &val16);
    printf("  Reg(0x61): 0x%04x\r\n\r\n", val16);
}
