#ifndef TOF_SENSORS_H
#define TOF_SENSORS_H

#include "hardware.h"

// I2C addresses
#define TOF1_ADDR_DEFAULT          0x29
#define TOF1_ADDR_NEW              0x2A
#define TOF2_ADDR_NEW              0x2B

// LED thresholds
#define TOF1_LED_THRESHOLD_MM      100u
#define TOF2_LED_THRESHOLD_MM      100u
#define VL53L0X_OUT_OF_RANGE       8190u

// Pin assignments
#define LED1_PIN   0u   // PA0
#define LED2_PIN   1u   // PA1
#define XSHUT1_PIN 0u   // PB0
#define XSHUT2_PIN 1u   // PB1

typedef struct
{
    uint8_t addr7;
    uint8_t stop_variable;
} VL53L0X_Dev;

// Subsystem init
void leds_init(void);
void xshut_init(void);
void I2C_init(void);

// LED control
void led1_on(void);
void led1_off(void);
void led2_on(void);
void led2_off(void);

// XSHUT control
void xshut1_low(void);
void xshut2_low(void);
void xshut1_release(void);
void xshut2_release(void);

// VL53L0X driver
bool vl53l0x_device_is_booted(VL53L0X_Dev *dev);
bool vl53l0x_set_address(VL53L0X_Dev *dev, uint8_t new_addr7);
bool vl53l0x_init_device(VL53L0X_Dev *dev);
bool vl53l0x_read_range_single_device(VL53L0X_Dev *dev, uint16_t *range);

// Debug
void print_ref_registers(VL53L0X_Dev *dev, const char *name);

#endif