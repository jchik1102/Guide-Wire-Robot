#ifndef TOF_SENSORS_H
#define TOF_SENSORS_H

#include "hardware.h"

// I2C address (single sensor, default address)
#define TOF_ADDR                   0x29

// Safety threshold
#define TOF_FRONT_STOP_MM          100u
#define VL53L0X_OUT_OF_RANGE       8190u

// Pin assignments
#define XSHUT_PIN 0u   // PB0

typedef struct
{
    uint8_t addr7;
    uint8_t stop_variable;
} VL53L0X_Dev;

// Subsystem init
void xshut_init(void);
void I2C_init(void);

// XSHUT control
void xshut_low(void);
void xshut_release(void);

// VL53L0X driver
bool vl53l0x_device_is_booted(VL53L0X_Dev *dev);
bool vl53l0x_init_device(VL53L0X_Dev *dev);
bool vl53l0x_read_range_single_device(VL53L0X_Dev *dev, uint16_t *range);

// Safety: returns true if motors should be stopped
// If sensor failed init (ok=false), motors are allowed
bool tof_safety_stop(VL53L0X_Dev *tof, bool tof_ok);

// Debug
void print_ref_registers(VL53L0X_Dev *dev, const char *name);

#endif
