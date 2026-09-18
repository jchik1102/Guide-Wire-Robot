#ifndef GUIDEWIRE_H
#define GUIDEWIRE_H

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    GUIDEWIRE_STATE_STOP = 0,
    GUIDEWIRE_STATE_FORWARD,
    GUIDEWIRE_STATE_LEFT,
    GUIDEWIRE_STATE_RIGHT,
    GUIDEWIRE_STATE_INTERSECTION
} guidewire_state_t;

typedef struct
{
    uint16_t left;
    uint16_t center;
    uint16_t right;
} guidewire_readings_t;

void guidewire_init(void);
void guidewire_control(void);

uint16_t guidewire_read_adc(uint8_t channel);
uint16_t guidewire_read_left_sensor(void);
uint16_t guidewire_read_center_sensor(void);
uint16_t guidewire_read_right_sensor(void);

guidewire_readings_t guidewire_read_sensors(void);
guidewire_state_t guidewire_follow(void);
guidewire_state_t guidewire_get_last_state(void);
guidewire_readings_t guidewire_get_last_readings(void);

bool intersection_detection(void);

#endif
