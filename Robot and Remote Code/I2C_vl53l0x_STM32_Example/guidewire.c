#include <stdint.h>
#include <stdbool.h>
#include "hardware.h"
#include "ir_motor.h"
#include "guidewire.h"

/*
Guidewire control module for a 3-inductor T layout:
- left and right inductors steer the robot
- center inductor is used for intersection detection

This file is self-contained so it can be added to the current project
without changing the existing headers first.

ADC mapping used here:
- PA2 -> ADC channel 2 -> left inductor
- PA3 -> ADC channel 3 -> center inductor
- PA5 -> ADC channel 5 -> right inductor
*/

#define GUIDEWIRE_LEFT_ADC_CHANNEL        2u
#define GUIDEWIRE_CENTER_ADC_CHANNEL      3u
#define GUIDEWIRE_RIGHT_ADC_CHANNEL       5u

#define GUIDEWIRE_CONFIGURE_ADC_GPIO      1u
#define GUIDEWIRE_ADC_PORT                GPIOA
#define GUIDEWIRE_LEFT_ADC_PIN            2u
#define GUIDEWIRE_CENTER_ADC_PIN          3u
#define GUIDEWIRE_RIGHT_ADC_PIN           5u

#define GUIDEWIRE_SAMPLE_COUNT            8u
#define GUIDEWIRE_TRACK_MIN_SIGNAL        120u
#define GUIDEWIRE_BALANCE_WINDOW          90u
#define GUIDEWIRE_TURN_MARGIN             140u
#define GUIDEWIRE_INTERSECTION_MIN        260u
#define GUIDEWIRE_INTERSECTION_MARGIN     90u

static bool guidewire_adc_ready = false;
static guidewire_state_t guidewire_last_state = GUIDEWIRE_STATE_STOP;
static guidewire_readings_t guidewire_last_readings = {0u, 0u, 0u};

static uint32_t guidewire_channel_mask(uint8_t channel)
{
    return (uint32_t)1u << channel;
}

static void guidewire_gpio_pin_mode_analog(GPIO_TypeDef *port, uint8_t pin)
{
    port->MODER |= (3u << (pin * 2u));
    port->PUPDR &= ~(3u << (pin * 2u));
}

static void guidewire_motor_stop(void)
{
    GPIOA->ODR &= ~(LM1 | LM2 | RM1 | RM2);
}

static void guidewire_motor_forward(void)
{
    GPIOA->ODR &= ~(LM1 | LM2 | RM1 | RM2);
    GPIOA->ODR |= (LM1 | RM1);
}

static void guidewire_motor_left(void)
{
    GPIOA->ODR &= ~(LM1 | LM2 | RM1 | RM2);
    GPIOA->ODR |= (LM2 | RM1);
}

static void guidewire_motor_right(void)
{
    GPIOA->ODR &= ~(LM1 | LM2 | RM1 | RM2);
    GPIOA->ODR |= (LM1 | RM2);
}

static bool guidewire_adc_wait_for_flag(uint32_t flag)
{
    uint32_t timeout = 200000u;

    while (((ADC1->ISR & flag) == 0u) && (timeout > 0u))
    {
        timeout--;
    }

    return (timeout > 0u);
}

void guidewire_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->IOPENR |= BIT0 | BIT1;

    gpio_pin_mode_output(GPIOA, 11u);
    gpio_pin_mode_output(GPIOA, 12u);
    gpio_pin_mode_output(GPIOA, 13u);
    gpio_pin_mode_output(GPIOA, 14u);
    guidewire_motor_stop();

#if GUIDEWIRE_CONFIGURE_ADC_GPIO
    guidewire_gpio_pin_mode_analog(GUIDEWIRE_ADC_PORT, GUIDEWIRE_LEFT_ADC_PIN);
    guidewire_gpio_pin_mode_analog(GUIDEWIRE_ADC_PORT, GUIDEWIRE_CENTER_ADC_PIN);
    guidewire_gpio_pin_mode_analog(GUIDEWIRE_ADC_PORT, GUIDEWIRE_RIGHT_ADC_PIN);
#endif

    if ((ADC1->CR & ADC_CR_ADEN) != 0u)
    {
        ADC1->CR |= ADC_CR_ADDIS;
        waitms(1);
    }

    ADC1->CFGR1 = 0u;
    ADC1->CFGR2 = ADC_CFGR2_CKMODE_0;
    ADC1->SMPR = ADC_SMPR_SMP;
    ADC1->CHSELR = 0u;

    ADC1->CR |= ADC_CR_ADVREGEN;
    waitms(1);

    ADC1->CR |= ADC_CR_ADCAL;
    while ((ADC1->CR & ADC_CR_ADCAL) != 0u)
    {
    }

    ADC1->ISR = ADC_ISR_ADRDY | ADC_ISR_EOC | ADC_ISR_EOSEQ | ADC_ISR_OVR;
    ADC1->CR |= ADC_CR_ADEN;
    guidewire_adc_ready = guidewire_adc_wait_for_flag(ADC_ISR_ADRDY);

    guidewire_motor_stop();
}

uint16_t guidewire_read_adc(uint8_t channel)
{
    if (!guidewire_adc_ready)
    {
        guidewire_init();
    }

    ADC1->CHSELR = guidewire_channel_mask(channel);
    ADC1->ISR = ADC_ISR_EOC | ADC_ISR_EOSEQ | ADC_ISR_OVR;
    ADC1->CR |= ADC_CR_ADSTART;

    if (!guidewire_adc_wait_for_flag(ADC_ISR_EOC))
    {
        return 0u;
    }

    return (uint16_t)(ADC1->DR & ADC_DR_DATA);
}

static uint16_t guidewire_read_filtered(uint8_t channel)
{
    uint32_t sum = 0u;
    uint8_t i;

    for (i = 0u; i < GUIDEWIRE_SAMPLE_COUNT; i++)
    {
        sum += guidewire_read_adc(channel);
    }

    return (uint16_t)(sum / GUIDEWIRE_SAMPLE_COUNT);
}

uint16_t guidewire_read_left_sensor(void)
{
    return guidewire_read_filtered(GUIDEWIRE_LEFT_ADC_CHANNEL);
}

uint16_t guidewire_read_center_sensor(void)
{
    return guidewire_read_filtered(GUIDEWIRE_CENTER_ADC_CHANNEL);
}

uint16_t guidewire_read_right_sensor(void)
{
    return guidewire_read_filtered(GUIDEWIRE_RIGHT_ADC_CHANNEL);
}

guidewire_readings_t guidewire_read_sensors(void)
{
    guidewire_readings_t readings;

    readings.left = guidewire_read_left_sensor();
    readings.center = guidewire_read_center_sensor();
    readings.right = guidewire_read_right_sensor();

    guidewire_last_readings = readings;
    return readings;
}

bool intersection_detection(void)
{
    guidewire_readings_t readings = guidewire_last_readings;

    if (readings.center < GUIDEWIRE_INTERSECTION_MIN)
    {
        return false;
    }

    if (readings.center < (uint16_t)(readings.left + GUIDEWIRE_INTERSECTION_MARGIN))
    {
        return false;
    }

    if (readings.center < (uint16_t)(readings.right + GUIDEWIRE_INTERSECTION_MARGIN))
    {
        return false;
    }

    return true;
}

guidewire_state_t guidewire_follow(void)
{
    int32_t diff;
    uint16_t strongest_side;

    guidewire_last_readings = guidewire_read_sensors();

    if (intersection_detection())
    {
        return GUIDEWIRE_STATE_INTERSECTION;
    }

    strongest_side = (guidewire_last_readings.left > guidewire_last_readings.right) ?
        guidewire_last_readings.left : guidewire_last_readings.right;

    if (strongest_side < GUIDEWIRE_TRACK_MIN_SIGNAL)
    {
        return GUIDEWIRE_STATE_STOP;
    }

    diff = (int32_t)guidewire_last_readings.left - (int32_t)guidewire_last_readings.right;

    if ((diff <= (int32_t)GUIDEWIRE_BALANCE_WINDOW) &&
        (diff >= -(int32_t)GUIDEWIRE_BALANCE_WINDOW))
    {
        return GUIDEWIRE_STATE_FORWARD;
    }

    if (diff > (int32_t)GUIDEWIRE_TURN_MARGIN)
    {
        return GUIDEWIRE_STATE_LEFT;
    }

    if (diff < -(int32_t)GUIDEWIRE_TURN_MARGIN)
    {
        return GUIDEWIRE_STATE_RIGHT;
    }

    if (diff > 0)
    {
        return GUIDEWIRE_STATE_LEFT;
    }

    if (diff < 0)
    {
        return GUIDEWIRE_STATE_RIGHT;
    }

    return GUIDEWIRE_STATE_FORWARD;
}

void guidewire_control(void)
{
    guidewire_last_state = guidewire_follow();

    switch (guidewire_last_state)
    {
        case GUIDEWIRE_STATE_FORWARD:
            guidewire_motor_forward();
            break;

        case GUIDEWIRE_STATE_LEFT:
            guidewire_motor_left();
            break;

        case GUIDEWIRE_STATE_RIGHT:
            guidewire_motor_right();
            break;

        case GUIDEWIRE_STATE_INTERSECTION:
            /* Placeholder behavior for now: stop at intersections. */
            guidewire_motor_stop();
            break;

        case GUIDEWIRE_STATE_STOP:
        default:
            guidewire_motor_stop();
            break;
    }
}

guidewire_state_t guidewire_get_last_state(void)
{
    return guidewire_last_state;
}

guidewire_readings_t guidewire_get_last_readings(void)
{
    return guidewire_last_readings;
}
