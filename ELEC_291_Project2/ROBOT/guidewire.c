//  Author: Krish Vashist & Nathan Law
    //  Date: 2026-03-16
    //  Description: This file contains the working in progress code for the entire ROBOT setup

    #include <stdio.h>
    #include <stdlib.h>
    #include <stdbool.h>
    #include "../Common/Include/stm32l051xx.h"

    // PIN MAPPING (configure these to MODER register)
    // PA11 -> LM1 (Pin 21)
    // PA12 -> LM2 (Pin 22)
    // PA13 -> RM1 (Pin 23)
    // PA14 -> RM2 (Pin 24)
    // PA15 -> IR receiver input (Pin 25)
    /* straight: GPIOA->ODR |= (LM1 | RM1);
       left:     GPIOA->ODR |= (LM2 | RM1);
       right:    GPIOA->ODR |= (LM1 | RM2);
       stop:     GPIOA->ODR &= ~(LM1 | LM2 | RM1 | RM2); */

    // -- MOTOR PIN DEFINITIONS --
    #define LM1 BIT7
    #define LM2 BIT8

    #define RM1 BIT13
    #define RM2 BIT14

    #define SYSCLK    32000000L
    #define TICK_FREQ 1000L

    void wait(int ms)
    {
        for (volatile int i = 0; i < ms * (SYSCLK / TICK_FREQ) / 5; i++);
    }

    void Hardware_Init(void)
    {
        RCC->IOPENR |= BIT0;   // GPIOA Clock

        // Configure PA1-PA4 as LEDs (0x155)
        // AND PA11-PA14 as Motor Outputs (0x5500000)
        GPIOA->MODER = (GPIOA->MODER & ~0x3C03C3FF) | 0x14014155;


        // Initially stop the car
        GPIOA->ODR &= ~(LM1 | LM2 | RM1 | RM2);
    }

    // ---- Guidewire ADC pin/channel mapping ----
    // PA2 -> ADC channel 2 -> left inductor
    // PA3 -> ADC channel 3 -> center inductor
    // PA5 -> ADC channel 5 -> right inductor

    #define GUIDEWIRE_LEFT_ADC_CHANNEL        2u
    #define GUIDEWIRE_CENTER_ADC_CHANNEL      3u
    #define GUIDEWIRE_RIGHT_ADC_CHANNEL       5u

    #define GUIDEWIRE_SAMPLE_COUNT            16u
    #define GUIDEWIRE_TRACK_MIN_SIGNAL        70u
    #define GUIDEWIRE_BALANCE_WINDOW          300u
    #define GUIDEWIRE_INTERSECTION_CENTER_MIN 1500u
    #define GUIDEWIRE_INTERSECTION_SIDE_MIN   800u
    #define GUIDEWIRE_CENTER_BIAS             0

    #define GUIDEWIRE_LEFT_OFFSET             56u
    #define GUIDEWIRE_CENTER_OFFSET           24u
    #define GUIDEWIRE_RIGHT_OFFSET            23u

    // New turning thresholds:
    // |diff| <= BALANCE_WINDOW           -> straight
    // BALANCE_WINDOW < |diff| <= SOFT    -> gentle correction
    // |diff| > HARD                      -> stronger correction
    #define GUIDEWIRE_SOFT_TURN_MARGIN    200u
    #define GUIDEWIRE_HARD_TURN_MARGIN    750u

    // Motor pulse timing (tune these if needed)
    #define FORWARD_MS                    10u
    #define SOFT_TURN_FORWARD_MS          25u
    #define SOFT_TURN_PULSE_MS            4u
    #define HARD_TURN_PULSE_MS            4u
    #define STOP_MS                       20u
    #define INTERSECTION_REARM_LOOPS      120
    #define INTERSECTION_CONFIRM_COUNT    2
    #define INTERSECTION_CENTER_MARGIN    100u

    // Change this to 0, 1, or 2 to select which path
    #define ACTIVE_PATH      0

    #define PATH_FORWARD     0
    #define PATH_LEFT        1
    #define PATH_RIGHT       2
    #define PATH_STOP        3

    static const int paths[3][8] = {
        { PATH_FORWARD, PATH_LEFT,  PATH_LEFT,  PATH_FORWARD, PATH_RIGHT, PATH_LEFT,  PATH_RIGHT, PATH_STOP },
        { PATH_LEFT,    PATH_RIGHT, PATH_LEFT,  PATH_RIGHT,   PATH_FORWARD,PATH_FORWARD,PATH_STOP, PATH_STOP },
        { PATH_RIGHT,   PATH_FORWARD,PATH_RIGHT,PATH_LEFT,    PATH_RIGHT, PATH_LEFT,  PATH_FORWARD,PATH_STOP }
    };

    static int intersection_count = 0;
    static int intersection_cooldown = 0;
    static int startup_mode = 1;
    static int intersection_confirm = 0;
    static bool recovery_mode = false; // Enforces soft-steering until robot is centered

    typedef enum
    {
        GUIDEWIRE_STATE_STOP = 0,
        GUIDEWIRE_STATE_FORWARD,
        GUIDEWIRE_STATE_LEFT,
        GUIDEWIRE_STATE_RIGHT
    } guidewire_state_t;

    typedef struct
    {
        uint16_t left;
        uint16_t center;
        uint16_t right;
    } guidewire_readings_t;

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

    static void guidewire_motor_soft_left(void)
    {
        // Gentle left: Right motor forward, Left motor off (coasts)
        GPIOA->ODR &= ~(LM1 | LM2 | RM1 | RM2);
        GPIOA->ODR |= RM1;
    }

    static void guidewire_motor_soft_right(void)
    {
        // Gentle right: Left motor forward, Right motor off (coasts)
        GPIOA->ODR &= ~(LM1 | LM2 | RM1 | RM2);
        GPIOA->ODR |= LM1;
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
        RCC->IOPENR |= BIT0;

        guidewire_gpio_pin_mode_analog(GPIOA, 2u);
        guidewire_gpio_pin_mode_analog(GPIOA, 3u);
        guidewire_gpio_pin_mode_analog(GPIOA, 5u);

        if ((ADC1->CR & ADC_CR_ADEN) != 0u)
        {
            ADC1->CR |= ADC_CR_ADDIS;
            wait(1);
        }

        ADC1->CFGR1 = 0u;
        ADC1->CFGR2 = ADC_CFGR2_CKMODE_0;
        ADC1->SMPR  = ADC_SMPR_SMP;
        ADC1->CHSELR = 0u;

        ADC1->CR |= ADC_CR_ADVREGEN;
        wait(1);

        ADC1->CR |= ADC_CR_ADCAL;
        while ((ADC1->CR & ADC_CR_ADCAL) != 0u)
        {
        }

        ADC1->ISR = ADC_ISR_ADRDY | ADC_ISR_EOC | ADC_ISR_EOSEQ | ADC_ISR_OVR;
        ADC1->CR |= ADC_CR_ADEN;
        guidewire_adc_wait_for_flag(ADC_ISR_ADRDY);

        guidewire_motor_stop();
    }

    uint16_t guidewire_read_adc(uint8_t channel)
    {
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

    guidewire_readings_t guidewire_read_sensors(void)
    {
        guidewire_readings_t readings;

        uint16_t raw_left   = guidewire_read_filtered(GUIDEWIRE_LEFT_ADC_CHANNEL);
        uint16_t raw_center = guidewire_read_filtered(GUIDEWIRE_CENTER_ADC_CHANNEL);
        uint16_t raw_right  = guidewire_read_filtered(GUIDEWIRE_RIGHT_ADC_CHANNEL);

        readings.left   = (raw_left   > GUIDEWIRE_LEFT_OFFSET)   ? (raw_left   - GUIDEWIRE_LEFT_OFFSET)   : 0u;
        readings.center = (raw_center > GUIDEWIRE_CENTER_OFFSET) ? (raw_center - GUIDEWIRE_CENTER_OFFSET) : 0u;
        readings.right  = (raw_right  > GUIDEWIRE_RIGHT_OFFSET)  ? (raw_right  - GUIDEWIRE_RIGHT_OFFSET)  : 0u;

        return readings;
    }

    void handle_intersection(void)
    {
        if (intersection_count >= 8)
        {
            guidewire_motor_stop();
            return;
        }

        int action = paths[ACTIVE_PATH][intersection_count];
        intersection_count++;

        switch (action)
        {
			case PATH_FORWARD:
  {
      guidewire_motor_forward();
      wait(60);

      for (int i = 0; i < 25; i++)
      {
          guidewire_readings_t ex = guidewire_read_sensors();
          int32_t ex_diff = (int32_t)ex.left - (int32_t)ex.right;

          if (ex_diff > (int32_t)GUIDEWIRE_BALANCE_WINDOW)
              guidewire_motor_soft_left();
          else if (ex_diff < -(int32_t)GUIDEWIRE_BALANCE_WINDOW)
              guidewire_motor_soft_right();
          else
              guidewire_motor_forward();

          wait(8);
      }
      recovery_mode = false;
      break;
  }
             case PATH_LEFT:
          guidewire_motor_left();
          wait(120);    // ~90 degree spin
          guidewire_motor_forward();
          wait(40);    // Brief push forward onto new line
          recovery_mode = true;
          break;

        case PATH_RIGHT:
            guidewire_motor_right();
            wait(120);    // ~90 degree spin
            guidewire_motor_forward();
            wait(70);    // Brief push forward onto new line
            recovery_mode = true;
            break;

              case PATH_STOP:
              default:
                  guidewire_motor_stop();
                  while(1);
          }
      }

      int main(void)
      {
          Hardware_Init();
          guidewire_init();
          wait(500); // allow ADC + amplifier to stabilize

      intersection_count = 0;
      intersection_cooldown = 0;

          while (1)
          {
              guidewire_readings_t r = guidewire_read_sensors();
              int32_t diff = (int32_t)r.left - (int32_t)r.right + GUIDEWIRE_CENTER_BIAS;
              int32_t abs_diff = (diff >= 0) ? diff : -diff;
              uint16_t strongest_side = (r.left > r.right) ? r.left : r.right;

              printf("L:%4u C:%4u R:%4u D:%4ld\r\n",
                     r.left, r.center, r.right, diff);
              fflush(stdout);

              if (startup_mode)
              {
                  if (strongest_side < GUIDEWIRE_TRACK_MIN_SIGNAL)
                  {
                      guidewire_motor_left();   // sweep to find line
                      wait(10);
                      guidewire_motor_stop();
                      wait(10);
                      continue;
                  }
                  else if (abs_diff > (int32_t)GUIDEWIRE_BALANCE_WINDOW)
                  {
                      // Found the line, but crooked! Pulse slowly to center perfectly on the spot.
                      if (diff > 0) guidewire_motor_left();
                      else guidewire_motor_right();
                      
                      wait(2);  // Tiny pulse
                      guidewire_motor_stop();
                      wait(10); // Let momentum stop & sensors settle
                      continue;
                  }
                  
                  // Perfectly centered! Release control to normal forward line-tracking.
                  startup_mode = 0;
                  recovery_mode = false;
                  continue;
              }

              if (strongest_side < GUIDEWIRE_TRACK_MIN_SIGNAL)
              {
                  guidewire_motor_stop();
                  wait(50);
                  continue;
              }

              if (intersection_cooldown > 0)
              {
                  intersection_cooldown--;
                  intersection_confirm = 0;
              }
              else if ((r.center >= GUIDEWIRE_INTERSECTION_CENTER_MIN) &&
                       (r.left >= GUIDEWIRE_INTERSECTION_SIDE_MIN) &&
                       (r.right >= GUIDEWIRE_INTERSECTION_SIDE_MIN))
              {
                  intersection_confirm++;

                  if (intersection_confirm >= INTERSECTION_CONFIRM_COUNT)
                  {
                      handle_intersection();
                      intersection_cooldown = INTERSECTION_REARM_LOOPS;
                      intersection_confirm = 0;
                      continue;
                  }
              }
              else
              {
                  intersection_confirm = 0;
              }

              if (abs_diff <= (int32_t)GUIDEWIRE_BALANCE_WINDOW)
              {
                  guidewire_motor_forward();
                  wait(FORWARD_MS);
                  recovery_mode = false;  // Perfectly aligned, clear recovery mode
              }
              else if (diff > 0)
              {
                  if (abs_diff > (int32_t)GUIDEWIRE_HARD_TURN_MARGIN)
                  {
                      if (recovery_mode)
                      {
                          guidewire_motor_soft_left();
                          wait(SOFT_TURN_FORWARD_MS);
                      }
                      else
                      {
                          guidewire_motor_left();
                          wait(HARD_TURN_PULSE_MS);
                      }
                  }
                  else
                  {
                      guidewire_motor_soft_left();
                      wait(SOFT_TURN_FORWARD_MS);
                  }
              }
              else
              {
                  if (abs_diff > (int32_t)GUIDEWIRE_HARD_TURN_MARGIN)
                  {
                      if (recovery_mode)
                      {
                          guidewire_motor_soft_right();
                          wait(SOFT_TURN_FORWARD_MS);
                      }
                      else
                      {
                          guidewire_motor_right();
                          wait(HARD_TURN_PULSE_MS);
                      }
                  }
                  else
                  {
                      guidewire_motor_soft_right();
                      wait(SOFT_TURN_FORWARD_MS);
                  }
              }
          }
      }