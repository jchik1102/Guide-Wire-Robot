
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "../Common/Include/stm32l051xx.h"
#include "tof_sensors.h"

#define SYSCLK    32000000L
#define TICK_FREQ 1000L
#define BAUDRATE  115200L

// ---------------- Motors ----------------
#define LM1 BIT7    // PA7
#define LM2 BIT8    // PA8
#define RM1 BIT13   // PA13
#define RM2 BIT14   // PA14

// ---------------- Guidewire ADC inputs ----------------
// PA2 -> ADC2 left
// PA3 -> ADC3 center
// PA5 -> ADC5 right
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

#define GUIDEWIRE_SOFT_TURN_MARGIN        200u
#define GUIDEWIRE_HARD_TURN_MARGIN        750u

#define FORWARD_MS                        10u
#define SOFT_TURN_FORWARD_MS              25u
#define HARD_TURN_PULSE_MS                4u
#define INTERSECTION_REARM_LOOPS          120
#define INTERSECTION_CONFIRM_COUNT        2

#define PATH_FORWARD     0
#define PATH_LEFT        1
#define PATH_RIGHT       2
#define PATH_STOP        3

// ---------------- IR receiver ----------------
#define IR_CMD_STOP        0x0
#define IR_CMD_FORWARD     0x1
#define IR_CMD_BACKWARD    0x2
#define IR_CMD_LEFT        0x3
#define IR_CMD_RIGHT       0x4
#define IR_CMD_MODE        0x5 // not sent by current remote
#define IR_CMD_PATH1       0x6
#define IR_CMD_PATH2       0x7
#define IR_CMD_PATH3       0x8
#define IR_CMD_CLOSE       0x9
#define IR_CMD_VERY_CLOSE  0xA
#define IR_CMD_HORN        0xB
#define IR_CMD_180         0xC
#define IR_CMD_CLAP_MODE   0xE
#define IR_MANUAL_HOLD_MS  180u
#define IR_CMD_FAN_TOGGLE  0xF

// ---------------- Learning mode ----------------
#define LEARNING_RECORD_MS                20000u
#define LEARNING_MAX_EVENTS               256u

// ---------------- Noise mode ----------------
// Mic mapping confirmed by user:
// PA0 = back, PA1 = front, PA6 = left
#define MIC_BACK_ADC_CHANNEL              0u
#define MIC_FRONT_ADC_CHANNEL             1u
#define MIC_LEFT_ADC_CHANNEL              6u
#define MIC_MIDPOINT                      2048
#define CLAP_THRESHOLD                    1400
#define CLAP_DEBOUNCE_MS                  200u
#define CLAP_WINDOW_MS                    1500u
#define NOISE_BURST_MS                    1000u
#define ONE_CLAP_CAPTURE_MS                30u
#define RIGHT_TURN_PREP_MS                550u
#define ONE_CLAP_DOMINANCE_MARGIN         220
#define ONE_CLAP_SIMILAR_MARGIN           260
#define ONE_CLAP_RIGHT_LEFT_MARGIN        250

typedef enum
{
    MANUAL_CMD_NONE = 0,
    MANUAL_CMD_FWD,
    MANUAL_CMD_BACK,
    MANUAL_CMD_LEFT,
    MANUAL_CMD_RIGHT,
    MANUAL_CMD_STOP
} manual_cmd_t;

typedef struct
{
    uint16_t time_ms;
    manual_cmd_t cmd;
} learning_event_t;

typedef struct
{
    uint16_t left;
    uint16_t center;
    uint16_t right;
} guidewire_readings_t;

typedef enum
{
    MODE_IDLE = 0,
    MODE_MANUAL,
    MODE_GUIDEWIRE,
    MODE_LEARNING,
    MODE_NOISE
} robot_mode_t;

typedef enum
{
    LEARNING_STATE_IDLE = 0,
    LEARNING_STATE_RECORD_JOYSTICK,
    LEARNING_STATE_REPLAY_JOYSTICK
} learning_state_t;

typedef enum
{
    NOISE_ONE_UNKNOWN = 0,
    NOISE_ONE_FRONT,
    NOISE_ONE_BACK,
    NOISE_ONE_LEFT,
    NOISE_ONE_RIGHT
} noise_one_clap_dir_t;

typedef enum
{
    NOISE_SEEK_NONE = 0,
    NOISE_SEEK_FORWARD,
    NOISE_SEEK_RIGHT_TURN,
    NOISE_SEEK_RIGHT_FORWARD
} noise_seek_state_t;

static const int paths[3][8] = {
    { PATH_FORWARD, PATH_LEFT,  PATH_LEFT,  PATH_FORWARD, PATH_RIGHT, PATH_LEFT,  PATH_RIGHT, PATH_STOP },
    { PATH_LEFT,    PATH_RIGHT, PATH_LEFT,  PATH_RIGHT,   PATH_FORWARD, PATH_FORWARD, PATH_STOP, PATH_STOP },
    { PATH_RIGHT,   PATH_FORWARD, PATH_RIGHT, PATH_LEFT,  PATH_RIGHT, PATH_LEFT,  PATH_FORWARD, PATH_STOP }
};

// ---------------- Global state ----------------
volatile uint32_t msTicks = 0;

static robot_mode_t current_mode = MODE_IDLE;

// Manual / guidewire
static manual_cmd_t manual_cmd = MANUAL_CMD_STOP;
static int active_path = 0;
static int intersection_count = 0;
static int intersection_cooldown = 0;
static int startup_mode = 1;
static int intersection_confirm = 0;
static bool recovery_mode = false;
static volatile bool guidewire_abort_requested = false;

// ToF
static VL53L0X_Dev tof_front = { TOF_ADDR, 0 };
static bool tof_ok = false;
static bool manual_tof_block = false;

// Learning
static learning_state_t learning_state = LEARNING_STATE_IDLE;
static learning_event_t learning_events[LEARNING_MAX_EVENTS];
static uint16_t learning_event_count = 0;
static bool learning_run_valid = false;
static uint32_t learning_start_time = 0;
static uint16_t learning_run_duration = 0;
static uint16_t learning_replay_index = 0;
static manual_cmd_t learning_live_cmd = MANUAL_CMD_STOP;

// Noise
static int clap_count = 0;
static uint32_t first_clap_time = 0;
static uint32_t last_clap_time = 0;
static int saved_back = 0;
static int saved_front = 0;
static int saved_left = 0;
static uint32_t noise_motion_end_time = 0;
static noise_seek_state_t noise_seek_state = NOISE_SEEK_NONE;
static uint32_t noise_seek_stage_start = 0;

// IR decode globals
volatile uint16_t StartTime = 0;
volatile uint16_t MeasuredWidth = 0;
volatile int STATE = 0;
volatile int BitCount = 0;
volatile int ir_command = 0;
volatile int NewData_flag = 0;

// Guidewire path state
static bool guidewire_has_active_path = false;
static bool intersection_latched = false;
static int intersection_clear_confirm = 0;

// Non-blocking special action state
typedef enum
{
    ACTION_NONE = 0,
    ACTION_IR_TURN_180
} special_action_t;

static special_action_t special_action = ACTION_NONE;
static uint32_t special_action_end_time = 0;
static bool ir_manual_active = false;
static uint32_t ir_manual_hold_until = 0;

// ---------------- Time / delay ----------------
void SysTick_Handler(void)
{
    msTicks++;
}

static unsigned int millis(void)
{
    return msTicks;
}

static void wait(int ms)
{
    for (volatile int i = 0; i < ms * (SYSCLK / TICK_FREQ) / 5; i++);
}

// ---------------- UART / Bluetooth ----------------
static void UART1_Init(void)
{
    RCC->IOPENR  |= BIT0;
    RCC->APB2ENR |= BIT14;

    // PA9 TX, PA10 RX, AF4
    GPIOA->MODER &= ~((3u << (9u * 2u)) | (3u << (10u * 2u)));
    GPIOA->MODER |=  ((2u << (9u * 2u)) | (2u << (10u * 2u)));

    GPIOA->PUPDR &= ~((3u << (9u * 2u)) | (3u << (10u * 2u)));
    GPIOA->PUPDR |=  ((1u << (9u * 2u)) | (1u << (10u * 2u)));

    GPIOA->AFR[1] &= ~((0xFu << 4) | (0xFu << 8));
    GPIOA->AFR[1] |=  ((4u << 4) | (4u << 8));

    USART1->CR1 = 0;
    USART1->CR2 = 0;
    USART1->CR3 = 0;
    USART1->BRR = (SYSCLK / BAUDRATE);
    USART1->CR1 |= BIT3 | BIT2 | BIT0;
}

static void clear_uart_errors(void)
{
    if(USART1->ISR & (BIT3 | BIT2 | BIT1 | BIT0))
    {
        USART1->ICR = BIT0 | BIT1 | BIT2 | BIT3;
        while(USART1->ISR & BIT5)
        {
            volatile unsigned int dummy = USART1->RDR;
            (void)dummy;
        }
    }
}

static bool uart_byte_available(void)
{
    return ((USART1->ISR & BIT5) != 0);
}

static char uart_read_byte(void)
{
    return (char)USART1->RDR;
}

static int is_ignored_char(char c)
{
    return (c == '\r' || c == '\n' || c == ' ' || c == '\t');
}

// ---------------- Hardware / motors ----------------
static void motor_gpio_init(void)
{
    RCC->IOPENR |= BIT0;

    GPIOA->MODER &= ~((3u << (7u * 2u)) | (3u << (8u * 2u)) |
                      (3u << (13u * 2u)) | (3u << (14u * 2u)));
    GPIOA->MODER |=  ((1u << (7u * 2u)) | (1u << (8u * 2u)) |
                      (1u << (13u * 2u)) | (1u << (14u * 2u)));

    GPIOA->OTYPER &= ~(BIT7 | BIT8 | BIT13 | BIT14);
    GPIOA->PUPDR &= ~((3u << (7u * 2u)) | (3u << (8u * 2u)) |
                      (3u << (13u * 2u)) | (3u << (14u * 2u)));

    GPIOA->ODR &= ~(LM1 | LM2 | RM1 | RM2);
}

static void IR_Init(void)
{
    RCC->IOPENR |= BIT0;   // GPIOA clock
    RCC->APB1ENR |= BIT0;  // TIM2 clock

    // Working receive path for this board:
    // PA15 alternate-function mode, old style that tested correctly.
    GPIOA->MODER = (GPIOA->MODER & ~(BIT30)) | (BIT31);
    GPIOA->AFR[1] |= (BIT30 | BIT28);

    GPIOA->PUPDR &= ~(3u << (15u * 2u));

    // Timer speed: 1 tick = 1 us
    TIM2->PSC = 31;
    TIM2->ARR = 0xFFFF;

    // Input capture on CH1, both edges
    TIM2->CCMR1 |= BIT0;
    TIM2->CCER  |= (BIT1 | BIT0 | BIT3);
    TIM2->DIER  |= BIT1;

    NVIC->ISER[0] |= BIT15;
    TIM2->CR1 |= BIT0;
    __enable_irq();
}

void TIM2_Handler(void)
{
    if (TIM2->SR & BIT1)
    {
        uint16_t currentCapture = TIM2->CCR1;

        if (!(GPIOA->IDR & BIT15))
        {
            StartTime = currentCapture;
        }
        else
        {
            MeasuredWidth = currentCapture - StartTime;

            // Current remote agreement:
            // header ~421 us, 0 ~210 us, 1 ~842 us, 4 bits total
            if ((MeasuredWidth > 300) && (MeasuredWidth < 600))
            {
                STATE = 1;
                BitCount = 0;
                ir_command = 0;
            }
            else if (STATE == 1)
            {
                if ((MeasuredWidth > 100) && (MeasuredWidth < 350))
                {
                    ir_command = (ir_command << 1);
                    BitCount++;
                }
                else if ((MeasuredWidth > 650) && (MeasuredWidth < 1100))
                {
                    ir_command = (ir_command << 1) | 1;
                    BitCount++;
                }
                else
                {
                    STATE = 0;
                }

                if (BitCount == 4)
                {
                    NewData_flag = 1;
                    STATE = 0;
                }
            }
        }

        TIM2->SR &= ~BIT1;
    }
}

static void manual_stop_all(void)
{
    GPIOA->ODR &= ~(LM1 | LM2 | RM1 | RM2);
}

static void motor_forward_trimmed(void)
{
    GPIOA->ODR &= ~(LM1 | LM2 | RM1 | RM2);
    GPIOA->ODR |= (LM1 | RM1);
}

static void drive_command_apply(manual_cmd_t cmd, bool respect_tof)
{
    bool blocked = (respect_tof && manual_tof_block);

    switch(cmd)
    {
        case MANUAL_CMD_BACK:
            GPIOA->ODR &= ~(LM1 | LM2 | RM1 | RM2);
            GPIOA->ODR |= (LM2 | RM2);
            break;

        case MANUAL_CMD_FWD:
            if(blocked) manual_stop_all();
            else
            {
                motor_forward_trimmed();
            }
            break;

        case MANUAL_CMD_LEFT:
            if(blocked) manual_stop_all();
            else
            {
                GPIOA->ODR &= ~(LM1 | LM2 | RM1 | RM2);
                GPIOA->ODR |= (LM2 | RM1);
            }
            break;

        case MANUAL_CMD_RIGHT:
            if(blocked) manual_stop_all();
            else
            {
                GPIOA->ODR &= ~(LM1 | LM2 | RM1 | RM2);
                GPIOA->ODR |= (LM1 | RM2);
            }
            break;

        case MANUAL_CMD_STOP:
        case MANUAL_CMD_NONE:
        default:
            manual_stop_all();
            break;
    }
}

// Guidewire-specific motor motions
static void guidewire_motor_stop(void)      { manual_stop_all(); }
static void guidewire_motor_forward(void)   { motor_forward_trimmed(); }
static void guidewire_motor_left(void)      { GPIOA->ODR = (GPIOA->ODR & ~(LM1|LM2|RM1|RM2)) | (LM2|RM1); }
static void guidewire_motor_right(void)     { GPIOA->ODR = (GPIOA->ODR & ~(LM1|LM2|RM1|RM2)) | (LM1|RM2); }
static void guidewire_motor_soft_left(void) { GPIOA->ODR = (GPIOA->ODR & ~(LM1|LM2|RM1|RM2)) | RM1; }
static void guidewire_motor_soft_right(void){ GPIOA->ODR = (GPIOA->ODR & ~(LM1|LM2|RM1|RM2)) | LM1; }

static void start_ir_turn_180(void)
{
    current_mode = MODE_MANUAL;
    manual_cmd = MANUAL_CMD_STOP;
    special_action = ACTION_IR_TURN_180;
    special_action_end_time = millis() + 650u;
    GPIOA->ODR &= ~(LM1 | LM2 | RM1 | RM2);
    GPIOA->ODR |= (LM1 | RM2);
}

static void service_special_action(void)
{
    if (special_action == ACTION_NONE) return;

    if (special_action == ACTION_IR_TURN_180)
    {
        if ((int32_t)(millis() - special_action_end_time) >= 0)
        {
            manual_stop_all();
            special_action = ACTION_NONE;
            manual_cmd = MANUAL_CMD_STOP;
        }
    }
}

static void cancel_ir_manual_hold(void)
{
    ir_manual_active = false;
    ir_manual_hold_until = 0;
}

static void refresh_ir_manual_hold(manual_cmd_t cmd)
{
    current_mode = MODE_MANUAL;
    manual_cmd = cmd;
    ir_manual_active = true;
    ir_manual_hold_until = millis() + IR_MANUAL_HOLD_MS;
}

static void service_ir_manual_hold(void)
{
    if (!ir_manual_active) return;
    if (current_mode != MODE_MANUAL) return;
    if (special_action != ACTION_NONE) return;

    if ((int32_t)(millis() - ir_manual_hold_until) >= 0)
    {
        manual_cmd = MANUAL_CMD_STOP;
        manual_stop_all();
        current_mode = MODE_IDLE;
        cancel_ir_manual_hold();
    }
}

// ---------------- ToF ----------------
static void update_manual_tof_block(void)
{
    if (current_mode != MODE_MANUAL)
    {
        manual_tof_block = false;
        return;
    }

    if (tof_ok) manual_tof_block = tof_safety_stop(&tof_front, tof_ok);
    else manual_tof_block = false;

    if (manual_tof_block) manual_stop_all();
}

static void apply_manual_command(void)
{
    if(current_mode != MODE_MANUAL) return;
    update_manual_tof_block();
    drive_command_apply(manual_cmd, true);
}

// ---------------- ADC shared for guidewire + noise ----------------
static void gpio_pin_mode_analog(GPIO_TypeDef *port, uint8_t pin)
{
    port->MODER |= (3u << (pin * 2u));
    port->PUPDR &= ~(3u << (pin * 2u));
}

static bool adc_wait_for_flag(uint32_t flag)
{
    uint32_t timeout = 200000u;
    while (((ADC1->ISR & flag) == 0u) && (timeout > 0u)) timeout--;
    return (timeout > 0u);
}

static void ADC_shared_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->IOPENR  |= BIT0;

    gpio_pin_mode_analog(GPIOA, 0u);
    gpio_pin_mode_analog(GPIOA, 1u);
    gpio_pin_mode_analog(GPIOA, 2u);
    gpio_pin_mode_analog(GPIOA, 3u);
    gpio_pin_mode_analog(GPIOA, 5u);
    gpio_pin_mode_analog(GPIOA, 6u);

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
    while ((ADC1->CR & ADC_CR_ADCAL) != 0u) {}

    ADC1->ISR = ADC_ISR_ADRDY | ADC_ISR_EOC | ADC_ISR_EOSEQ | ADC_ISR_OVR;
    ADC1->CR |= ADC_CR_ADEN;
    adc_wait_for_flag(ADC_ISR_ADRDY);
}

static uint32_t adc_channel_mask(uint8_t channel)
{
    return (uint32_t)1u << channel;
}

static uint16_t read_adc_channel(uint8_t channel)
{
    ADC1->CHSELR = adc_channel_mask(channel);
    ADC1->ISR = ADC_ISR_EOC | ADC_ISR_EOSEQ | ADC_ISR_OVR;
    ADC1->CR |= ADC_CR_ADSTART;
    if (!adc_wait_for_flag(ADC_ISR_EOC)) return 0u;
    return (uint16_t)(ADC1->DR & ADC_DR_DATA);
}

static uint16_t guidewire_read_filtered(uint8_t channel)
{
    uint32_t sum = 0u;
    for (uint8_t i = 0u; i < GUIDEWIRE_SAMPLE_COUNT; i++) sum += read_adc_channel(channel);
    return (uint16_t)(sum / GUIDEWIRE_SAMPLE_COUNT);
}

static guidewire_readings_t guidewire_read_sensors(void)
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

// ---------------- Learning mode ----------------
static void learning_clear_run(void)
{
    special_action = ACTION_NONE;
    learning_state = LEARNING_STATE_IDLE;
    learning_event_count = 0;
    learning_run_valid = false;
    learning_start_time = 0;
    learning_run_duration = 0;
    learning_replay_index = 0;
    learning_live_cmd = MANUAL_CMD_STOP;
    manual_stop_all();
}

static void learning_store_event(manual_cmd_t cmd)
{
    uint32_t elapsed;

    if (learning_state != LEARNING_STATE_RECORD_JOYSTICK) return;

    elapsed = millis() - learning_start_time;
    if (elapsed > LEARNING_RECORD_MS) elapsed = LEARNING_RECORD_MS;

    if ((learning_event_count > 0u) &&
        (learning_events[learning_event_count - 1u].cmd == cmd))
    {
        return;
    }

    if (learning_event_count < LEARNING_MAX_EVENTS)
    {
        learning_events[learning_event_count].time_ms = (uint16_t)elapsed;
        learning_events[learning_event_count].cmd = cmd;
        learning_event_count++;
    }
}

static void learning_start_record(void)
{
    special_action = ACTION_NONE;
    learning_state = LEARNING_STATE_RECORD_JOYSTICK;
    learning_event_count = 0;
    learning_run_valid = false;
    learning_run_duration = 0;
    learning_replay_index = 0;
    learning_live_cmd = MANUAL_CMD_STOP;
    learning_start_time = millis();
    manual_stop_all();

    learning_events[0].time_ms = 0u;
    learning_events[0].cmd = MANUAL_CMD_STOP;
    learning_event_count = 1u;
}

static void learning_finish_record(void)
{
    uint32_t elapsed = millis() - learning_start_time;
    if (elapsed > LEARNING_RECORD_MS) elapsed = LEARNING_RECORD_MS;

    learning_run_duration = (uint16_t)elapsed;

    if ((learning_event_count < LEARNING_MAX_EVENTS) &&
        (learning_events[learning_event_count - 1u].cmd != MANUAL_CMD_STOP))
    {
        learning_events[learning_event_count].time_ms = learning_run_duration;
        learning_events[learning_event_count].cmd = MANUAL_CMD_STOP;
        learning_event_count++;
    }

    manual_stop_all();
    learning_live_cmd = MANUAL_CMD_STOP;
    learning_state = LEARNING_STATE_IDLE;
    learning_run_valid = (learning_event_count > 0u);
    current_mode = MODE_LEARNING;
}

static void learning_start_replay(void)
{
    if (!learning_run_valid) return;

    special_action = ACTION_NONE;
    learning_state = LEARNING_STATE_REPLAY_JOYSTICK;
    learning_start_time = millis();
    if (learning_run_duration == 0u) learning_run_duration = LEARNING_RECORD_MS;
    learning_replay_index = 0;
    learning_live_cmd = MANUAL_CMD_STOP;
    manual_stop_all();
}

static void learning_service(void)
{
    uint32_t elapsed;

    if (current_mode != MODE_LEARNING) return;

    if (learning_state == LEARNING_STATE_RECORD_JOYSTICK)
    {
        elapsed = millis() - learning_start_time;
        if (elapsed >= LEARNING_RECORD_MS) learning_finish_record();
    }
    else if (learning_state == LEARNING_STATE_REPLAY_JOYSTICK)
    {
        elapsed = millis() - learning_start_time;

        while ((learning_replay_index < learning_event_count) &&
               (elapsed >= learning_events[learning_replay_index].time_ms))
        {
            learning_live_cmd = learning_events[learning_replay_index].cmd;
            drive_command_apply(learning_live_cmd, false);
            learning_replay_index++;
        }

        if ((learning_replay_index >= learning_event_count) || (elapsed >= learning_run_duration))
        {
            manual_stop_all();
            learning_live_cmd = MANUAL_CMD_STOP;
            learning_state = LEARNING_STATE_IDLE;
            current_mode = MODE_LEARNING;
        }
    }
}

// ---------------- Noise mode ----------------
static void noise_reset_runtime(void)
{
    clap_count = 0;
    first_clap_time = 0;
    last_clap_time = 0;
    saved_back = 0;
    saved_front = 0;
    saved_left = 0;
    noise_motion_end_time = 0;
    noise_seek_state = NOISE_SEEK_NONE;
    noise_seek_stage_start = 0;
    manual_stop_all();
}

static void noise_enter_mode(void)
{
    special_action = ACTION_NONE;
    current_mode = MODE_NOISE;
    noise_reset_runtime();
}

static void noise_exit_mode(void)
{
    noise_reset_runtime();
    current_mode = MODE_MANUAL;
    manual_cmd = MANUAL_CMD_STOP;
}

static int adc_amp(uint8_t channel)
{
    int raw = (int)read_adc_channel(channel);
    int amp = raw - MIC_MIDPOINT;
    return (amp < 0) ? -amp : amp;
}

static void capture_one_clap_peaks(int *back_peak, int *front_peak, int *left_peak)
{
    uint32_t start = millis();
    int b = 0, f = 0, l = 0;

    while ((millis() - start) < ONE_CLAP_CAPTURE_MS)
    {
        int vb = adc_amp(MIC_BACK_ADC_CHANNEL);
        int vf = adc_amp(MIC_FRONT_ADC_CHANNEL);
        int vl = adc_amp(MIC_LEFT_ADC_CHANNEL);

        if (vb > b) b = vb;
        if (vf > f) f = vf;
        if (vl > l) l = vl;
    }

    *back_peak = b;
    *front_peak = f;
    *left_peak = l;
}

static noise_one_clap_dir_t classify_one_clap(int front, int back, int left)
{
    int fb_diff = front - back;
    int fb_abs = (fb_diff < 0) ? -fb_diff : fb_diff;
    int max_all = front;
    if (back > max_all) max_all = back;
    if (left > max_all) max_all = left;

    // Reject weak/noisy cases
    if (max_all < (CLAP_THRESHOLD + 150))
    {
        return NOISE_ONE_UNKNOWN;
    }

    // Strong clear left
    if ((left > front + 320) && (left > back + 320))
    {
        return NOISE_ONE_LEFT;
    }

    // Strong clear front
    if ((front > left + 320) && (front > back + 260))
    {
        return NOISE_ONE_FRONT;
    }

    // Strong clear back
    if ((back > left + 320) && (back > front + 260))
    {
        return NOISE_ONE_BACK;
    }

    // Infer right: front and back similarly strong, left clearly weaker
    if ((fb_abs <= 220) &&
        (front > (CLAP_THRESHOLD + 180)) &&
        (back  > (CLAP_THRESHOLD + 180)) &&
        (left + 260 < front) &&
        (left + 260 < back))
    {
        return NOISE_ONE_RIGHT;
    }

    return NOISE_ONE_UNKNOWN;
}

static void noise_start_seek_forward(void)
{
    noise_seek_state = NOISE_SEEK_FORWARD;
    noise_seek_stage_start = millis();
    drive_command_apply(MANUAL_CMD_FWD, false);
}

static void noise_start_seek_right_then_forward(void)
{
    noise_seek_state = NOISE_SEEK_RIGHT_TURN;
    noise_seek_stage_start = millis();
    drive_command_apply(MANUAL_CMD_RIGHT, false);
}

static void noise_start_burst(manual_cmd_t cmd)
{
    noise_seek_state = NOISE_SEEK_NONE;
    noise_motion_end_time = millis() + NOISE_BURST_MS;
    drive_command_apply(cmd, false);
}

static void noise_service(void)
{
    int back_mic, front_mic, left_mic;
    int heard_clap;
    uint32_t now = millis();

    if (current_mode != MODE_NOISE) return;

    // Handle seek states first
    if (noise_seek_state == NOISE_SEEK_RIGHT_TURN)
    {
        if ((now - noise_seek_stage_start) >= RIGHT_TURN_PREP_MS)
        {
            noise_seek_state = NOISE_SEEK_RIGHT_FORWARD;
            noise_seek_stage_start = now;
            drive_command_apply(MANUAL_CMD_FWD, false);
        }
        return;
    }
    else if (noise_seek_state == NOISE_SEEK_RIGHT_FORWARD ||
             noise_seek_state == NOISE_SEEK_FORWARD)
    {
        uint16_t range = 0;
        if (tof_ok && vl53l0x_read_range_single_device(&tof_front, &range))
        {
            if (range < TOF_FRONT_STOP_MM)
            {
                manual_stop_all();
                noise_seek_state = NOISE_SEEK_NONE;
                last_clap_time = now;
                clap_count = 0;
                return;
            }
        }

        drive_command_apply(MANUAL_CMD_FWD, false);
        return;
    }

    // Timed bursts for 1-clap back/left and 2-4 clap actions
    if (noise_motion_end_time != 0)
    {
        if ((int32_t)(now - noise_motion_end_time) >= 0)
        {
            manual_stop_all();
            noise_motion_end_time = 0;
            last_clap_time = now;
            clap_count = 0;
        }
        return;
    }

    // Only listen when stopped
    if (GPIOA->ODR & (LM1 | LM2 | RM1 | RM2))
    {
        return;
    }

    back_mic  = adc_amp(MIC_BACK_ADC_CHANNEL);
    front_mic = adc_amp(MIC_FRONT_ADC_CHANNEL);
    left_mic  = adc_amp(MIC_LEFT_ADC_CHANNEL);

    heard_clap = (back_mic > CLAP_THRESHOLD ||
                  front_mic > CLAP_THRESHOLD ||
                  left_mic > CLAP_THRESHOLD);

    if (heard_clap && ((now - last_clap_time) > CLAP_DEBOUNCE_MS))
    {
        clap_count++;
        last_clap_time = now;

        if (clap_count == 1)
        {
            first_clap_time = now;
            capture_one_clap_peaks(&saved_back, &saved_front, &saved_left);
        }
    }

    if ((clap_count > 0) && ((now - first_clap_time) > CLAP_WINDOW_MS))
    {
        if (clap_count == 1)
        {
            noise_one_clap_dir_t dir = classify_one_clap(saved_front, saved_back, saved_left);

            switch(dir)
            {
                case NOISE_ONE_LEFT:
                case NOISE_ONE_FRONT:
                case NOISE_ONE_BACK:
                case NOISE_ONE_RIGHT:
                    noise_start_burst(MANUAL_CMD_RIGHT); // 1 clap now means turn 180 degrees
                    break;

                case NOISE_ONE_UNKNOWN:
                default:
                    clap_count = 0;
                    break;
            }
        }
        else if (clap_count == 2)
        {
            noise_start_burst(MANUAL_CMD_RIGHT);
        }
        else if (clap_count == 3)
        {
            noise_start_burst(MANUAL_CMD_LEFT);
        }
        else if (clap_count >= 4)
        {
            noise_start_burst(MANUAL_CMD_FWD);
        }

        if (noise_seek_state == NOISE_SEEK_NONE && noise_motion_end_time == 0)
        {
            // uncertain case did nothing
        }

        if (noise_seek_state != NOISE_SEEK_NONE || noise_motion_end_time != 0)
        {
            clap_count = 0;
        }
    }
}

// ---------------- Bluetooth handling ----------------
static void reset_guidewire_state(void)
{
    intersection_count = 0;
    intersection_cooldown = 0;
    startup_mode = 1;
    intersection_confirm = 0;
    recovery_mode = false;
    guidewire_abort_requested = false;
    guidewire_has_active_path = false;
    intersection_latched = false;
    intersection_clear_confirm = 0;
}

static void process_bluetooth(void)
{
    char rx;

    clear_uart_errors();

    while(uart_byte_available())
    {
        rx = uart_read_byte();

        if(is_ignored_char(rx)) continue;

        if(current_mode == MODE_GUIDEWIRE)
        {
            if((rx == 'S') || (rx == 's')) guidewire_abort_requested = true;
            continue;
        }

        if(current_mode == MODE_NOISE)
        {
            if((rx == 'S') || (rx == 's')) noise_exit_mode();
            continue;
        }

        switch(rx)
        {
            case 'F':
            case 'f':
                if(current_mode == MODE_LEARNING)
                {
                    if(learning_state == LEARNING_STATE_RECORD_JOYSTICK)
                    {
                        learning_store_event(MANUAL_CMD_FWD);
                        learning_live_cmd = MANUAL_CMD_FWD;
                        drive_command_apply(learning_live_cmd, false);
                    }
                    else if(learning_state == LEARNING_STATE_IDLE)
                    {
                        special_action = ACTION_NONE;
                        cancel_ir_manual_hold();
                        current_mode = MODE_MANUAL;
                        manual_cmd = MANUAL_CMD_FWD;
                    }
                }
                else
                {
                    special_action = ACTION_NONE;
                    cancel_ir_manual_hold();
                    current_mode = MODE_MANUAL;
                    manual_cmd = MANUAL_CMD_FWD;
                }
                break;

            case 'L':
            case 'l':
                if(current_mode == MODE_LEARNING)
                {
                    if(learning_state == LEARNING_STATE_RECORD_JOYSTICK)
                    {
                        learning_store_event(MANUAL_CMD_LEFT);
                        learning_live_cmd = MANUAL_CMD_LEFT;
                        drive_command_apply(learning_live_cmd, false);
                    }
                    else if(learning_state == LEARNING_STATE_IDLE)
                    {
                        special_action = ACTION_NONE;
                        cancel_ir_manual_hold();
                        current_mode = MODE_MANUAL;
                        manual_cmd = MANUAL_CMD_LEFT;
                    }
                }
                else
                {
                    special_action = ACTION_NONE;
                    cancel_ir_manual_hold();
                    current_mode = MODE_MANUAL;
                    manual_cmd = MANUAL_CMD_LEFT;
                }
                break;

            case 'R':
            case 'r':
                if(current_mode == MODE_LEARNING)
                {
                    if(learning_state == LEARNING_STATE_RECORD_JOYSTICK)
                    {
                        learning_store_event(MANUAL_CMD_RIGHT);
                        learning_live_cmd = MANUAL_CMD_RIGHT;
                        drive_command_apply(learning_live_cmd, false);
                    }
                    else if(learning_state == LEARNING_STATE_IDLE)
                    {
                        special_action = ACTION_NONE;
                        cancel_ir_manual_hold();
                        current_mode = MODE_MANUAL;
                        manual_cmd = MANUAL_CMD_RIGHT;
                    }
                }
                else
                {
                    special_action = ACTION_NONE;
                    cancel_ir_manual_hold();
                    current_mode = MODE_MANUAL;
                    manual_cmd = MANUAL_CMD_RIGHT;
                }
                break;

            case 'B':
            case 'b':
                if(current_mode == MODE_LEARNING)
                {
                    if(learning_state == LEARNING_STATE_RECORD_JOYSTICK)
                    {
                        learning_store_event(MANUAL_CMD_BACK);
                        learning_live_cmd = MANUAL_CMD_BACK;
                        drive_command_apply(learning_live_cmd, false);
                    }
                    else if(learning_state == LEARNING_STATE_IDLE)
                    {
                        special_action = ACTION_NONE;
                        cancel_ir_manual_hold();
                        current_mode = MODE_MANUAL;
                        manual_cmd = MANUAL_CMD_BACK;
                    }
                }
                else
                {
                    special_action = ACTION_NONE;
                    cancel_ir_manual_hold();
                    current_mode = MODE_MANUAL;
                    manual_cmd = MANUAL_CMD_BACK;
                }
                break;

            case 'S':
            case 's':
                if(current_mode == MODE_LEARNING)
                {
                    if (learning_state == LEARNING_STATE_RECORD_JOYSTICK)
                    {
                        learning_store_event(MANUAL_CMD_STOP);
                        learning_live_cmd = MANUAL_CMD_STOP;
                        drive_command_apply(MANUAL_CMD_STOP, false);
                    }
                    else if (learning_state == LEARNING_STATE_IDLE)
                    {
                        learning_live_cmd = MANUAL_CMD_STOP;
                        drive_command_apply(MANUAL_CMD_STOP, false);
                    }
                    // Ignore S during replay so idle app traffic cannot kill replay
                }
                else
                {
                    cancel_ir_manual_hold();
                    manual_cmd = MANUAL_CMD_STOP;
                    manual_stop_all();
                    current_mode = MODE_IDLE;
                }
                break;

            case '1':
                special_action = ACTION_NONE;
                cancel_ir_manual_hold();
                manual_stop_all();
                manual_cmd = MANUAL_CMD_STOP;
                reset_guidewire_state();
                active_path = 0;
                guidewire_has_active_path = true;
                current_mode = MODE_GUIDEWIRE;
                break;

            case '2':
                special_action = ACTION_NONE;
                cancel_ir_manual_hold();
                manual_stop_all();
                manual_cmd = MANUAL_CMD_STOP;
                reset_guidewire_state();
                active_path = 1;
                guidewire_has_active_path = true;
                current_mode = MODE_GUIDEWIRE;
                break;

            case '3':
                special_action = ACTION_NONE;
                cancel_ir_manual_hold();
                manual_stop_all();
                manual_cmd = MANUAL_CMD_STOP;
                reset_guidewire_state();
                active_path = 2;
                guidewire_has_active_path = true;
                current_mode = MODE_GUIDEWIRE;
                break;

            case 'E':
            case 'e':
                if(current_mode == MODE_LEARNING)
                {
                    learning_clear_run();
                    cancel_ir_manual_hold();
                    current_mode = MODE_IDLE;
                }
                else
                {
                    cancel_ir_manual_hold();
                    manual_stop_all();
                    manual_cmd = MANUAL_CMD_STOP;
                    current_mode = MODE_LEARNING;
                    learning_state = LEARNING_STATE_IDLE;
                    learning_live_cmd = MANUAL_CMD_STOP;
                }
                break;

            case 'T':
            case 't':
                if((current_mode == MODE_LEARNING) && (learning_state != LEARNING_STATE_REPLAY_JOYSTICK))
                    learning_start_record();
                break;

            case 'U':
            case 'u':
                if((current_mode == MODE_LEARNING) && (learning_state != LEARNING_STATE_REPLAY_JOYSTICK))
                    learning_start_record(); // same as T for now
                break;

            case 'P':
            case 'p':
                if((current_mode == MODE_LEARNING) && (learning_state == LEARNING_STATE_IDLE))
                    learning_start_replay();
                break;

            case 'N':
            case 'n':
                special_action = ACTION_NONE;
                cancel_ir_manual_hold();
                manual_stop_all();
                manual_cmd = MANUAL_CMD_STOP;
                noise_enter_mode();
                break;

            default:
                break;
        }
    }
}


static void process_ir(void)
{
    if (!NewData_flag)
        return;

    NewData_flag = 0;

    if (current_mode == MODE_LEARNING || current_mode == MODE_NOISE)
        return;

    switch(ir_command)
    {
        case IR_CMD_FORWARD:
            special_action = ACTION_NONE;
            refresh_ir_manual_hold(MANUAL_CMD_FWD);
            break;

        case IR_CMD_LEFT:
            special_action = ACTION_NONE;
            refresh_ir_manual_hold(MANUAL_CMD_LEFT);
            break;

        case IR_CMD_RIGHT:
            special_action = ACTION_NONE;
            refresh_ir_manual_hold(MANUAL_CMD_RIGHT);
            break;

        case IR_CMD_180:
            cancel_ir_manual_hold();
            start_ir_turn_180();
            break;

        case IR_CMD_CLAP_MODE:
            cancel_ir_manual_hold();
            special_action = ACTION_NONE;
            manual_stop_all();
            manual_cmd = MANUAL_CMD_STOP;
            noise_enter_mode();
            break;

        case IR_CMD_BACKWARD:
            special_action = ACTION_NONE;
            refresh_ir_manual_hold(MANUAL_CMD_BACK);
            break;

        case IR_CMD_STOP:
            cancel_ir_manual_hold();
            special_action = ACTION_NONE;
            manual_cmd = MANUAL_CMD_STOP;
            manual_stop_all();
            if (current_mode == MODE_GUIDEWIRE)
            {
                reset_guidewire_state();
            }
            current_mode = MODE_IDLE;
            break;

        case IR_CMD_MODE:
            cancel_ir_manual_hold();
            special_action = ACTION_NONE;
            manual_stop_all();
            manual_cmd = MANUAL_CMD_STOP;

            if (current_mode == MODE_GUIDEWIRE)
            {
                reset_guidewire_state();
                current_mode = MODE_MANUAL;
            }
            else
            {
                reset_guidewire_state();
                current_mode = MODE_GUIDEWIRE;
            }
            break;

        case IR_CMD_PATH1:
            cancel_ir_manual_hold();
            special_action = ACTION_NONE;
            manual_stop_all();
            manual_cmd = MANUAL_CMD_STOP;
            reset_guidewire_state();
            active_path = 0;
            guidewire_has_active_path = true;
            current_mode = MODE_GUIDEWIRE;
            break;

        case IR_CMD_PATH2:
            cancel_ir_manual_hold();
            special_action = ACTION_NONE;
            manual_stop_all();
            manual_cmd = MANUAL_CMD_STOP;
            reset_guidewire_state();
            active_path = 1;
            guidewire_has_active_path = true;
            current_mode = MODE_GUIDEWIRE;
            break;

        case IR_CMD_PATH3:
            cancel_ir_manual_hold();
            special_action = ACTION_NONE;
            manual_stop_all();
            manual_cmd = MANUAL_CMD_STOP;
            reset_guidewire_state();
            active_path = 2;
            guidewire_has_active_path = true;
            current_mode = MODE_GUIDEWIRE;
            break;

        default:
            break;
    }
}

// ---------------- Guidewire mode ----------------
static bool guidewire_wait_check_abort(int ms)
{
    int elapsed = 0;

    while(elapsed < ms)
    {
        wait(1);
        elapsed++;
        process_bluetooth();
        process_ir();

        if(guidewire_abort_requested)
        {
            guidewire_motor_stop();
            reset_guidewire_state();
            current_mode = MODE_IDLE;
            return true;
        }
    }
    return false;
}

static bool handle_intersection_runtime(void)
{
    int action;

    if (intersection_count >= 8)
    {
        guidewire_motor_stop();
        current_mode = MODE_IDLE;
        return true;
    }

    action = paths[active_path][intersection_count];
    intersection_count++;
    intersection_latched = true;
    intersection_clear_confirm = 0;

    switch (action)
    {
        case PATH_FORWARD:
        {
            int i;
            guidewire_motor_forward();
            if(guidewire_wait_check_abort(60)) return true;

            for (i = 0; i < 25; i++)
            {
                guidewire_readings_t ex = guidewire_read_sensors();
                int32_t ex_diff = (int32_t)ex.left - (int32_t)ex.right;

                if (ex_diff > (int32_t)GUIDEWIRE_BALANCE_WINDOW) guidewire_motor_soft_left();
                else if (ex_diff < -(int32_t)GUIDEWIRE_BALANCE_WINDOW) guidewire_motor_soft_right();
                else guidewire_motor_forward();

                if(guidewire_wait_check_abort(8)) return true;
            }

            recovery_mode = false;
            break;
        }

        case PATH_LEFT:
            guidewire_motor_left();
            if(guidewire_wait_check_abort(120)) return true;
            guidewire_motor_forward();
            if(guidewire_wait_check_abort(40)) return true;
            recovery_mode = true;
            break;

        case PATH_RIGHT:
            guidewire_motor_right();
            if(guidewire_wait_check_abort(120)) return true;
            guidewire_motor_forward();
            if(guidewire_wait_check_abort(70)) return true;
            recovery_mode = true;
            break;

        case PATH_STOP:
        default:
            guidewire_motor_stop();
            reset_guidewire_state();
            current_mode = MODE_IDLE;
            return true;
    }

    return false;
}

static void run_guidewire_mode(void)
{
    while(current_mode == MODE_GUIDEWIRE)
    {
        guidewire_readings_t r = guidewire_read_sensors();
        int32_t diff = (int32_t)r.left - (int32_t)r.right + GUIDEWIRE_CENTER_BIAS;
        int32_t abs_diff = (diff >= 0) ? diff : -diff;
        uint16_t strongest_side = (r.left > r.right) ? r.left : r.right;

        process_bluetooth();
        process_ir();

        if(guidewire_abort_requested)
        {
            guidewire_motor_stop();
            reset_guidewire_state();
            current_mode = MODE_IDLE;
            break;
        }

        if (!guidewire_has_active_path)
        {
            guidewire_motor_stop();
            wait(5);
            continue;
        }

        if (startup_mode)
        {
            startup_mode = 0;
            recovery_mode = false;
        }

        if (strongest_side < GUIDEWIRE_TRACK_MIN_SIGNAL)
        {
            guidewire_motor_stop();
            if(guidewire_wait_check_abort(50)) break;
            continue;
        }

        {
            bool is_intersection_now =
                (r.center >= GUIDEWIRE_INTERSECTION_CENTER_MIN) &&
                (r.left   >= GUIDEWIRE_INTERSECTION_SIDE_MIN) &&
                (r.right  >= GUIDEWIRE_INTERSECTION_SIDE_MIN);

            if (intersection_latched)
            {
                if (!is_intersection_now)
                {
                    intersection_clear_confirm++;
                    if (intersection_clear_confirm >= 4)
                    {
                        intersection_latched = false;
                        intersection_clear_confirm = 0;
                    }
                }
                else
                {
                    intersection_clear_confirm = 0;
                }
                intersection_confirm = 0;
            }
            else
            {
                if (intersection_cooldown > 0)
                {
                    intersection_cooldown--;
                    intersection_confirm = 0;
                }
                else if (is_intersection_now)
                {
                    intersection_confirm++;
                    if (intersection_confirm >= INTERSECTION_CONFIRM_COUNT)
                    {
                        intersection_latched = true;
                        intersection_clear_confirm = 0;

                        if(handle_intersection_runtime()) break;

                        intersection_cooldown = INTERSECTION_REARM_LOOPS;
                        intersection_confirm = 0;
                        continue;
                    }
                }
                else
                {
                    intersection_confirm = 0;
                }
            }
        }

        if (abs_diff <= (int32_t)GUIDEWIRE_BALANCE_WINDOW)
        {
            guidewire_motor_forward();
            if(guidewire_wait_check_abort(FORWARD_MS)) break;
            recovery_mode = false;
        }
        else if (diff > 0)
        {
            if (abs_diff > (int32_t)GUIDEWIRE_HARD_TURN_MARGIN)
            {
                if (recovery_mode)
                {
                    guidewire_motor_soft_left();
                    if(guidewire_wait_check_abort(SOFT_TURN_FORWARD_MS)) break;
                }
                else
                {
                    guidewire_motor_left();
                    if(guidewire_wait_check_abort(HARD_TURN_PULSE_MS)) break;
                }
            }
            else
            {
                guidewire_motor_soft_left();
                if(guidewire_wait_check_abort(SOFT_TURN_FORWARD_MS)) break;
            }
        }
        else
        {
            if (abs_diff > (int32_t)GUIDEWIRE_HARD_TURN_MARGIN)
            {
                if (recovery_mode)
                {
                    guidewire_motor_soft_right();
                    if(guidewire_wait_check_abort(SOFT_TURN_FORWARD_MS)) break;
                }
                else
                {
                    guidewire_motor_right();
                    if(guidewire_wait_check_abort(HARD_TURN_PULSE_MS)) break;
                }
            }
            else
            {
                guidewire_motor_soft_right();
                if(guidewire_wait_check_abort(SOFT_TURN_FORWARD_MS)) break;
            }
        }
    }
}

// ---------------- Main ----------------
int main(void)
{
    motor_gpio_init();
    IR_Init();
    UART1_Init();
    ADC_shared_init();

    // ToF init
    xshut_init();
    I2C_init();
    xshut_release();
    wait(10);
    tof_front.addr7 = TOF_ADDR;
    tof_ok = vl53l0x_init_device(&tof_front);

    SysTick->LOAD = (SYSCLK / 1000L) - 1L;
    SysTick->VAL  = 0;
    SysTick->CTRL = BIT2 | BIT1 | BIT0;

    wait(200);

    reset_guidewire_state();
    current_mode = MODE_IDLE;
    learning_clear_run();
    noise_reset_runtime();

    while (1)
    {
        if(current_mode == MODE_LEARNING)
        {
            process_bluetooth();
            learning_service();
        }
        else
        {
            process_bluetooth();
            process_ir();

            if(current_mode == MODE_NOISE)
            {
                noise_service();
            }
            else if(current_mode == MODE_GUIDEWIRE)
            {
                run_guidewire_mode();
            }
            else
            {
                service_special_action();
                service_ir_manual_hold();
                if((current_mode == MODE_MANUAL) && (special_action == ACTION_NONE))
                {
                    apply_manual_command();
                }
            }
        }
    }
}
