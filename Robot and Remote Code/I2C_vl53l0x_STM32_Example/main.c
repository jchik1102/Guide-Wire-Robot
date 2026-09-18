// =============================================================================
//  Integrated Robot: Dual VL53L0X ToF Sensors + IR Remote Motor Control
//  Main entry point
// =============================================================================

#include <stdio.h>
#include "../Common/Include/serial.h"
#include "hardware.h"
#include "tof_sensors.h"
#include "ir_motor.h"

int main(void)
{
    VL53L0X_Dev tof1;
    VL53L0X_Dev tof2;
    bool tof1_ok = false;
    bool tof2_ok = false;
    bool read1_ok = false;
    bool read2_ok = false;
    uint16_t range1 = 0;
    uint16_t range2 = 0;

    waitms(500);

    printf("\x1b[2J\x1b[1;1H");
    printf("Integrated Robot: Dual VL53L0X + IR Motor Control\r\n");
    printf("File: %s  Compiled: %s, %s\r\n\r\n", __FILE__, __DATE__, __TIME__);

    // ---- Init ToF subsystem ----
    tof1.addr7 = TOF1_ADDR_DEFAULT;
    tof1.stop_variable = 0;
    tof2.addr7 = TOF1_ADDR_DEFAULT;
    tof2.stop_variable = 0;

    leds_init();
    xshut_init();
    I2C_init();

    xshut1_low();
    xshut2_low();
    waitms(50);

    xshut1_release();
    waitms(50);

    if (vl53l0x_device_is_booted(&tof1)) {
        if (vl53l0x_set_address(&tof1, TOF1_ADDR_NEW))
            tof1_ok = vl53l0x_init_device(&tof1);
    }
    printf(tof1_ok ? "TOF1 init OK at 0x%02X\r\n" : "TOF1 init FAILED\r\n", tof1.addr7);
    if (tof1_ok) print_ref_registers(&tof1, "TOF1");

    xshut2_release();
    waitms(50);

    if (vl53l0x_device_is_booted(&tof2)) {
        if (vl53l0x_set_address(&tof2, TOF2_ADDR_NEW))
            tof2_ok = vl53l0x_init_device(&tof2);
    }
    printf(tof2_ok ? "TOF2 init OK at 0x%02X\r\n" : "TOF2 init FAILED\r\n", tof2.addr7);
    if (tof2_ok) print_ref_registers(&tof2, "TOF2");

    // ---- Init IR + motors (after ToF so I2C setup is undisturbed) ----
    IR_Motor_Init();
    printf("IR + Motor init done\r\n\r\n");

    // ---- Main loop ----
    while (1)
    {
        // ToF readings + LED indicators
        read1_ok = tof1_ok ? vl53l0x_read_range_single_device(&tof1, &range1) : false;
        read2_ok = tof2_ok ? vl53l0x_read_range_single_device(&tof2, &range2) : false;

        if (read1_ok && (range1 < TOF1_LED_THRESHOLD_MM))
            led1_on();
        else
            led1_off();

        if (read2_ok && (range2 > TOF2_LED_THRESHOLD_MM))
            led2_on();
        else
            led2_off();

        // Serial output
        if (read1_ok && read2_ok)
            printf("D1:%4u mm  D2:%4u mm\r", range1, range2);
        else if (read1_ok)
            printf("D1:%4u mm  D2:ERR \r", range1);
        else if (read2_ok)
            printf("D1:ERR   D2:%4u mm\r", range2);
        else
            printf("D1:ERR   D2:ERR \r");
        fflush(stdout);

        // IR motor commands
        process_ir_command();
    }
}