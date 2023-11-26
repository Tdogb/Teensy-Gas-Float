#include "float-vesc.h"

float_vesc::float_vesc(/* args */)
{
    /** Setup UART port (Serial1 on Atmega32u4) */
    Serial1.begin(115200);

    /** Define which ports to use as UART */
    UART.setSerialPort(&Serial1);
}

void float_vesc::float_vesc_set_current(float current) {
    UART.setCurrent(current);
}

void float_vesc::float_vesc_set_brake_current(float current) {
    UART.setBrakeCurrent(current);
}

float float_vesc::float_vesc_get_motor_current() {
    return UART.data.avgMotorCurrent;
}

float float_vesc::float_vesc_get_motor_rpm() {
    return UART.data.rpm;
}

float float_vesc::float_vesc_get_vbus() {
    return UART.data.inpVoltage;
}