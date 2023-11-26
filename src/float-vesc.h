/*
  Name:    getVescValues.ino
  Created: 19-08-2018
  Author:  SolidGeek
  Description:  This example is made using a Arduino Micro (Atmega32u4) that has a HardwareSerial port (Serial1) seperated from the Serial port. 
                A Arduino Nano or Uno that only has one Serial port will not be able to display the data returned.
*/
#include <Arduino.h>
#include <VescUart.h>

class float_vesc
{
private:
    VescUart UART;
public:
    float_vesc(/* args */);
    void float_vesc_init(void);
    void float_vesc_set_current(float current);
    void float_vesc_set_brake_current(float current);
    float float_vesc_get_motor_current(void);
    float float_vesc_get_motor_rpm(void);
    float float_vesc_get_vbus(void);
};