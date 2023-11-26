#ifndef FLOAT_IMU
#define FLOAT_IMU
//
// MPU-9250 Mahony AHRS  S.J. Remington 3/2020
// last update 12/17/2020
// added full matrix calibration for accel and mag
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
// ***Standard orientation defined by gyro/accel: X North Y West Z Up***

// VERY VERY IMPORTANT!
// Both the accelerometer and magnetometer MUST be properly calibrated for this program to work, and the gyro offset must be determned.
// Follow the procedure described in http://sailboatinstruments.blogspot.com/2011/08/improved-magnetometer-calibration.html
// or in more detail, the tutorial https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/
//
// To collect data for calibration, use the companion programs MPU9250_cal and Magneto 1.2 from sailboatinstruments.blogspot.com
//
// For correcting the data, below I use the diagonal element of matrix A and ignore
// the off diagonal components. If those terms are large, (most likely only for the magnetometer)
// add them in to the corrections in function get_MPU_scaled()
//
// This version must be compiled with library routines in subfolder "libs"
#include "Wire.h"
// I2Cdev and MPU9250 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU9250.h"

class float_imu
{
private:
    void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);
    float vector_dot(float a[3], float b[3]);
    void vector_normalize(float a[3]);
    MPU9250 accelgyro;
    I2Cdev I2C_M;
    // vvvvvvvvvvvvvvvvvv  VERY VERY IMPORTANT vvvvvvvvvvvvvvvvvvvvvvvvvvvvv
    //These are the previously determined offsets and scale factors for accelerometer and magnetometer, using MPU9250_cal and Magneto 1.2
    //The AHRS will NOT work well or at all if these are not correct
    //
    // redetermined 12/16/2020
    //acel offsets and correction matrix
    float A_B[3] {  539.75,  218.36,  834.53}; 
    float A_Ainv[3][3]
    {{  0.51280,  0.00230,  0.00202},
    {  0.00230,  0.51348, -0.00126},
    {  0.00202, -0.00126,  0.50368}};

    // mag offsets and correction matrix
    float M_B[3]
    {   18.15,   28.05,  -36.09};
    float M_Ainv[3][3]
    {{  0.68093,  0.00084,  0.00923},
    {  0.00084,  0.69281,  0.00103},
    {  0.00923,  0.00103,  0.64073}};

    float G_off[3] = {-299.7, 113.2, 202.4}; //raw offsets, determined for gyro at rest
    // ^^^^^^^^^^^^^^^^^^^ VERY VERY IMPORTANT ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    char s[60]; //snprintf buffer
    //raw data and scaled as vector
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t mx, my, mz;
    float Axyz[3];
    float Mxyz[3];
    #define gscale (250./32768.0)*(PI/180.0)  //gyro default 250 LSB per d/s -> rad/s

    // NOW USING MAHONY FILTER

    // These are the free parameters in the Mahony filter and fusion scheme,
    // Kp for proportional feedback, Ki for integral
    // with MPU-9250, angles start oscillating at Kp=40. Ki does not seem to help and is not required.
    #define Kp 30.0
    #define Ki 0.0

    // globals for AHRS loop timing

    unsigned long now = 0, last = 0; //micros() timers
    float deltat = 0;  //loop time in seconds
    unsigned long now_ms, last_ms = 0; //millis() timers
    unsigned long print_ms = 50; //print every "print_ms" milliseconds


    // Vector to hold quaternion
    float q[4] = {1.0, 0.0, 0.0, 0.0};
public:
    float_imu(/* args */);
    // class default I2C address is 0x68
    // specific I2C addresses may be passed as a parameter here
    // AD0 low = 0x68 (default for Sparkfun module)
    // AD0 high = 0x69
    // MAHONY FILTER SELECTED BELOW
    void get_MPU_scaled(void);
    void float_imu_update(void);
    float yaw, pitch, roll; //Euler angle output
    float Gxyz[3];
};

#endif