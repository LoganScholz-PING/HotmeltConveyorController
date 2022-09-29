#ifndef HOTMELT_CONVEYOR_H
#define HOTMELT_CONVEYOR_H

#include <Arduino.h>

enum MOTOR_STATE { RUNNING, STOPPED, NOT_INIT, INTERRUPT, JOGGING, ERROR };

// arduino uno pin definitions
// ~ inputs ~
// ~ Uno R3 interrupt enabled pins ~ 
#define HM_PIN_EMERGENCY_STOP      2 // level 0 interrupt // PD2
#define HM_PIN_PROXIMITY_INTERRUPT 3 // level 1 interrupt // PD3
#define HM_PIN_LIDAR_DATA_READY    4 // 
// ~ normal digital pins ~ 
// #define HM_PIN_START_MOTOR_BTN 5     // PD5
// #define PORTD_START_MOTOR_BTN_MASK 0B00100000
// #define HM_PIN_STOP_MOTOR_BTN  6     // PD6
// #define PORTD_STOP_MOTOR_BTN_MASK 0B01000000
// #define HM_PIN_JOG_MOTOR_BTN   7     // PD7
// #define PORTD_JOG_MOTOR_BTN_MASK 0B10000000

// ~ outputs ~
#define HM_PIN_MOTOR_ENABLE    9     // PB1
#define PORTB_MOTOR_OFF_MASK 0B11111101 // for ultra fast motor stop (motor must be attached to PB1 (digital pin 9))

#endif