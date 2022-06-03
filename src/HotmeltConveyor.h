#ifndef HOTMELT_CONVEYOR_H
#define HOTMELT_CONVEYOR_H

#include <Arduino.h>

// arduino uno pin definitions
// ~ inputs ~
// ~ Uno R3 interrupt enabled pins ~ 
#define HM_PIN_EMERGENCY_STOP      2 // level 0 interrupt // PD2
#define HM_PIN_PROXIMITY_INTERRUPT 3 // level 1 interrupt // PD3
// ~ normal digital pins ~ 
#define HM_PIN_START_MOTOR_BTN 5     // PD5
#define PORTD_START_MOTOR_BTN_MASK 0B00100000
#define HM_PIN_STOP_MOTOR_BTN  6     // PD6
#define PORTD_STOP_MOTOR_BTN_MASK 0B01000000
#define HM_PIN_JOG_MOTOR_BTN   7     // PD7
#define PORTD_JOG_MOTOR_BTN_MASK 0B10000000

// ~ outputs ~
#define HM_PIN_MOTOR_ENABLE    9     // PB1
#define PORTB_MOTOR_OFF_MASK 0B11111101 // for ultra fast motor stop (motor must be attached to PB1 (digital pin 9))

// program state tracking variables
enum MOTOR_STATE { RUNNING, STOPPED, NOT_INIT, INTERRUPT, JOGGING, ERROR };

unsigned long lastHeartBeatTime = 0;
unsigned long heartBeatDelay    = 1000;

unsigned long last_button_press_time = 0;
unsigned long button_debounce_delay = 500;

unsigned long jog_timer = 2000;

// physical button tracking
bool pinput_start_previous   = false;
bool pinput_stop_previous    = false;
bool pinput_jog_previous     = false;
bool pinput_start            = false;
bool pinput_stop             = false;
bool pinput_jog              = false;
bool pinput_emergency_stop   = false;
bool pinput_proximity_sensor = false;

// nextion button tracking (may not need)
bool dinput_stop        = false;
bool dinput_start       = false;
bool dinput_mtr_reverse = false;
bool dinput_mtr_forward = false;
bool dinput_jog         = false;

#endif