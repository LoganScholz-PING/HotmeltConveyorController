#include <Arduino.h>
#include <SoftwareSerial.h> // for talking to nextion display
#include <EasyNextionLibrary.h> // from SRC folder - modified to use SoftwareSerial
#include "HotmeltConveyor.h"
#include "Heartbeat.h"
#include "NextionTriggerFunctions.h"

SoftwareSerial swSerial(11, 12); // nextion display will be connected to 11(RX) and 12(TX)
EasyNex myNex(swSerial);

MOTOR_STATE motor_state = NOT_INIT;

void intCallBack() {
  detachInterrupt(HM_PIN_EMERGENCY_STOP);
  detachInterrupt(HM_PIN_PROXIMITY_INTERRUPT);

  PORTB = PORTB & PORTB_MOTOR_OFF_MASK; // set motor enable pin false
  motor_state = INTERRUPT;

  attachInterrupt(digitalPinToInterrupt(HM_PIN_EMERGENCY_STOP),      intCallBack, FALLING);
  attachInterrupt(digitalPinToInterrupt(HM_PIN_PROXIMITY_INTERRUPT), intCallBack, FALLING);
}

void stopMotor() {
  // turn off the motor
  Serial.println(F("[INFO] STOP"));
  motor_state = STOPPED;
  digitalWrite(HM_PIN_MOTOR_ENABLE, LOW); // turn off motor
  myNex.writeStr("t1.txt", "MOTOR STATUS: STOPPED");
  last_button_press_time = millis();
}

void jogMotor() {
  // jog the conveyor forward jog_timer seconds
  Serial.println(F("[INFO] JOG"));
  myNex.writeStr("t1.txt", "MOTOR STATUS: JOGGING");
  motor_state = JOGGING;
  digitalWrite(HM_PIN_MOTOR_ENABLE, HIGH); // turn off motor
  unsigned long start_time = millis();
  while(millis() - start_time <= jog_timer) { /* move motor for jog_timer seconds */ }
  stopMotor();
  last_button_press_time = millis();
}

void startMotor() {
  // start button pressed
  Serial.println(F("[INFO] START"));
  motor_state = RUNNING;
  digitalWrite(HM_PIN_MOTOR_ENABLE, HIGH);
  myNex.writeStr("t1.txt", "MOTOR STATUS: STARTED");
  last_button_press_time = millis();
}

void checkForPhysicalButtonPress() {
  byte port_d_contents = PORTD;
  pinput_start = !(port_d_contents & PORTD_START_MOTOR_BTN_MASK); // these are INPUT_PULLUP so false = pressed
  pinput_stop  = !(port_d_contents & PORTD_STOP_MOTOR_BTN_MASK);
  pinput_jog   = !(port_d_contents & PORTD_JOG_MOTOR_BTN_MASK);

  if((millis() - last_button_press_time) >= button_debounce_delay) {
    // there is a priority system here: 1-stop 2-jog 3-start
    if((pinput_stop != pinput_stop_previous)        && pinput_stop)  { stopMotor();  }
    else if((pinput_jog != pinput_jog_previous)     && pinput_jog)   { jogMotor();   }
    else if((pinput_start != pinput_start_previous) && pinput_start) { startMotor(); }
  }

  pinput_start_previous = pinput_start;
  pinput_stop_previous  = pinput_stop;
  pinput_jog_previous   = pinput_jog;
}

void initializePins() {
  // interrupt inputs (INPUT_PULLUP @ CHIP LEVEL)
  pinMode(HM_PIN_EMERGENCY_STOP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HM_PIN_EMERGENCY_STOP), intCallBack, FALLING);
  pinMode(HM_PIN_PROXIMITY_INTERRUPT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HM_PIN_PROXIMITY_INTERRUPT), intCallBack, FALLING);
  
  // normal inputs
  pinMode(HM_PIN_START_MOTOR_BTN, INPUT_PULLUP);
  pinMode(HM_PIN_STOP_MOTOR_BTN, INPUT_PULLUP);
  pinMode(HM_PIN_JOG_MOTOR_BTN, INPUT_PULLUP);

  // outputs
  pinMode(HM_PIN_MOTOR_ENABLE, OUTPUT);
  digitalWrite(HM_PIN_MOTOR_ENABLE, LOW); // turn motor off to start
  motor_state = NOT_INIT;
}

void setup() {
  Serial.begin(9600); // hardware serial
  while(!Serial) { /* hang out */ }

  myNex.begin(9600); // software serial port (pin 11(RX) pin 12(TX))

  initializePins(); // from HotmeltConveyor.h
  myNex.writeStr("t1.txt", "INITIALIZED");
  lastHeartBeatTime = millis();
}

void loop() {
  if(!HM_PIN_EMERGENCY_STOP) { motor_state = ERROR; } // emergency stop is pressed
  else {
    myNex.NextionListen(); // check for digital button presses
    checkForPhysicalButtonPress(); // check for physical button presses (these take priority)
  }
  
  HeartBeat();
}