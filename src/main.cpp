#include <Arduino.h>
#include <SoftwareSerial.h> // for talking to nextion display
#include <EasyNextionLibrary.h> // from SRC folder - modified to use SoftwareSerial
#include "HotmeltConveyor.h"
#include "Heartbeat.h"
#include "NextionTriggerFunctions.h"

// object instantiations
SoftwareSerial swSerial(11, 12); // nextion display will be connected to 11(RX) and 12(TX)
EasyNex myNex(swSerial);

// program state tracking variables
MOTOR_STATE motor_state = MOTOR_STATE::NOT_INIT;
bool MOTOR_OFF = false;
bool MOTOR_ON  = true;

bool prox_interrupt_triggered = false;
bool estop_interrupt_triggered = false;

#define TURN_MOTOR_OFF digitalWrite(HM_PIN_MOTOR_ENABLE, true)
#define TURN_MOTOR_ON  digitalWrite(HM_PIN_MOTOR_ENABLE, false);

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

void intCallBackESTOP() {
  //PORTB = PORTB & PORTB_MOTOR_OFF_MASK; // set motor enable pin false
  TURN_MOTOR_OFF;
  estop_interrupt_triggered = true;
  motor_state = INTERRUPT;
}

void intCallBackPROX() {
  //PORTB = PORTB & PORTB_MOTOR_OFF_MASK; // set motor enable pin false
  TURN_MOTOR_OFF;
  prox_interrupt_triggered = true;
  motor_state = INTERRUPT;
}

void stopMotor() {
  // turn off the motor
  Serial.println(F("[INFO] STOP"));
  motor_state = STOPPED;
  TURN_MOTOR_OFF; // turn off motor
  myNex.writeStr("t1.txt", "MOTOR STATUS: STOPPED");
  last_button_press_time = millis();
}

void jogMotor() {
  // jog the conveyor forward jog_timer seconds
  Serial.println(F("[INFO] JOG"));
  myNex.writeStr("t1.txt", "MOTOR STATUS: JOGGING");
  motor_state = JOGGING;
  TURN_MOTOR_ON;
  unsigned long start_time = millis();
  while(millis() - start_time <= jog_timer) { /* move motor for jog_timer seconds */ }
  stopMotor();
  last_button_press_time = millis();
}

void startMotor() {
  // start button pressed
  Serial.println(F("[INFO] START"));
  motor_state = RUNNING;
  TURN_MOTOR_ON;
  myNex.writeStr("t1.txt", "MOTOR STATUS: STARTED");
  last_button_press_time = millis();
}

void checkForPhysicalButtonPress() {
  // INPUTS WILL ONLY BE POLED FROM THE NEXTION DISPLAY!

  // byte port_d_contents = PORTD;
  // pinput_start = !(port_d_contents & PORTD_START_MOTOR_BTN_MASK); // these are INPUT_PULLUP so false = pressed
  // pinput_stop  = !(port_d_contents & PORTD_STOP_MOTOR_BTN_MASK);
  // pinput_jog   = !(port_d_contents & PORTD_JOG_MOTOR_BTN_MASK);

  // if((millis() - last_button_press_time) >= button_debounce_delay) {
  //   // there is a priority system here: 1-stop 2-jog 3-start
  //   if((pinput_stop != pinput_stop_previous)        && pinput_stop)  { stopMotor();  }
  //   else if((pinput_jog != pinput_jog_previous)     && pinput_jog)   { jogMotor();   }
  //   else if((pinput_start != pinput_start_previous) && pinput_start) { startMotor(); }
  // }

  // pinput_start_previous = pinput_start;
  // pinput_stop_previous  = pinput_stop;
  // pinput_jog_previous   = pinput_jog;
}

void initializePins() {
  // interrupt inputs (INPUT_PULLUP @ CHIP LEVEL)
  pinMode(HM_PIN_EMERGENCY_STOP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HM_PIN_EMERGENCY_STOP), intCallBackESTOP, FALLING);
  pinMode(HM_PIN_PROXIMITY_INTERRUPT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HM_PIN_PROXIMITY_INTERRUPT), intCallBackPROX, FALLING);
  
  // normal inputs
  //pinMode(HM_PIN_START_MOTOR_BTN, INPUT_PULLUP);
  //pinMode(HM_PIN_STOP_MOTOR_BTN, INPUT_PULLUP);
  //pinMode(HM_PIN_JOG_MOTOR_BTN, INPUT_PULLUP);

  // outputs
  pinMode(HM_PIN_MOTOR_ENABLE, OUTPUT);
  TURN_MOTOR_OFF;
}

void setup() {
  Serial.begin(9600); // hardware serial
  while(!Serial) { /* hang out */ }

  myNex.begin(9600); // software serial port (pin 11(RX) pin 12(TX))

  initializePins();
  myNex.writeStr("t1.txt", "INITIALIZED");
  lastHeartBeatTime = millis();
  motor_state = NOT_INIT;
  Serial.println("\nENTERING MAIN LOOP...\n");
}

void loop() {
  if(prox_interrupt_triggered) { 
    motor_state = MOTOR_STATE::INTERRUPT; 
    myNex.writeStr("t1.txt", "PROX INTERRUPT TRIGGERED"); 
    Serial.println(F("PROX INTERRUPT TRIGGERED!"));
    prox_interrupt_triggered = false;
  } else if (estop_interrupt_triggered) {
    motor_state = MOTOR_STATE::INTERRUPT; 
    myNex.writeStr("t1.txt", "E-STOP INTERRUPT TRIGGERED");
    Serial.println(F("E-STOP INTERRUPT TRIGGERED!")); 
    estop_interrupt_triggered = false;
  }

  myNex.NextionListen();
  
  HeartBeat(motor_state);
}