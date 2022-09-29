#include <Arduino.h>

// for nextion display
#include <SoftwareSerial.h> 
#include <EasyNextionLibrary.h> // from local project "lib" folder - modified to use SoftwareSerial

// for VL6180x
#include <Wire.h>
#include <VL6180X.h>

// general or program specific includes 
#include "HotmeltConveyor.h"
#include "Heartbeat.h"
#include "NextionTriggerFunctions.h"

// library object instantiations
SoftwareSerial swSerial(11, 12); // nextion display will be connected to 11(TX) and 12(RX)
EasyNex myNex(swSerial);

VL6180X LIDAR;

// program state tracking variables
MOTOR_STATE motor_state = MOTOR_STATE::NOT_INIT;
bool MOTOR_OFF = false;
bool MOTOR_ON  = true;

volatile bool prox_interrupt_triggered = false;
volatile bool estop_interrupt_triggered = false;
bool lidar_interrupt_triggered = false;

#define TURN_MOTOR_OFF digitalWrite(HM_PIN_MOTOR_ENABLE, true)
#define TURN_MOTOR_ON  digitalWrite(HM_PIN_MOTOR_ENABLE, false);

unsigned long lastHeartBeatTime = 0;
unsigned long heartBeatDelay    = 1000;

unsigned long last_button_press_time = 0;
unsigned long button_debounce_delay = 500;

unsigned long jog_timer = 2000;

// nextion button tracking (may not need)
bool dInput_stop        = false;
bool dInput_start       = false;
bool dInput_mtr_reverse = false;
bool dInput_mtr_forward = false;
bool dInput_jog         = false;

void delaySafeMilli(unsigned long time_to_wait) {
  unsigned long start = millis();
  while(millis() - start <= time_to_wait) { /* do nothing but let background tasks run */ }
}

void intCallBackESTOP() {
  TURN_MOTOR_OFF;
  estop_interrupt_triggered = true;
  motor_state = MOTOR_STATE::INTERRUPT;
}

void intCallBackPROX() {
  TURN_MOTOR_OFF;
  prox_interrupt_triggered = true;
  motor_state = MOTOR_STATE::INTERRUPT;
}

void startMotor() {
  // start button pressed
  Serial.println(F("[INFO] START"));
  motor_state = MOTOR_STATE::RUNNING;
  TURN_MOTOR_ON;
  myNex.writeStr("t0.txt", "MOTOR STATUS: STARTED");
  last_button_press_time = millis();
}

void stopMotor() {
  // turn off the motor
  Serial.println(F("[INFO] STOP"));
  motor_state = MOTOR_STATE::STOPPED;
  TURN_MOTOR_OFF; // turn off motor
  myNex.writeStr("t0.txt", "MOTOR STATUS: STOPPED");
  last_button_press_time = millis();
}

void jogMotor() {
  // jog the conveyor forward jog_timer milliseconds
  Serial.println(F("[INFO] JOG"));
  myNex.writeStr("t0.txt", "MOTOR STATUS: JOGGING");
  motor_state = MOTOR_STATE::JOGGING;
  TURN_MOTOR_ON;
  unsigned long start_time = millis();
  while(millis() - start_time <= jog_timer) { /* move motor for jog_timer milliseconds */ }
  stopMotor();
  last_button_press_time = millis();
}

void initializePins() {
  // interrupt inputs (INPUT_PULLUP @ CHIP LEVEL)
  // physical emergency stop button interrupt
  pinMode(HM_PIN_EMERGENCY_STOP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HM_PIN_EMERGENCY_STOP), intCallBackESTOP, FALLING);
  // 1st LIDAR proximity interrupt (OR MECHANICAL BUTTON INTERRUPT)
  pinMode(HM_PIN_PROXIMITY_INTERRUPT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HM_PIN_PROXIMITY_INTERRUPT), intCallBackPROX, FALLING);

  // outputs
  pinMode(HM_PIN_MOTOR_ENABLE, OUTPUT);
  TURN_MOTOR_OFF;
}

void initLIDAR() {
  Wire.begin();
  LIDAR.init();
  LIDAR.configureDefault();
  LIDAR.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  LIDAR.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  LIDAR.setTimeout(200);

  // stop continuous mode if already active
  LIDAR.stopContinuous();
  // in case stopContinuous() triggered a single-shot
  // measurement, wait for it to complete
  delaySafeMilli(300);
  // start interleaved continuous mode with period of 100 ms
  LIDAR.startRangeContinuous(50);
}

void initializeNextionDisplay() {
  myNex.begin(38400); // software serial port (pin 11(RX) pin 12(TX))
}

void checkLIDARDistance() {
  if(!lidar_interrupt_triggered && 
     (motor_state != MOTOR_STATE::INTERRUPT && motor_state != MOTOR_STATE::STOPPED && motor_state != MOTOR_STATE::ERROR) && 
     LIDAR.readRangeContinuousMillimeters() <= 250) {
    TURN_MOTOR_OFF;
    myNex.writeStr("t0.txt", "--> CONVEYOR ENDSTOP INTERRUPT!");
    motor_state = MOTOR_STATE::INTERRUPT;
    lidar_interrupt_triggered = true;
  }

  if (LIDAR.timeoutOccurred()) { Serial.println("ERROR!! LIDAR TIMEOUT!"); }
}

void setup() {
  Serial.begin(38400); // hardware serial
  while(!Serial) { /* hang out */ }

  initializeNextionDisplay();
  initializePins();
  initLIDAR();
  
  myNex.writeStr("t0.txt", "--> CONVEYOR INITIAL STARTUP");
  lastHeartBeatTime = millis();
  motor_state = MOTOR_STATE::NOT_INIT;

  Serial.println("\nENTERING MAIN LOOP...\n");
}

void loop() {
  unsigned long loop_timer_start = micros();
  
  checkLIDARDistance(); // will set prox_interrupt_triggered = true if anything is within 250mm

  if (prox_interrupt_triggered) { 
    motor_state = MOTOR_STATE::INTERRUPT; 
    myNex.writeStr("t0.txt", "PROX INTERRUPT TRIGGERED"); 
    Serial.println(F("PROX INTERRUPT TRIGGERED!"));
    prox_interrupt_triggered = false;
  } else if (estop_interrupt_triggered) {
    motor_state = MOTOR_STATE::INTERRUPT; 
    myNex.writeStr("t0.txt", "E-STOP INTERRUPT TRIGGERED");
    Serial.println(F("E-STOP INTERRUPT TRIGGERED!")); 
    estop_interrupt_triggered = false;
  } else if (lidar_interrupt_triggered) {
    if(LIDAR.readRangeContinuousMillimeters() > 250) {
      if(LIDAR.timeoutOccurred()) {
        Serial.println("LIDAR TIMEOUT IN MAIN LOOP");
      } else {
        TURN_MOTOR_ON;
        motor_state = MOTOR_STATE::RUNNING;
        lidar_interrupt_triggered = false;
        myNex.writeStr("t0.txt", "--> REMOVED CONVEYOR ENDSTOP INTERRUPT!");
      }
    }
  }

  myNex.NextionListen();
  HeartBeat(motor_state);

  unsigned long loop_timer_stop = micros();

  unsigned long loop_delta = loop_timer_stop - loop_timer_start;
  Serial.print(" --> Loop Delta (uS): ");
  Serial.println(loop_delta);
}