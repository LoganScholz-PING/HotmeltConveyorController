#include <Arduino.h>
#include "HotmeltConveyor.h"

extern void stopMotor();
extern void jogMotor();
extern void startMotor();

// stop button
void trigger0() {
    stopMotor();
}

// start button
void trigger1() {
    startMotor();
}

// set motor direction: reverse
void trigger2() {
    // TODO - is there a "reverse" motor switch?
} 

// jog motor button
void trigger3() {
    jogMotor();
} 

// set motor direction: forward
void trigger4() {
    // TODO - is there a "forward" motor switch?
} 
