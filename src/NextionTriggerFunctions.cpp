#include <Arduino.h>
#include "HotmeltConveyor.h"

extern void startMotor();
extern void stopMotor();
extern void jogMotor();

// start button
void trigger0() {
    startMotor();
}

// stop button
void trigger1() {
    stopMotor();
}

// jog
void trigger2() {
    jogMotor();
}
