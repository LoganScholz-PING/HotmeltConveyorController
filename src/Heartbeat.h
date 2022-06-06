#ifndef HEARTBEAT_H
#define HEARTBEAT_H

#include <Arduino.h>
#include "HotmeltConveyor.h"

extern unsigned long lastHeartBeatTime;
extern unsigned long heartBeatDelay;
extern MOTOR_STATE motor_state;

void HeartBeat() {
    if ((millis() - lastHeartBeatTime) > heartBeatDelay) {
      Serial.print(F(":<3:MS:"));

      switch(motor_state) { // print motor_state enum in a human-readable format
          case(RUNNING):
            Serial.println(F("RUNNING"));
            break;
          case(STOPPED):
            Serial.println(F("STOPPED"));
            break;
          case(NOT_INIT):
            Serial.println(F("NOT_INIT"));
            break;
          case(ERROR):
            Serial.println(F("ERROR"));
            break;
          default:
            Serial.println(F("UNK"));
            break;
      }

      lastHeartBeatTime = millis();
    }
}

#endif