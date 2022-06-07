#ifndef HEARTBEAT_H
#define HEARTBEAT_H

#include <Arduino.h>
#include "HotmeltConveyor.h"

extern unsigned long lastHeartBeatTime;
extern unsigned long heartBeatDelay;

void HeartBeat(MOTOR_STATE motor_state) {
    if ((millis() - lastHeartBeatTime) > heartBeatDelay) {
      //Serial.print("motor_state HB: "); Serial.println(motor_state);
      Serial.print(F(":<3:MS:"));

      switch(motor_state) { // print motor_state enum in a human-readable format
          case(MOTOR_STATE::RUNNING):
            Serial.println(F("RUNNING"));
            break;
          case(MOTOR_STATE::STOPPED):
            Serial.println(F("STOPPED"));
            break;
          case MOTOR_STATE::NOT_INIT:
            Serial.println(F("NOT_INIT"));
            break;
          case MOTOR_STATE::INTERRUPT:
            Serial.println(F("INTERRUPT"));
            break;
          case(MOTOR_STATE::JOGGING):
            Serial.println(F("JOGGING"));
            break;
          case(MOTOR_STATE::ERROR):
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