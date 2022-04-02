
#pragma once

#include <Servo.h>
#include <Arduino.h>
#include <Ajustes.h>
#include "vuelo/Glovales.h"

void mixer(float x,float y, float z);
void mixer_start();
void mixer_centrar();
void servo_write(float x, float xx, float y, float yy);