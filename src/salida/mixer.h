
#pragma once

#include <Servo.h>
#include <Arduino.h>
#include <Ajustes.h>
#include "vuelo/Glovales.h"
#include "vuelo/filter.h"

void mixer(float x,float y, float z, float dt);
void mixer_start();
void mixer_centrar();
void servo_write(float x, float xx, float y, float yy);