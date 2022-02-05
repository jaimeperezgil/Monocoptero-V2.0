
#pragma once

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "soporte/filter.h"
#include "soporte/Glovales.h"

void init_estimador();
void IMU_leer(float dt);
void IMU_pos(float dt);