
#pragma once

#include <Arduino.h>
#include "vuelo/Glovales.h"

void pid_setup();
void controlador(float dt);
float medir_lidar();
void lidar_strart();
void altitude(float dt);
void mixer(float x,float y, float z,float dt);
void beep(bool est);
void reset_PID_integrales();
void servo_write(float x, float xx, float y, float yy);