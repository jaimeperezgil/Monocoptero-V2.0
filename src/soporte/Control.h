
#pragma once

#include <Arduino.h>
#include "soporte/Glovales.h"

void pid_setup();
void controlador(float dt);
float medir_lidar();
void lidar_strart();
void altitude(float dt);
void mixer();
void beep(bool est);
void reset_PID_integrales();
void servo_write(float x, float xx, float y, float yy);