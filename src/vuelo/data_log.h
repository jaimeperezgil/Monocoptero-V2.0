
#pragma once

#include <SD.h>
#include <SPI.h>
#include <Arduino.h>
#include <Ajustes.h>
#include <vector>
#include "vuelo/Glovales.h"

void log_setup();
void SD_write();
void log(float dt);
void log_set_canal_double(String n, double* v);
void log_set_canal_float(String n, float* v);
void log_set_canal_int(String n, int* v);
void log_set_canal_esp(String n, float (*v)());
void log_rest_time();