
#pragma once

#include <Arduino.h>
#include <Ajustes.h>
#include <vector>
#include "vuelo/Glovales.h"

void add_poceso(String s, void (*f)(float), unsigned int dt, bool a=true);
void run();
void proceso_activo(String s, bool e);
