
#pragma once
#include "Arduino.h"
#include "Ajustes.h"
#include "vuelo/Glovales.h"
#include "vuelo/filter.h"

void optflow_init(Stream* _streamPtr);
void optflow_leer(float dT);
