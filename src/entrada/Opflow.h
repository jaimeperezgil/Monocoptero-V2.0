
#pragma once
#include "Arduino.h"
#include "Ajustes.h"
#include "soporte/Glovales.h"
#include "soporte/filter.h"

void optflow_init(Stream* _streamPtr);
void optflow_leer(float dT);
