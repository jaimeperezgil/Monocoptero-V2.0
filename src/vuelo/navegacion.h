/*

Al pulsar boton se empieza minion

Crear sub estados iguales a numero de instrucciones

Comandos:

Altitud alt tiempo(seg)
Esperar tiempo(seg)
Cordenada_X X tiempo(seg)
Cordenada_Y Y tiempo(seg)
Altitud alt tiempo(seg)
Aterrizage

*/

#pragma once
#include <Arduino.h>
#include "vuelo/Glovales.h"
#include "vuelo/filter.h"

void init_camino();
void descargar_camino();
void camino();
void camino_rest();