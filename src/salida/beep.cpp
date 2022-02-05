
#include "beep.h"
#include "Ajustes.h"
#include "vuelo/Glovales.h"

void beep_setup(){
return;
}

long b_reposo=0;
bool estado_beep_reposo=false;

void beep_reposo(){
    if(beep_activado){
        if(estado_beep_reposo){
            tone(buzzer_pin, 10);
            if(millis()-b_reposo>=100){
                b_reposo=millis();
                estado_beep_reposo=false;
            }
        }else{
            noTone(buzzer_pin);
            if(millis()-b_reposo>=1500){
                b_reposo=millis();
                estado_beep_reposo=true;
            }
        }
    }
}

long b_volando=0;
bool estado_beep_volando=false;

void beep_volando(){
    if(beep_activado){
        if(estado_beep_volando){
            tone(buzzer_pin, 10);
            if(millis()-b_volando>=100){    //tiempo encendido
                b_volando=millis();
                estado_beep_volando=false;
            }
        }else{
            noTone(buzzer_pin);
            if(millis()-b_volando>=500){   //timepo apagado
                b_volando=millis();
                estado_beep_volando=true;
            }
        }
    }
}