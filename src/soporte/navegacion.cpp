
#include "navegacion.h"

#include <SD.h>
#include <SPI.h>
//#include <string>
#include <vector>
#include"soporte/data_log.h"

using namespace std;

#define nombre_camino "camino.txt"

vector<pair<String,pair<int,int>>> vec(10);

File cam;

byte estado_camino;
int instruccion=0;
String men_camino;

void init_camino(){
    if(SD.exists(nombre_camino)){
        Serial.println("Camino encontrado");
    }else{
        while(true) Serial.println("Camino no encontrado");
    }

    descargar_camino();

    log_set_canal_int("instruccion",&instruccion);
    log_set_canal_double("pos_setpoint_x",&pos_setpoint_x);
    log_set_canal_double("pos_setpoint_y",&pos_setpoint_y);
    
}

void descargar_camino(){

    cam=SD.open(nombre_camino);

    while(cam.available()){
        char c=cam.read();

        if(c==' ' || c=='\n'){
            switch (estado_camino){
            case 0:
                estado_camino=1;
                vec[instruccion].first=men_camino;

                if(vec[instruccion].first=="Aterrizage"){
                    vec[instruccion].second.first=-1;
                    vec[instruccion].second.second=-1;
                    instruccion++;
                    estado_camino=0;
                }
                break;
            case 1:
                if(vec[instruccion].first=="Altitud"){
                    vec[instruccion].second.first=atoi(men_camino.c_str());
                    estado_camino++;
                }else if(vec[instruccion].first=="Esperar"){
                    vec[instruccion].second.first=atoi(men_camino.c_str());
                    vec[instruccion].second.second=-1;
                    instruccion++;
                    estado_camino=0;
                }else if(vec[instruccion].first=="CordenadaX"){
                    vec[instruccion].second.first=atoi(men_camino.c_str());
                    estado_camino++;
                }else if(vec[instruccion].first=="Cordenada_Y"){
                    vec[instruccion].second.first=atoi(men_camino.c_str());
                    estado_camino++;
                }
                break;
            case 2:
                if(vec[instruccion].first=="Altitud"){
                    vec[instruccion].second.second=atoi(men_camino.c_str());
                    instruccion++;
                    estado_camino=0;
                }else if(vec[instruccion].first=="CordenadaX"){
                    vec[instruccion].second.second=atoi(men_camino.c_str());
                    instruccion++;
                    estado_camino=0;
                }else if(vec[instruccion].first=="Cordenada_Y"){
                    vec[instruccion].second.second=atoi(men_camino.c_str());
                    instruccion++;
                    estado_camino=0;
                }
                break;
            }

        men_camino="";
        }else{
            men_camino+=c;
        }
    }
    cam.close();

    //delay(5000);

    for(int i=0;i<=instruccion-1;i++){
        /*Serial.println(vec[i].first);
        Serial.println(vec[i].second.first);
        Serial.println(vec[i].second.second);*/
    }
}

int instruccion_ejecutada=0;
long tiempo_cam;
float alt_last_setpoint=0;
float pos_setpoint_x_last=0;

void camino_rest(){
    tiempo_cam=millis();
    instruccion_ejecutada=0;
}

void camino(){

    if(vec[instruccion_ejecutada].first=="Altitud"){
        if(millis()-tiempo_cam>=100){
            tiempo_cam=millis();
            double media =(float)(vec[instruccion_ejecutada].second.first-alt_last_setpoint)/vec[instruccion_ejecutada].second.second;
            media/=10;
            altitud_setpoint+=media;
            altitud_setpoint=constrain(altitud_setpoint,0,vec[instruccion_ejecutada].second.first);
            if(altitud_setpoint>=vec[instruccion_ejecutada].second.first){
                tiempo_cam=millis();
                alt_last_setpoint=vec[instruccion_ejecutada].second.first;
                instruccion_ejecutada++;
            }
        }
    }else if(vec[instruccion_ejecutada].first=="Esperar"){
        if(millis()-tiempo_cam>=vec[instruccion_ejecutada].second.first*1000){
            instruccion_ejecutada++;
            tiempo_cam=millis();
        }
    }else if(vec[instruccion_ejecutada].first=="CordenadaX"){
        if(millis()-tiempo_cam>=100){
            tiempo_cam=millis();
            double media =(float)(vec[instruccion_ejecutada].second.first-pos_setpoint_x_last)/vec[instruccion_ejecutada].second.second;
            media/=10;
            pos_setpoint_x+=media;
            pos_setpoint_x=constrain(pos_setpoint_x,0,vec[instruccion_ejecutada].second.first);
            if(pos_setpoint_x>=vec[instruccion_ejecutada].second.first){
                tiempo_cam=millis();
                pos_setpoint_x_last=vec[instruccion_ejecutada].second.first;
                instruccion_ejecutada++;
            }
        }
    }else if(vec[instruccion_ejecutada].first=="Aterrizage"){
        estado=ATERRIZAGE;
    }
    //Serial.println(pos_setpoint_x);
}