
#include "navegacion.h"

#include <SD.h>
#include <SPI.h>
#include <vector>
#include "vuelo/data_log.h"
#include <Arduino.h>
#include "vuelo/Glovales.h"
#include "vuelo/filter.h"
#include <map>

#define nombre_camino "camino.txt"

//using namespace std;

struct comando{
    String comando;
    int arg1,arg2;
};

std::vector<comando> vec;

struct instru{
    int arg;
    bool (*funct)(int,int);
};

std::map<String, instru> lista_comandos;

long tiempo_cam;
float alt_last_setpoint=0;
float pos_setpoint_x_last=0;

bool Peril_intruccion(int p,int n){
    //perfil =p;
    return true;
}

bool Altitud_intruccion(int a,int t){
    if(millis()-tiempo_cam>=100){
        tiempo_cam=millis();
        double media =(float)(a-alt_last_setpoint)/t;
        media/=10;
        altitud_setpoint+=media;
        altitud_setpoint=constrain(altitud_setpoint,0,a);
        if(altitud_setpoint>=a){
            tiempo_cam=millis();
            alt_last_setpoint=a;
            return true;
        }
    }
    return false;
}

bool Esperar_intruccion(int t,int n){
    if(millis()-tiempo_cam>=t*1000){
        tiempo_cam=millis();
        return true;
    }
    return false;
}

bool Cordenada_X_intruccion(int x,int t){
    if(millis()-tiempo_cam>=100){
        tiempo_cam=millis();
        double media =(float)(x-pos_setpoint_x_last)/t;
        media/=10;
        pos_setpoint_x+=media;
        pos_setpoint_x=constrain(pos_setpoint_x,0,x);
        if(pos_setpoint_x>=x){
            tiempo_cam=millis();
            pos_setpoint_x_last=x;
            return true;
        }
    }
    return false;
}

bool Cordenada_Y_intruccion(int p,int n){
    //perfil =p;
    return true;
}

bool Aterrizage_intruccion(int p,int n){
    estado=ATERRIZAGE;
    return false;
}

File cam;
int instruccion=0;

void init_camino(){
    if(SD.exists(nombre_camino)){
        Serial.println("Camino encontrado");
    }else{
        while(true) Serial.println("Camino no encontrado");
    }

    /*
    Perfil n
    Altitud alt tiempo(seg)
    Esperar tiempo(seg)
    Cordenada_X X tiempo(seg)
    Cordenada_Y Y tiempo(seg)
    Aterrizage  */
    lista_comandos["Perfil"].arg=1;
    lista_comandos["Perfil"].funct=&Peril_intruccion;
    lista_comandos["Altitud"].arg=2;
    lista_comandos["Altitud"].funct=&Altitud_intruccion;
    lista_comandos["Esperar"].arg=1;
    lista_comandos["Esperar"].funct=&Esperar_intruccion;
    lista_comandos["Cordenada_X"].arg=2;
    lista_comandos["Cordenada_X"].funct=&Cordenada_X_intruccion;
    lista_comandos["Cordenada_y"].arg=2;
    lista_comandos["Cordenada_y"].funct=&Cordenada_Y_intruccion;
    lista_comandos["Aterrizage"].arg=0;
    lista_comandos["Aterrizage"].funct=&Aterrizage_intruccion;

    descargar_camino();

    log_set_canal_int("instruccion",&instruccion);
    log_set_canal_double("pos_setpoint_x",&pos_setpoint_x);
    log_set_canal_double("pos_setpoint_y",&pos_setpoint_y);
}

byte estado_lectura;
String men_camino;
comando leyendo;

void descargar_camino(){
    cam=SD.open(nombre_camino);
delay(1000);
    while(cam.available()){
        char c=cam.read();

        if(c=='\n'){
            String s="";
            men_camino+=' ';
            leyendo.arg1=0;
            leyendo.arg2=0;
            for(int i=0;i<men_camino.length();i++){
                if(men_camino[i]==' '){
                    switch (estado_lectura){
                        case 0:
                            leyendo.comando=s;
                            estado_lectura++;
                        break;

                        case 1:
                            leyendo.arg1=atoi(s.c_str());
                            estado_lectura++;
                        break;

                        case 2:
                            leyendo.arg2=atoi(s.c_str());
                            estado_lectura++;
                        break;
                    }
                    s="";
                }else s+=men_camino[i];
            }
            //Comprobar que comando existe y numero de argumentos correcto
            if(lista_comandos.count(leyendo.comando)>0){
                if(lista_comandos[leyendo.comando].arg!=estado_lectura-1){
                    Serial.print("ERROR EN ");
                    Serial.print(vec.size()+1);
                    Serial.print(" INSTRUCCION. SE INTRODUJERON ");
                    Serial.print(estado_lectura-1);
                    Serial.print(" ARGUMENTOS Y SE ESPERABA ");
                    Serial.println(lista_comandos[leyendo.comando].arg);
                    stop=true;
                }
            }else{
                    stop=true;
                Serial.print("ERROR EN ");
                Serial.print(vec.size()+1);
                Serial.println(" INSTRUCCION. COMANDO NO EXISTE");
            }

            vec.push_back(leyendo);
            men_camino="";
            estado_lectura=0;
        }else men_camino+=c;
    }


    cam.close();

   /*for(int i=0;i<vec.size();i++){
        Serial.println(vec[i].comando);
        Serial.println(vec[i].arg1);
        Serial.println(vec[i].arg2);
   }*/
}

void camino_rest(){
    tiempo_cam=millis();
    instruccion=0;
}

void camino(){
    if(lista_comandos[vec[instruccion].comando].funct(vec[instruccion].arg1,vec[instruccion].arg2))instruccion++;
    //Serial.println(pos_setpoint_x);     //Cordenada_X 5 10
}