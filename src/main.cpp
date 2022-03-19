
#include <Arduino.h>
#include "vuelo/Glovales.h"

#include "Ajustes.h"
#include "vuelo/Control.h"
#include "salida/Motor.h"
#include "entrada/Estimador.h"

#include "vuelo/filter.h"
#include <EEPROM.h>
#include "vuelo/EEPROMAnything.h"
#include "comunicaciones/Telemetria.h"
#include "vuelo/data_log.h"
#include "entrada/Opflow.h"
#include "entrada/lidar.h"
#include "salida/mixer.h"
#include "salida/beep.h"

#include "vuelo/data_log.h"
#include "vuelo/navegacion.h"
#include "vuelo/planificador.h"

pt1Filter_t f;

void load_EEPROM(){
  int direccion=0;
  direccion+=EEPROM_readAnything(direccion,offsetX);
  direccion+=EEPROM_readAnything(direccion,offsetXX);
  direccion+=EEPROM_readAnything(direccion,offsetY);
  direccion+=EEPROM_readAnything(direccion,offsetYY);

  direccion=20;
  pos.of_imu_x=map(EEPROM.read(direccion),0,255,-127,127);
  direccion++;
  pos.of_imu_y=map(EEPROM.read(direccion),0,255,-127,127);
}

long tiempo_alt;

bool ajuste_pos=false;
bool ajuste_cal=false;

long cooldown=0;
bool error=false;

long tiempo_bajada=0;

long lop=0;
int lt=0;

bool vateria_suficiente=true;

float estado_float(){
  return estado;
}

void bateria(float dt){
  voltage=analogRead(volt)*calVolt;
  if(voltage<19 && voltage>3){      //Sin vateria pero hay una vateria conectada
    vateria_suficiente=false;
  }else{
    vateria_suficiente=true;
  }
}

void setup(){
  delay(1000);

  Serial.begin(serialSpeed);
  Serial5.begin(115200);
  Serial8.begin(115200);

  pinMode(boton, INPUT_PULLUP);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);

  pinMode(volt, INPUT);

  log_setup();
  pid_setup();
  motor_setup();
  mixer_start();
  load_EEPROM();
  lidar_strart();
  optflow_init(&Serial8);
  TFmini_start(&Serial5);
  init_estimador();
  TLM_start();
  init_camino();

  log_set_canal_esp("estado", &estado_float);
  log_set_canal_float("voltage", &voltage);
  log_set_canal_int("loop_time", &lt);

  add_poceso("Leer_IMU", &IMU_leer, 1);
  add_poceso("Leer_optflow", &optflow_leer, 0);
  add_poceso("Integrara_acc", &IMU_pos, 0);
  add_poceso("leer_lidar", &getDistance, 42);

  add_poceso("controlador", &controlador, 1);
  add_poceso("Control_atitud", &altitude, 23, false);

  add_poceso("log_datos", &log, 51, false);
  add_poceso("TLM", &TLM, 16,false);
  add_poceso("recibir_datos", &recibir, 7,false);

  add_poceso("Bateria", &bateria, 101);

  estado=PREPARADO;
}

void loop(){

  lt=millis()-lop;
  lop=millis();

  run();

  //Serial.print(pos.ang_x);
  //Serial.print(",");
  //Serial.println(pos.ang_y);
  if(lt>1)Serial.println(lt);

  if(estado==PREPARADO || estado==BLOQUEADO){         //si no estamos volando motor=0
    Potencia_Motor(0);
    beep_reposo();
  }

  if((digitalRead(boton)==false || inicio_remoto ) && (estado==PREPARADO || estado==ENFRIAMIENTO)){        //Iniciamos la mision
    estado=CALENTANDO;
    inicio_remoto=false;
    digitalWrite(led2, HIGH);

    tiempo=millis();
  }

  if(estado==CALENTANDO){                  //calentamiento
    Potencia_Motor(50);
    if(millis()-tiempo>tiempo_calentamiento*1000){
      reset_PID_integrales();

      altitud_setpoint=altura_min;
      pos.ang_z=0;
      pos.pos_vel_x=0;
      pos.pos_vel_y=0;
      pos.pos_x=0;
      pos.pos_y=0;
      opflow_pos_x=0;
      opflow_pos_y=0;
      pos.ang_z=0;

      camino_rest();

      proceso_activo("Control_atitud", true);
      proceso_activo("log_datos", true);

      estado=VOLANDO;
    }
  }

  if(estado==VOLANDO ||estado==ATERRIZAGE){         //Volando
    digitalWrite(led1, HIGH);
    beep_volando();

    camino();                         //Ajuster setpoinst segun camino a seguir

    if(!vateria_suficiente){
      estado=ATERRIZAGE;
    }

  }else{
    altitud_setpoint=0;
    digitalWrite(led1,LOW);
  }

  if(estado==ATERRIZAGE){                          //Calcular descenso
    digitalWrite(led3, HIGH);
    if(millis()-tiempo_bajada>=100){
      tiempo_bajada=millis();
      float media=(float)vel_descenso/10;
      altitud_setpoint-=media;
    }
    if(altitud<alt_corte){
      proceso_activo("Control_atitud", false);
      proceso_activo("log_datos", false);
      estado=ENFRIAMIENTO;
      cooldown=millis();
      altitud_setpoint=0;
    }
    if(altitud_setpoint<alt_corte){
      altitud_setpoint=0;
    }
  }else{
    digitalWrite(led3, LOW);
  }

  if(estado==ENFRIAMIENTO){                                    //Calcular enfriamiento

    if(millis()-cooldown>timpo_cooldows*1000){
      estado=PREPARADO;
      Potencia_Motor(0);
      digitalWrite(led2, LOW);
      SD_write();
    }else{
      Potencia_Motor(30);
    }
  }

  if((estado==VOLANDO ||estado==ATERRIZAGE) && (pos.ang_x>50 or pos.ang_x<-50 or pos.ang_y>50 or pos.ang_y<-50)){   //Parada de emergencia
    while(true){
      Potencia_Motor(0);
      delay(100);
      tone(buzzer_pin, 400);
      delay(100);
      noTone(buzzer_pin);
    }
  }

  //Configuraccion por teminal

  if(estado==PREPARADO || estado==BLOQUEADO){
    digitalWrite(led2, LOW);
    if(Serial.available()){
      String m=Serial.readString();

      estado=BLOQUEADO;
      if(m=="puntoMedio"){
        ajuste_pos=true;

        Serial.println("Instrucciones para centrar los servos:");
        Serial.println("X: 1 +10 y 2 -10  XX: 3 +10 y 4 -10   Y: 5 +10 y 6 -10    YY: 7 +10 y 8 -10");
        Serial.println("9 para guardar  10 resetear");

      }else if(m=="calibrar"){
        ajuste_cal=true;

        Serial.println("Calibrando");
      }else{
        Serial.println("Comando no encontrado   Comandos disponibles:\npuntoMedio  calibrar");
        estado=PREPARADO;
      }
    }

    if(ajuste_pos){
      int j =Serial.parseInt();

      if(j==1){
        offsetX+=10;
      }else if(j==2){
        offsetX-=10;
      }else if(j==3){
        offsetXX+=10;
      }else if(j==4){
        offsetXX-=10;
      }else if(j==5){
        offsetY+=10;
      }else if(j==6){
        offsetY-=10;
      }else if(j==7){
        offsetYY+=10;
      }else if(j==8){
        offsetYY-=10;
      }else if(j==9){
        int direccion=0;
        direccion+=EEPROM_writeAnything(direccion,offsetX);
        direccion+=EEPROM_writeAnything(direccion,offsetXX);
        direccion+=EEPROM_writeAnything(direccion,offsetY);
        direccion+=EEPROM_writeAnything(direccion,offsetYY);
        ajuste_pos=false;
        estado=PREPARADO;
        Serial.println("Salvado");
      }else if(j==10){
        offsetX=1500;
        offsetXX=1500;
        offsetY=1500;
        offsetYY=1500;
      }
        
      Serial.print("Valoreas actuales: X ");
      Serial.print(offsetX);
      Serial.print(" XX ");
      Serial.print(offsetXX);
      Serial.print(" Y ");
      Serial.print(offsetY);
      Serial.print(" YY ");
      Serial.println(offsetYY);
    }

  }

}     //fin void loop
