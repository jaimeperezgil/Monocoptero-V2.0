//#pragma once
#include "vuelo/Glovales.h"

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//#include "Estimador.h"

int id_radio=0;
byte bt_ret=0;

const byte addresses [][6] = {"00001", "00002"};
RF24 radio(10, 26);

typedef struct midata {
  long t;
  float v1;
  float v2;
  float v3;
  float v4;
  float v5;
  float v6;
  byte id;
};
midata data;

struct mandar {
  byte D;
};
mandar sen;

void TLM_start(){
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1,addresses[1]);

  //Serial.println("Radio iniciada");
}

void recibir(float dt){
  radio.startListening();
  while(radio.available()){
    radio.read(&sen, sizeof(mandar));
  }

  if(sen.D==0){
    bt_ret=0;
  }
  if(sen.D==1){
    bt_ret=10;
    if(!inicio_remoto){
      inicio_remoto=true;
    }
  }
  if(sen.D==2){
    //haz lo que sea
    bt_ret=10;

  }
  if(sen.D==3){
    bt_ret=10;
    
    //PID_x.rest_int();
    //PID_y.rest_int();
    //PID_z.rest_int();

    pos.ang_z=0;

    pos.pos_vel_x=0;
    pos.pos_vel_y=0;
    pos.pos_x=0;
    pos.pos_y=0;

    opflow_pos_x=0;
    opflow_pos_x=0;
  }
  if(sen.D==4){
    bt_ret=10;
    estado=VOLANDO;
  }
}

void transmitir(){
  radio.stopListening();
  radio.write(&data, sizeof(midata));
}

long dt_r=0;

bool reducir_TLM=false;

void TLM_run(){
    transmitir();
    if(id_radio==0){
      data.id=id_radio;
      id_radio++;
      data.t=millis();

      data.v1=pos.ang_x;
      data.v2=pos.ang_y;
      data.v3=pos.ang_z;

      data.v4=pos.vel_x;
      data.v5=pos.vel_y;
      data.v6=pos.vel_z;

      //id_radio=0;

    }else if(id_radio==1){
      data.id=id_radio;
      id_radio++;
      data.t=millis();

      data.v1=pos.pos_vel_x;
      data.v2=pos.pos_vel_y;

      data.v3=opflow_pos_x;
      data.v4=opflow_pos_y;

      /*Serial.print(opflow_pos_x);
      Serial.print(",");
      Serial.println(opflow_pos_y);*/

      data.v5=altitud;
      data.v6=estado;
    }else if(id_radio==2){
      data.id=id_radio;
      data.t=millis();

      //pot_motor=map(pot_motor,0,180,0,100);
      data.v1=pot_motor;
      data.v2=altitud_setpoint;

      data.v3=pDOP;
      //data.v3=integral_altitud;
      data.v4=voltage;

      data.v5=0;

      data.v6=bt_ret;
      id_radio=0;
    }
}

long tiempo_TLM=0;

void TLM(float dt){
  if(!reducir_TLM){
    if(millis()-tiempo_TLM>16){
      tiempo_TLM=millis();
      TLM_run();
      if(millis()-tiempo_TLM>40){
        reducir_TLM=true;
      }
    }
  }
  if(reducir_TLM){
    if(millis()-tiempo_TLM>500){
      tiempo_TLM=millis();
     TLM_run();
     if(millis()-tiempo_TLM<15){
       reducir_TLM=false;
     }
    }
  }
}