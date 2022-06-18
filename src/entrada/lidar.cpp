
#include "lidar.h"
#include "Arduino.h"
#include "Ajustes.h"
#include "vuelo/Glovales.h"

//pt1Filter_t filtro_medi_alt;

Stream* stream;


int state;
uint16_t distance;
uint16_t strength;


void TFmini_start(Stream* _streamPtr){  
  stream=_streamPtr;
  stream->write((uint8_t)0x42);
  stream->write((uint8_t)0x57);
  stream->write((uint8_t)0x02);
  stream->write((uint8_t)0x00);
  stream->write((uint8_t)0x00);
  stream->write((uint8_t)0x00);
  stream->write((uint8_t)0x01);
  stream->write((uint8_t)0x06);

  //pt1FilterInit(&filtro_medi_alt,1,0.1);
}

void getDistance(float dt){
    while(stream->available()>=9){      //Esperar a tener 9 bytes en la cola
      if((0x59 == stream->read()) && (0x59 == stream->read())) // byte 1 and byte 2
      {
        unsigned int t1 = stream->read(); // byte 3 = Dist_L
        unsigned int t2 = stream->read(); // byte 4 = Dist_H
        t2 <<= 8;
        t2 += t1;
        int TFminival=t2;
        //Serial.println(t2);
        altitud=t2-TFminiof;
        t1 = stream->read(); // byte 5 = Strength_L
        t2 = stream->read(); // byte 6 = Strength_H
        t2 <<= 8;
        t2 += t1;
        for(int i=0; i<3; i++)stream->read(); // byte 7, 8, 9 are ignored
      }
    }
}
