
#include "Opflow.h"
#include "vuelo/data_log.h"

Stream* stream_opflow;

#define calidad_minima 50

float opflow_giro_scalar_x=2.5;
float opflow_giro_scalar_y=1.7;

float opflow_giro_comp_x;
float opflow_giro_comp_y;

int opflow_scalar=10000;

pt1Filter_t filtro_x;
pt1Filter_t filtro_y;

mediaFilter_t media_x;
mediaFilter_t media_y;

void optflow_init(Stream* _streamPtr){
    stream_opflow=_streamPtr;

    pt1FilterInit(&filtro_x,0.5,0.1);
    pt1FilterInit(&filtro_y,0.5,0.1);

    mediaFilter_init(&media_x,5);
    mediaFilter_init(&media_y,5);

    Serial.println("iniciando Opflow");

    log_set_canal_int("Opflow_calidad",&opflow_calidad);
}

double tiempo_acumulado;
double giro_acc_x;
double giro_acc_y;

//giro_update

void opflow_giro_update(float dt){
    giro_acc_x+=pos.vel_x*dt;
    giro_acc_y+=pos.vel_y*dt;

    tiempo_acumulado+=dt;
}

void opflow_giroAvr_zero(){
    giro_acc_x=0;
    giro_acc_y=0;
    tiempo_acumulado=0;
}

#define opflow_calibrando false
#define cal_const 0.03

double bodyrateAcc_x;
double flowAcc_x;

double bodyrateAcc_y;
double flowAcc_y;

byte opflow_cal_estado_x=0;
byte opflow_cal_estado_y=0;

void opflow_calibrar_x(){
    if(opflow_calibrando && altitud>0.1){
        if(opflow_cal_estado_x==0){
            bodyrateAcc_x+=-opflow_giro_comp_x;
            flowAcc_x+=opflow_vel_x;

            if(opflow_vel_x<0){
                if(bodyrateAcc_x>flowAcc_x){
                    opflow_giro_scalar_x-=cal_const;
                }else{
                    opflow_giro_scalar_x+=cal_const;
                }

                opflow_cal_estado_x=1;
                bodyrateAcc_x=0;
                flowAcc_x=0;
            }

        }else{
            bodyrateAcc_x+=opflow_giro_comp_x;
            flowAcc_x+=-opflow_vel_x;

            if(opflow_vel_x>0){
                if(bodyrateAcc_x>flowAcc_x){
                    opflow_giro_scalar_x-=cal_const;
                }else{
                    opflow_giro_scalar_x+=cal_const;
                }

                opflow_cal_estado_x=0;
                bodyrateAcc_x=0;
                flowAcc_x=0;
            }
        }
        //Serial.println(opflow_giro_scalar_x);
    }
}

void opflow_calibrar_y(){
    if(opflow_calibrando && altitud>0.1){
        if(opflow_cal_estado_y==0){
            bodyrateAcc_y+=opflow_giro_comp_y;
            flowAcc_y+=opflow_vel_y;

            if(opflow_vel_y<0){
                if(bodyrateAcc_y>flowAcc_y){
                    opflow_giro_scalar_y-=cal_const;
                }else{
                    opflow_giro_scalar_y+=cal_const;
                }

                opflow_cal_estado_y=1;
                bodyrateAcc_y=0;
                flowAcc_y=0;
            }

        }else{
            bodyrateAcc_y+=opflow_giro_comp_y;
            flowAcc_y+=opflow_vel_y;

            if(opflow_vel_y>0){
                if(bodyrateAcc_y>flowAcc_y){
                    opflow_giro_scalar_y-=cal_const;
                }else{
                    opflow_giro_scalar_y+=cal_const;
                }

                opflow_cal_estado_y=0;
                bodyrateAcc_y=0;
                flowAcc_y=0;
            }
        }
        Serial.println(opflow_giro_scalar_y);
    }
}

//https://ardupilot.org/plane/docs/common-mouse-based-optical-flow-sensor-adns3080.html

//#define opflow_resolucion 30
//#define opflow_FOV 42

void opflow_procesar(float dt){

    if(opflow_calidad<calidad_minima){
        return;
    }

    double scalar=(1.0/dt)/(float)opflow_scalar;

    //eje x;
    opflow_vel_x=opflow_vel_x*scalar;
    opflow_vel_x=opflow_vel_x* 1000 / 57296; 

    float opflow_giro_avr_x=giro_acc_x/tiempo_acumulado;
    float vel_x_rad=opflow_giro_avr_x* 1000 / 57296;             //deg/s a rad/s

    opflow_calibrar_x();

    opflow_giro_comp_x=vel_x_rad*(opflow_giro_scalar_x/10.0);

    double distancia_x=(opflow_vel_x+opflow_giro_comp_x)*(altitud/10);       //alt en cm        m --> cm
    distancia_x=mediaFilter_est(&media_x,distancia_x);
    distancia_x=pt1FilterApply(&filtro_x,distancia_x);
          
    opflow_vel_x=distancia_x;           

    //eje y
    opflow_vel_y=opflow_vel_y*scalar;
    opflow_vel_y=opflow_vel_y* 1000 / 57296; 

    float opflow_giro_avr_y=giro_acc_y/tiempo_acumulado;
    float vel_y_rad=opflow_giro_avr_y* 1000 / 57296;

    opflow_calibrar_y();

    opflow_giro_comp_y=vel_y_rad*(opflow_giro_scalar_y/10.0);

    double distancia_y=(opflow_vel_y-opflow_giro_comp_y)*(altitud/10);
    distancia_y=mediaFilter_est(&media_y,distancia_y);
    distancia_y=pt1FilterApply(&filtro_y,distancia_y);

    opflow_vel_y=distancia_y;  

    opflow_giroAvr_zero();

    opflow_update=true;
}

/*

estado 0      esperando
estado 1      comprevando MPS_v2
estado 2      esperar mensage de request
estado 3      leer

*/

int ofset_MSP=0;
uint8_t buf[120];            //Tamaño maximo buf
int estado_MSP=0;

long tiempo_opflow=0;

void optflow_leer(float dT){
    opflow_giro_update(dT);
    while(stream_opflow->available()>0){      //>=15
        uint8_t c=stream_opflow->read();       //HEX

        switch (estado_MSP){
            case 0:
                if(c=='$'){
                    estado_MSP=1;
                }
            break;

            case 1:
                if(c=='X'){
                    estado_MSP=2;
                }else{
                    estado_MSP=0;
                }
            break;

            case 2:
                if(c=='<'){
                    ofset_MSP=0;
                    estado_MSP=3;
                }else{
                    estado_MSP=0;
                }

            break;

            case 3:
                buf[ofset_MSP++]=c;
                if(ofset_MSP>=buf[3]+4){
                    ofset_MSP=0;
                    estado_MSP=0;

                    if(buf[1]==1){
                        opflow_lidar=(int)(int8_t)buf[7]*256+(int)buf[6];
                        
                        opflow_lidar=opflow_lidar/1000;
                    }

                    if(buf[1]==2){
                        opflow_vel_x=(int8_t)buf[6]*256+(int8_t)buf[7];                 //jj
                        opflow_vel_y=(int8_t)buf[10]*256+(int8_t)buf[11];

                        double dt=micros()-tiempo_opflow;
                        dt=dt/1000000;
                        tiempo_opflow=micros();

                        opflow_calidad=(int)buf[5];

                        opflow_procesar(dt);

                    }
                }
                if(c=='$'){         //Error
                    estado_MSP=0;
                }
            break;
        }
    }
}







