
#include "Control.h"

#include "data/Curba_motor.h"
#include "vuelo/filter.h"
#include "vuelo/PID.h"
#include "salida/Motor.h"
#include "vuelo/data_log.h"
#include "Matrices/Multiplicaccion_matrices.h"

using namespace BLA;

PID PID_altitud;

void reset_PID_integrales(){
  PID_altitud.rest_int();
}

void pid_setup(){
  
  PID_altitud.PID_constantes(Kp_alt,Ki_alt,Kd_alt,0,400,Kf_alt,0,10);
  PID_altitud.set_d_level(Kd_level_alt);

  log_set_canal_int("altitud",&altitud);
  log_set_canal_double("altitud_setpoint", &altitud_setpoint);
}

BLA::Matrix<5,8> K={53.0330,        0,      37.5000,    32.9225,        0,    25.3628,      0,        0,
                          0,  53.0330,    -37.5000,          0,   36.2015,   -25.3628,      0,        0,  
                    53.0330,        0,     -37.5000,    32.9225,        0,   -25.3628,      0,        0,
                          0,  53.0330,     37.5000,          0,   36.2015,    25.3628,      0,        0,
                          0,        0,           0,          0,         0,          0, 3.0000,   5.0006};

BLA::Matrix<8> x={0,0,0,0,0,0,0,0};
BLA::Matrix<5> u={0,0,0,0,0};

pt1Filter_t filtro_alt;

void controlador(float dt){

  //Load vector estados

  x(0) = (pos.ang_x* 1000 / 57296);   //eje x
  x(3) = (pos.vel_x* 1000 / 57296);

  x(1) = (pos.ang_y* 1000 / 57296);   //eje y
  x(4) = (pos.vel_y* 1000 / 57296);

  x(2) = (pos.ang_z* 1000 / 57296);   //ejez
  x(5) = -(pos.vel_z* 1000 / 57296);

  int setpoint=constrain(altitud_setpoint,0,150);
  x(6) = (setpoint-altitud)/100.f;
  x(7) = vel_alt/100.f;
  //Serial.println(u());

  u=K*x;      //LQR

  servo_write(u(0),-u(2),-u(1),u(3));   //Escribir valores    servo_write(u(0),u(2),u(1),u(3));

  u(4)=pt1FilterApply(&filtro_alt,u(4));
  Potencia_Motor(constrain(u(4)+60,30,80));
}



void lidar_strart(){
  pt1FilterInit(&filtro_alt,1,0.001);
}

double distancia_ant_=0;

void altitude(float dt){
  int setpoint=constrain(altitud_setpoint,0,150);

  float error=setpoint-altitud;

  int vel_alt_=(distancia_ant_-altitud);      //vel altitud
  distancia_ant_=altitud;
  vel_alt_=vel_alt_*-1;

  double SetpointFuerza_alt = PID_altitud.level_deriviada(error);

  SetpointFuerza_alt=constrain(SetpointFuerza_alt,-50,50);

  int senal = PID_altitud.PID_cal(SetpointFuerza_alt,vel_alt_)+potencia_base;//potencia_minima();    //Calculamos punto medio dependiendo del voltage

  senal=pt1FilterApply(&filtro_alt,senal);

  Potencia_Motor_micros(senal);
  Serial.print("PID:  ");
  Serial.print(map(senal,1000,2000,0,100));
  Serial.print("   LQR:  ");
  Serial.println(u(4)+60);
}
