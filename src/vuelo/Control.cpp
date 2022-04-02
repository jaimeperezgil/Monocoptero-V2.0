
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

PD control_vel_x;
PD control_ang_x;

PD control_vel_y;
PD control_ang_y;

PD control_vel_z;
PD control_ang_z;

void pid_setup(){
  
  PID_altitud.PID_constantes(Kp_alt,Ki_alt,Kd_alt,0,400,Kf_alt,0,10);
  PID_altitud.set_d_level(Kd_level_alt);

  log_set_canal_int("altitud",&altitud);
  log_set_canal_double("altitud_setpoint", &altitud_setpoint);


  control_vel_x.constantes(35,4);//53   -20 -1
  control_vel_y.constantes(-35,-4);
  control_vel_z.constantes(20,0);

  control_ang_x.constantes(5,0.1); //37
  control_ang_y.constantes(5,0.1);
  control_ang_z.constantes(6,0);

  /*control_ang_x.constantes(0,0);
  control_ang_y.constantes(0,0);
  control_ang_z.constantes(0,0);*/
}

BLA::Matrix<4,6> K={53.0330,        0,      37.5000,    32.9225,        0,    25.3628,
                          0,  53.0330,    -37.5000,          0,   36.2015,   -25.3628,
                    53.0330,        0,     -37.5000,    32.9225,        0,   -25.3628,
                          0,  53.0330,     37.5000,          0,   36.2015,    25.3628};

BLA::Matrix<6> x={0,0,0,0,0,0};
BLA::Matrix<4> u={0,0,0,0};

void controlador(float dt){

  //Load vector estados

  x(0) = (pos.ang_x* 1000 / 57296);   //eje x
  x(3) = (pos.vel_x* 1000 / 57296);

  x(1) = (pos.ang_y* 1000 / 57296);   //eje y
  x(4) = (pos.vel_y* 1000 / 57296);

  x(2) = (pos.ang_z* 1000 / 57296);   //ejez
  x(5) = -(pos.vel_z* 1000 / 57296);

  u=K*x;

  float comp_x=control_vel_x.cal((pos.vel_x* 1000 / 57296) + control_ang_x.cal((pos.ang_x* 1000 / 57296)));
  float comp_y=control_vel_y.cal((pos.vel_y* 1000 / 57296) + control_ang_y.cal((pos.ang_y* 1000 / 57296)));
  float comp_z=control_vel_z.cal(-(pos.vel_z* 1000 / 57296) + control_ang_z.cal((pos.ang_z* 1000 / 57296)));

  Serial.print(" ");
  Serial.print(" ");
  Serial.print(" ");
  Serial.print(" ");
  Serial.print(" ");
  Serial.print(" ");
  Serial.print(" ");
  Serial.print(" ");
  Serial.print(" ");
  Serial.print(" ");
  //Serial.println(comp_x);

  mixer(comp_x,comp_y,comp_z);
  //servo_write(u(0),-comp_x,0,0);
  //servo_write(u(0),-u(2),-u(1),u(3));   //Escribir valores    servo_write(u(0),u(2),u(1),u(3));
}

pt1Filter_t filtro_alt;

void lidar_strart(){
  pt1FilterInit(&filtro_alt,1,0.04);
}

double distancia_ant=0;

void altitude(float dt){
  int setpoint=constrain(altitud_setpoint,0,150);

  float error=setpoint-altitud;

  vel_alt=(distancia_ant-altitud);      //vel altitud
  distancia_ant=altitud;
  vel_alt=vel_alt*-1;

  double SetpointFuerza_alt = PID_altitud.level_deriviada(error);

  SetpointFuerza_alt=constrain(SetpointFuerza_alt,-50,50);

  int senal = PID_altitud.PID_cal(SetpointFuerza_alt,vel_alt)+potencia_base;//potencia_minima();    //Calculamos punto medio dependiendo del voltage

  senal=pt1FilterApply(&filtro_alt,senal);

  Potencia_Motor_micros(senal);
}
