
#include "Control.h"

#include "data/Curba_motor.h"
#include "vuelo/filter.h"
#include "vuelo/PID.h"
#include "salida/Motor.h"
#include "vuelo/data_log.h"
#include "Matrices/Multiplicaccion_matrices.h"

using namespace BLA;

PID PID_altitud;

PD control_vel_x;
PD control_ang_x;

PD control_acc_x;
PD control_pos_vel_x;
PD control_pos_x;

PD control_vel_y;
PD control_ang_y;

PD control_acc_y;
PD control_pos_vel_y;
PD control_pos_y;

PD control_vel_z;
PD control_ang_z;

pt1Filter_t filtro_pos_x;
pt1Filter_t filtro_pos_y;

void reset_PID_integrales(){
  PID_altitud.rest_int();

  control_vel_x.reset_i();
  control_vel_y.reset_i();
}

double x_i,y_i;
double pos_comp_x,pos_comp_y,vel_comp_x,vel_comp_y, acc_comp_x, acc_comp_y;
double ang_vel_deseado_x, ang_vel_deseado_y;
double limit_x, limit_y;

void pid_setup(){
  
  PID_altitud.PID_constantes(Kp_alt,Ki_alt,Kd_alt,0,400,Kf_alt,0,10);
  PID_altitud.set_d_level(Kd_level_alt);

  log_set_canal_int("altitud",&altitud);
  log_set_canal_double("altitud_setpoint", &altitud_setpoint);

  log_set_canal_double("integral_x", &x_i);
  log_set_canal_double("integral_y", &y_i);

  control_vel_x.constantes(14,-0,0.3);//53   -20 -1
  control_vel_y.constantes(-14,0,-0.3);
  control_vel_z.constantes(20,0,0);

  control_ang_x.constantes(5,0,0); //37
  control_ang_y.constantes(5,0,0);
  control_ang_z.constantes(6,0,0);

 // control_acc_x.constantes(0.1,0,0);
  //control_acc_y.constantes(0.1,0,0);

  //control_pos_vel_x.constantes(0.13,0,0.008);
  //control_pos_vel_y.constantes(0.13,0,0.008);
  control_pos_vel_x.constantes(0.04,0,0.04);     //(0.04,0,0)
  //control_pos_vel_y.constantes(-0.09,0,-0.008);
  control_pos_vel_y.constantes(0.04,0,0.04);

  control_pos_x.constantes(0.07,0,0.04);      //0.2
  control_pos_y.constantes(-0.07,0,0.04);

  pt1FilterInit(&filtro_pos_x,1,0.001);
  pt1FilterInit(&filtro_pos_y,1,0.001);

  /*control_ang_x.constantes(0,0);
  control_ang_y.constantes(0,0);
  control_ang_z.constantes(0,0);*/

  log_set_canal_double("vel_deseado_x", &pos_comp_x);
  log_set_canal_double("vel_deseado_y", &pos_comp_y);

  //log_set_canal_double("ang_deseado_x", &vel_comp_x);
  //log_set_canal_double("ang_deseado_y", &vel_comp_y);

  log_set_canal_double("ang_deseado_x", &acc_comp_x);
  log_set_canal_double("ang_deseado_y", &acc_comp_y);

  log_set_canal_double("ang_vel_deseado_x", &ang_vel_deseado_x);
  log_set_canal_double("ang_vel_deseado_y", &ang_vel_deseado_y);

  //log_set_canal_double("lim_x", &lim_x);
  //log_set_canal_double("lim_y", &lim_y);
}

BLA::Matrix<4,6> K={53.0330,        0,      37.5000,    32.9225,        0,    25.3628,
                          0,  53.0330,    -37.5000,          0,   36.2015,   -25.3628,
                    53.0330,        0,     -37.5000,    32.9225,        0,   -25.3628,
                          0,  53.0330,     37.5000,          0,   36.2015,    25.3628};

BLA::Matrix<6> x={0,0,0,0,0,0};
BLA::Matrix<4> u={0,0,0,0};

double constraini(double n, double min, double max){
  if(min>max){
    double j=max;
    max=min;
    min=j;
  }
  return constrain(n,min,max);
}

void controlador(float dt){

  //Load vector estados

  x(0) = (pos.ang_x* 1000 / 57296);   //eje x
  x(3) = (pos.vel_x* 1000 / 57296);

  x(1) = (pos.ang_y* 1000 / 57296);   //eje y
  x(4) = (pos.vel_y* 1000 / 57296);

  x(2) = (pos.ang_z* 1000 / 57296);   //ejez
  x(5) = -(pos.vel_z* 1000 / 57296);

  u=K*x;

  pos_comp_x=control_pos_x.cal(pt1FilterApply3(&filtro_pos_x,pos.pos_x,dt));
  pos_comp_y=control_pos_y.cal(pt1FilterApply3(&filtro_pos_y,pos.pos_y,dt));
  
  //pos_comp_x=constrain(pos_comp_x,-2,2);
  //pos_comp_y=constrain(pos_comp_y,-2,2);

  vel_comp_x=control_pos_vel_x.cal(pos.pos_vel_x+pos_comp_x);
  vel_comp_y=control_pos_vel_y.cal(pos.pos_vel_y+pos_comp_y);

  /*if(abs(pos.pos_vel_x+pos_comp_x)>0.05f)
    //limit_x=(pos.pos_vel_x+pos_comp_x)*0.02f;
    limit_x=vel_comp_x;
  else limit_x=0;

  if(abs(pos.pos_vel_y+pos_comp_y)>0.05f)
    //limit_y=(pos.pos_vel_y+pos_comp_y)*0.02f;
    limit_y=vel_comp_y;
  else limit_y=0;*/

  //Serial.println(vel_comp_x,6);

  //acc_comp_x=constrain(atan2(9.8,vel_comp_x), -limit_x, limit_x);
  //acc_comp_y=constrain(atan2(9.8,-vel_comp_y*1000), -limit_y, limit_y);

  acc_comp_x=constraini(vel_comp_x, -limit_x, limit_x);
  acc_comp_y=constraini(-vel_comp_y, -limit_y, limit_y);

  double comp=0;

  /*if(abs(pos.pos_vel_x+pos_comp_x)<0.5f)
    comp=0.01f;
  else if(abs(pos.pos_vel_x+pos_comp_x)>0.5f)
    comp=0.05f;
  else comp=0;

  if(pos.pos_vel_x+pos_comp_x>=0.1f)
    acc_comp_x=comp;
  else if(pos.pos_vel_x+pos_comp_x>=-0.1f)
    acc_comp_x=0;
  else acc_comp_x=-comp;

  if(pos.pos_vel_y+pos_comp_y>=0.1f)
    acc_comp_y=-comp;
  else if(pos.pos_vel_y+pos_comp_y>=-0.1f)
    acc_comp_y=0;
  else acc_comp_y=comp;*/

  ang_vel_deseado_x=control_ang_x.cal((pos.ang_x* 1000 / 57296)+pos_comp_x);  //acc_comp_x
  ang_vel_deseado_y=control_ang_y.cal((pos.ang_y* 1000 / 57296)+pos_comp_y);  //acc_comp_y

  double comp_x=control_vel_x.cal((pos.vel_x* 1000 / 57296) + ang_vel_deseado_x);
  double comp_y=control_vel_y.cal((pos.vel_y* 1000 / 57296) + ang_vel_deseado_y);
  double comp_z=control_vel_z.cal(-(pos.vel_z* 1000 / 57296) + control_ang_z.cal((pos.ang_z* 1000 / 57296)));

  //lim_x=((pos.vel_x* 1000 / 57296) + ang_vel_deseado_x)*15.f;
  //lim_y=((pos.vel_y* 1000 / 57296) + ang_vel_deseado_y)*15.f;

  //comp_x=constraini(comp_x,-lim_x,lim_x);
  //comp_y=constraini(comp_y,-lim_y,lim_y);

  Serial.print(" ");
  Serial.print(" ");
  Serial.print(" ");
  Serial.print(" ");
  Serial.print(" ");
  Serial.print(" ");
  Serial.print(" ");
  Serial.print(" ");
  Serial.print(pos.pos_x);
  Serial.print(",");
  Serial.println(pos.pos_y);

  x_i=control_ang_x.get_i();
  y_i=control_ang_y.get_i();

  mixer(comp_x,comp_y,comp_z,dt);
  //mixer(0,0,0);
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
