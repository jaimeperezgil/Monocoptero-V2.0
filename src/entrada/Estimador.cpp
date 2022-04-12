
#include"Estimador.h"
#include "vuelo/data_log.h"

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

long tiempo_ang;
long tiempo_ant_int_z;

pt1Filter_t filtro_mag;

pt1Filter_t filtro_gyro_x;
pt1Filter_t filtro_gyro_y;

void init_estimador(){
  if(!bno.begin()){
    while(1){Serial.print("No se ha podido niciar BNO055");};
  }
  //bno.setExtCrystalUse(true);

  pt1FilterInit(&filtro_mag,5,0.01);
  pt1FilterInit(&filtro_gyro_x,10,0.01);
  pt1FilterInit(&filtro_gyro_y,10,0.01);

  //biquadFilterInit(filtro_gyro_x,60,

  log_set_canal_float("ang_x", &pos.ang_x);
  log_set_canal_float("ang_y", &pos.ang_y);
  log_set_canal_float("ang_z", &pos.ang_z);

  log_set_canal_double("pos_x", &pos.pos_x);
  log_set_canal_double("pos_y", &pos.pos_y);
  log_set_canal_double("pos_vel_x", &pos.pos_vel_x);
  log_set_canal_double("pos_vel_y", &pos.pos_vel_y);

  log_set_canal_double("vel_x", &pos.vel_x);
  log_set_canal_double("vel_y", &pos.vel_y);
  log_set_canal_double("vel_z", &pos.vel_z);

  log_set_canal_float("acc_x", &pos.acc_x);
  log_set_canal_float("acc_y", &pos.acc_y);
  log_set_canal_float("acc_z", &pos.acc_z);
}

void IMU_leer(float dt){

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> giro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  pos.ang_x=euler.z()+adg_x;
  pos.ang_y=euler.y()+adg_y;

  pos.vel_x=pt1FilterApply4(&filtro_gyro_x,-giro.x(),60,dt);    //deg/s  Documentacion dice rad/s pero es deg/s
  pos.vel_y=pt1FilterApply4(&filtro_gyro_x,-giro.y(),60,dt);
  //pos.vel_x=-giro.x();    //deg/s  Documentacion dice rad/s pero es deg/s
  //pos.vel_y=-giro.y();
  pos.vel_z=giro.z();

  pos.ang_z+=-pos.vel_z*dt;   //integracion de z apartir de giroscopios

}

void IMU_pos(float dt){
  sensors_event_t event;
  bno.getEvent(&event,Adafruit_BNO055::VECTOR_LINEARACCEL);

  pos.acc_x=event.acceleration.x;
  pos.acc_y=event.acceleration.y;
  pos.acc_z=event.acceleration.z;

  pos.pos_vel_x+=pos.acc_x*dt;
  pos.pos_vel_y+=pos.acc_y*dt;

  if(opflow_update){
    opflow_update=false;
    
    pos.pos_vel_x=opflow_vel_x*0.8 +pos.pos_vel_x*0.2;
    pos.pos_vel_y=opflow_vel_y*0.8 +pos.pos_vel_y*0.2;
  }

  pos.pos_x+=pos.pos_vel_x*dt;
  pos.pos_y+=pos.pos_vel_y*dt;

}


void mag_cal(){     //estimar z con magnetometro   No se utiliza
  sensors_event_t event;
  bno.getEvent(&event,Adafruit_BNO055::VECTOR_MAGNETOMETER);

  pos.mag_x=event.magnetic.x;    //deg/s  Documentacion dice rad/s pero es deg/s
  pos.mag_y=event.magnetic.y;
  pos.mag_z=event.magnetic.z;

  double ang_x_rad=pos.ang_x/360*(2*3.14);
  double ang_y_rad=pos.ang_y/360*(2*3.14);

  double xm_mag =(float)pos.mag_x*cos(ang_y_rad)
            -(float)pos.mag_y*sin(ang_x_rad)*sin(ang_y_rad)
            +(float)pos.mag_z*cos(ang_x_rad)*sin(ang_y_rad);

  double ym_mag =(float)pos.mag_y*cos(ang_x_rad)
            +(float)pos.mag_z*sin(ang_x_rad);

  double z_mag =atan2(ym_mag,xm_mag)/(2*3.14)*360;

  z_mag=constrain(z_mag,-180,180);

  z_mag=pt1FilterApply(&filtro_mag,z_mag);
}