
#pragma once

#include "Ajustes.h"
#include <math.h>
#include <Wire.h>
#include "VL53L0X.h"



typedef struct poss{
    float ang_x, ang_y, ang_z;
    float acc_x, acc_y, acc_z;
    double vel_x, vel_y, vel_z;
    double mag_x, mag_y, mag_z;

    double pos_vel_x, pos_vel_y;
    double pos_x, pos_y;

    double pos_acc_vel_x, pos_acc_vel_y;

    double of_imu_x;
    double of_imu_y;
};
extern poss pos;

extern double opflow_pos_x;
extern double opflow_pos_y;
extern double opflow_vel_x;
extern double opflow_vel_y;
extern double opflow_lidar;
extern int opflow_calidad;

extern int offsetX;
extern int offsetXX;
extern int offsetY;
extern int offsetYY;

extern double pos_com_x;
extern double pos_com_y;
extern double vel_com_x;
extern double vel_com_y;

extern double pos_setpoint_x;
extern double pos_setpoint_y;

extern bool opflow_update;
extern byte pDOP;

extern double integral_altitud;

extern float voltage;
extern int altitud;
extern double vel_alt;
extern unsigned long tiempo;
extern byte pot_motor;
extern double altitud_setpoint;
extern bool inicio_remoto;

typedef enum {
  E_STOP,
  BLOQUEADO,
  PREPARADO,
  CALENTANDO,
  //SUBIENDO,
  VOLANDO,
  ATERRIZAGE,
  ENFRIAMIENTO
} estados;
extern estados estado;