
#include "Glovales.h"

estados estado;
poss pos;

double giro_update;

float voltage;
double erorr_ant_alt;
unsigned long aptTime;
int altitud;
double vel_alt;
int n_vuelos=0;
double avr_pot_motor;
byte avr_pot_motor_count;
unsigned long tiempo=0;
byte pot_motor=0;
int set_alt=alt;
byte pDOP=100;
byte GPS_sats=0;
double altitud_setpoint = alt;
bool inicio_remoto=false;

double opflow_pos_x=0;
double opflow_pos_y=0;
double opflow_vel_x=0;
double opflow_vel_y=0;

double pos_setpoint_x=0;
double pos_setpoint_y=0;

double opflow_lidar;
int opflow_calidad;

int offsetX;
int offsetXX;
int offsetY;
int offsetYY;

double pos_com_x;
double pos_com_y;
double vel_com_x;
double vel_com_y;

bool opflow_update=false;

double integral_altitud=0;

int TFminival=0;