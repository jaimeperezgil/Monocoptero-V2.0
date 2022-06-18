
#include "Glovales.h"

estados estado;
POS pos;
IMUU IMU;

float voltage;
int altitud;
double vel_alt;
unsigned long tiempo=0;
byte pot_motor=0;
int set_alt=alt;
byte pDOP=100;
double altitud_setpoint = alt;
bool inicio_remoto=false;

bool stop=false;

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

bool opflow_update=false;

double integral_altitud=0;