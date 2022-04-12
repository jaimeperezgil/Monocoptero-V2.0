
#pragma once
#include <Arduino.h>
#include "vuelo/Glovales.h"
#include "vuelo/filter.h"

class PD{
    double Kp, Kd, Ki;
    long time_ant;
    double error_ant;

    pt1Filter_t filtro_d;

    public:
    void constantes(double p,double i, double d);
    double cal(double error);
    float get_i();
    double integral=0;

    void reset_i();
};

class PID{
    
    double Kp,Ki,Kd, Kg, Kd_level;

    double fuerza;
    double fuerza_i;

    double fuerza_integral=0;

    double Saturacion;

    double error_ant;
    double Integral=0;
    long time_ant;
    long time_ant_level;

    double salida=0;

    double integral_base=0;

    pt1Filter_t filtro;
    pt1Filter_t filtro_d;
    pt1Filter_t filtro_vel_d;

    #define multiplicador_p 31.0f;              //Constantes para agustar valores con INAV
    #define multiplicador_i 4.0f;
    #define multiplicador_d 1905.0f;

    #define multiplicador_fuerza 6.56f;

    double Kp_vel, Ki_vel, Kd_vel, Kp_pos, Kd_pos, ki_pos;
    double vel_ant, pos_ant;
    long time_ant_vel, time_ant_pos;
    float dt_vel, dt_pos;
    double integral_pos=0,integral_vel=0;

    double salida_vel, salida_pos;
    double d_pos;

    public:
    void PID_constantes(double P, double I, double D,double G, double sat, double f,double f_i,double f_c);

    void set_d_level(double d);

    double level(double ang);

    double error_level_ant;

    double level_deriviada(double ang);
    
    double PID_cal(double Setpoint, double medicion);

    void rest_int();

    void set_intagral(double i);

    double get_intrgeal();

    double get_intrgeal_vel();

    void set_integral_vel(double i);

    void PID_pos_constantes(double p, double i, double d, double st);

    void PID_vel_constantes(double p, double i, double d, double st);

    double vel_comp(double vel, double setpoint);

    double pos_comp(double pos);

    double get_derivada_pos();
};