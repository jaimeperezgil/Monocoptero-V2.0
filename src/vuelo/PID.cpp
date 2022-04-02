
#include "PID.h"

    void PID::PID_constantes(double P, double I, double D,double G, double sat, double f,double f_i,double f_c){
        Kp=P/multiplicador_p;
        Ki=I/multiplicador_i;
        Kd=D/multiplicador_d;
        Kg=G;

        fuerza=f/multiplicador_fuerza;
        fuerza_i=f_i;                         //integral angulo


        Saturacion=sat;

        pt1FilterInit(&filtro,f_c,0.01);     //frecuencia de corte 1 hz
        pt1FilterInit(&filtro_d,3,0.01);
        pt1FilterInit(&filtro_vel_d,3,0.01);
    }

    void PID::set_d_level(double d){
        Kd_level=d/multiplicador_d;
    }

    double PID::level(double ang){
        float dT = (double)(micros() - time_ant_level);
        dT=dT/1000000;

        if(dT>=0.005){
            time_ant_level=micros();
            double p = pt1FilterApply4(&filtro,ang*fuerza,10,dT);
            return p;
        }
    }

    double error_level_ant;

    double PID::level_deriviada(double ang){
        float dT = (double)(micros() - time_ant_level);
        dT=dT/1000000;

        if(dT>=0.005){
            time_ant_level=micros();
            double p = pt1FilterApply4(&filtro,ang*fuerza,10,dT);

            //Serial.println(fuerza_integral);
            double d=Kd_level*((ang-error_level_ant)/dT);
            error_level_ant=ang;

            return p+d;
        }
    }
    
    double PID::PID_cal(double Setpoint, double medicion){

        float dT = (double)(micros() - time_ant);
        dT=dT/1000000;

        double error=Setpoint-medicion;

        if(dT>=0.005){
            time_ant=micros();

            double p =error*Kp;
            
            Integral+=error*dT*Ki;
            Integral =constrain(Integral,Saturacion*-1,Saturacion);
            double i =integral_base;
            i+=Integral;
            i=constrain(i,Saturacion*-1,Saturacion);

            Integral=constrain(Integral,Saturacion*-1,Saturacion);

            double delta=error_ant-medicion;
            double d=delta*(Kd/dT);
            d=pt1FilterApply4(&filtro_d,d,15,dT);
            error_ant=medicion;

            salida=p+i+d;
            salida=constrain(salida,-500,500);
        }
        return salida;
    }

    void PID::rest_int(){
        Integral=0;
        time_ant=micros();
    }

    void PID::set_intagral(double i){
        integral_base=i;
    }

    double PID::get_intrgeal(){
        return Integral+integral_base;
    }

    double PID::get_derivada_pos(){
        return d_pos;
    }

    double PID::get_intrgeal_vel(){
        return integral_vel;
    }

    void PID::set_integral_vel(double i){
        integral_vel=i;
    }

    void PID::PID_pos_constantes(double p, double i, double d, double st){
        Kp_pos=p;
        ki_pos=i;
        Kd_pos=d;
        dt_pos=st; 
    }

    void PID::PID_vel_constantes(double p, double i, double d, double st){
        Kp_vel=p;
        Ki_vel=i;
        Kd_vel=d;
        dt_vel=st; 
    }

    double PID::vel_comp(double vel, double setpoint){

        float dT = (double)(micros() - time_ant_vel);
        dT=dT/1000000;

        if(dT>=dt_vel){
            double error=vel-setpoint;

            time_ant_vel=micros();

            double p=error*Kp_vel;

            integral_vel+=error*dT*Ki_vel;
            integral_vel=constrain(integral_vel,-10,10);

            double d=((vel_ant-vel)/dT)*Kd_vel;
            d= pt1FilterApply4(&filtro_vel_d,d,10,dT);
            vel_ant=vel;

            salida_vel=p+d+integral_vel;
        }
        return salida_vel;
    }

    double PID::pos_comp(double pos){
        float dT = (double)(micros() - time_ant_pos);
        dT=dT/1000000;

        if(dT>=dt_pos){
            time_ant_pos=micros();

            double p=pos*Kp_pos;
            d_pos=((pos-pos_ant)/dT)*Kd_pos;
            pos_ant=pos;

            integral_pos+=pos*dT*ki_pos;
            integral_pos=constrain(integral_pos,-5,5);

            salida_pos=p+d_pos+integral_pos;
        }
        return salida_pos;
    }

    void PD::constantes(double p, double d){
        Kp=p;
        Kd=d;

        pt1FilterInit(&filtro_d,10,0.001);
    }

    double PD::cal(double error){
        double dt=(micros() - time_ant)/1000000.f;
        time_ant=micros();
        //Serial.println(dt,6);

        double salida= error*Kp+pt1FilterApply4(&filtro_d,Kd*((error-error_ant)/dt),5,dt);
        error_ant=error;

        return salida;
    }