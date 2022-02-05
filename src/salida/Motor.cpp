
#include "Motor.h"
#include "soporte/data_log.h"

Servo Motor;

int potencia_motor=0;

void motor_setup(){
    Motor.attach(motor,1000,2000);
    Motor.write(0);
    log_set_canal_int("potencia_motor",&potencia_motor);
}

void Potencia_Motor(float p){       //Fuerza del motor en %
    if(motor_armado){
        p=constrain(p,0,100);
        potencia_motor=p;
        pot_motor=p;
        Motor.writeMicroseconds(1000+p*10);
    }
}

void Potencia_Motor_micros(double p){       //Fuerza del motor en microsegundos
    if(motor_armado){
        p=constrain(p,1000,2000);
        potencia_motor=(p-1000)/10;
        pot_motor=map(p,1000,2000,0,100);
        Motor.writeMicroseconds(p);
    }
}