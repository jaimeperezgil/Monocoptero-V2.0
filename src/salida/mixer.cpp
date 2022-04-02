
#include "mixer.h"
#include "vuelo/data_log.h"

Servo servoX;
Servo servoXX;
Servo servoY;
Servo servoYY;

int out1,out2,out3,out4;

void mixer_start(){
    servoX.attach(ServoX);
    servoXX.attach(ServoXX);
    servoY.attach(ServoY);
    servoYY.attach(ServoYY);

    servoX.write(offsetX+90);
    servoXX.write(offsetXX+90);
    servoY.write(offsetY+90);
    servoYY.write(offsetYY+90);

    log_set_canal_int("Servo_X",&out1);
    log_set_canal_int("Servo_XX",&out2);
    log_set_canal_int("Servo_Y",&out3);
    log_set_canal_int("Servo_YY",&out4);
}

void mixer_centrar(){
    servoX.write(offsetX+90);
    servoXX.write(offsetXX+90);
    servoY.write(offsetY+90);
    servoYY.write(offsetYY+90);
}

void mixer(float x,float y, float z){

    int out1=constrain(x+z,-50,50);
    int out2=constrain((x*-1)+z,-50,50);

    int out3=constrain(y+z,-50,50);
    int out4=constrain((y*-1)+z,-50,50);

    servoX.writeMicroseconds((out1*5.555555)+offsetX);
    servoXX.writeMicroseconds((out2*5.55555)+offsetXX);

    servoY.writeMicroseconds((out3*5.55555)+offsetY);
    servoYY.writeMicroseconds((out4*5.55555)+offsetYY);
}

void servo_write(float x, float xx, float y, float yy){
    out1=constrain(x,-50,50);
    out2=constrain(xx,-50,50);
    out3=constrain(y,-50,50);
    out4=constrain(yy,-50,50);

    servoX.writeMicroseconds((out1*5.555555)+offsetX);
    servoXX.writeMicroseconds((out2*5.55555)+offsetXX);

    servoY.writeMicroseconds((out3*5.55555)+offsetY);
    servoYY.writeMicroseconds((out4*5.55555)+offsetYY);
}

