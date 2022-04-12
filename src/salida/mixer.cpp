
#include "mixer.h"
#include "vuelo/data_log.h"

Servo servoX;
Servo servoXX;
Servo servoY;
Servo servoYY;

pt1Filter_t filtro_X;
pt1Filter_t filtro_XX;
pt1Filter_t filtro_Y;
pt1Filter_t filtro_YY;

float out1,out2,out3,out4;

void mixer_start(){
    servoX.attach(ServoX,500,2500);
    servoXX.attach(ServoXX,500,2500);
    servoY.attach(ServoY,500,2500);
    servoYY.attach(ServoYY,500,2500);

    servoX.write(offsetX+90);
    servoXX.write(offsetXX+90);
    servoY.write(offsetY+90);
    servoYY.write(offsetYY+90);

    pt1FilterInit(&filtro_X,20,0.01);
    pt1FilterInit(&filtro_XX,20,0.01);
    pt1FilterInit(&filtro_Y,20,0.01);
    pt1FilterInit(&filtro_YY,20,0.01);

    log_set_canal_float("Servo_X",&out1);
    log_set_canal_float("Servo_XX",&out2);
    log_set_canal_float("Servo_Y",&out3);
    log_set_canal_float("Servo_YY",&out4);
}

void mixer_centrar(){
    servoX.write(offsetX+90);
    servoXX.write(offsetXX+90);
    servoY.write(offsetY+90);
    servoYY.write(offsetYY+90);
}

void mixer(float x,float y, float z, float dt){

    out1=constrain(x+z,-40,40);
    out2=constrain((x*-1)+z,-40,40);

    out3=constrain(y+z,-40,40);
    out4=constrain((y*-1)+z,-40,40);

    servoX.writeMicroseconds(pt1FilterApply3(&filtro_X,out1*10.f+offsetX,dt));
    servoXX.writeMicroseconds(pt1FilterApply3(&filtro_XX,out2*10.f+offsetXX,dt));

    servoY.writeMicroseconds(pt1FilterApply3(&filtro_Y,out3*10.f+offsetY,dt));
    servoYY.writeMicroseconds(pt1FilterApply3(&filtro_YY,out4*10.f+offsetYY,dt));
}

void servo_write(float x, float xx, float y, float yy){
    out1=constrain(x,-40,40);
    out2=constrain(xx,-40,40);
    out3=constrain(y,-40,40);
    out4=constrain(yy,-40,40);

    servoX.writeMicroseconds((out1*10)+offsetX);  //5.555555
    servoXX.writeMicroseconds((out2*10)+offsetXX);

    servoY.writeMicroseconds((out3*10)+offsetY);
    servoYY.writeMicroseconds((out4*10)+offsetYY);
}

