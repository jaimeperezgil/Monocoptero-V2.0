
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

void mixer(){

    /*float x = Outputx;
    float y = Outputy*-1;
    float z = Outputz;

    int out1=constrain(x+offsetX +z,1100,1900);
    int out2=constrain((x*-1)+offsetXX +z,1100,1900);

    int out3=constrain(y+offsetY +z,1100,1900);
    int out4=constrain((y*-1)+offsetYY +z,1100,1900);

    servoX.writeMicroseconds(out1);
    servoXX.writeMicroseconds(out2);

    servoY.writeMicroseconds(out3);
    servoYY.writeMicroseconds(out4);

    set_data(38,out1);
    set_data(39,out2);
    set_data(40,out3);
    set_data(41,out4);*/
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

