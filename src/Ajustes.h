

/*
NOTAS:

Cambio total de systema de estabilidad frente a otras versiones PID para deg/s y fuerza*ang con LPF

*/

#define boton 2
#define led1 41
#define led2 40
#define led3 39
#define volt 24
#define ServoX 5
#define ServoXX 6
#define ServoY 7
#define ServoYY 8
#define motor 3
#define serialSpeed 115200

#define alt 80
#define potencia_base 1600        //<- necesita ser ajustado

#define calVolt 0.036

#define tiempo_vuelo 20  //segundos 30

#define maxAng 10  //+/- 10

#define adg_x 7         //Ajustar para cambiar punto sobre el que se busca el equilibrio
#define adg_y 20

#define MAX_data_log 1200         //Suficiente para 1 min de data log

#define f_to_c 150      //Comando de los alerones necesarios para generar 1N de fuerza

/*#define offsetX 1400    //1256
#define offsetXX 1400 //-10

#define offsetY 1500
#define offsetYY 1500*/

#define Kp_alt 80
#define Ki_alt 10                   //10
#define Kd_alt 0                //Sin efecto aparente mas bien mal

#define Kf_alt 1.25                //Comportamiento mas estable y predecible con valores vajos      1.25
#define Kd_level_alt 1000           //muy importante

#define deg_to_rad 0.0174533

/*#define Kp_alt 50
#define Ki_alt 10
#define Kd_alt 50

#define Kp_alt_g 140
#define Ki_alt_g 10
#define Kd_alt_g 50*/

//#define Kf_alt 5

#define vel_descenso 20      //5cm/s
#define alt_cal 16
#define alt_corte 5

#define tiempo_subir 5
#define altura_min 25
#define altura_potMax 10

#define tiempo_calentamiento 3

#define TFminiof 10

#define timpo_cooldows 5        //seg

#define max_ang 40

#define starUp false

#define IMU_x_cal 90
#define IMU_y_cal 6

#define buzzer_pin 23

#define beep_activado false           //desactivar zumbador
#define motor_armado true              //desactivar motor

#define testMode true           // desactivar comprovaciones

#define chipSelect BUILTIN_SDCARD