
#include "data_log.h"

File dataFile;

int dir_act=0;

String data_buf[MAX_data_log];

const int indices_data_max=50;      //51 canales empezamos a contar desde 0-50
double data_log[indices_data_max+1];
String data_names[indices_data_max+1];
bool log_error=false;

void log_setup(){
  if (!SD.begin(BUILTIN_SDCARD)) {
    while(true){Serial.println("Targeta SD no detectada");}
  }
  Serial.println("SD inicializada");
}

using namespace std;

struct var_alog{
  String nombre;
  int inx;
  float (*var)(int);
};

vector<double*> vals_dou;
vector<float*> vals_float;
vector<int*> vals_int;
vector<float (*)()> vals_especial;

vector<var_alog> vars;

float dou(int inx){
  return *vals_dou[inx];
}
int ad_dou(double* dire){
  vals_dou.push_back(dire);
  return vals_dou.size()-1;
}

void log_set_canal_double(String n, double* v){
  var_alog s;
  s.nombre=n;
  s.inx=ad_dou(v);
  s.var=&dou;
  vars.push_back(s);
}

float flo(int inx){
  return *vals_float[inx];
}
int ad_flo(float* dire){
  vals_float.push_back(dire);
  return vals_float.size()-1;
}

void log_set_canal_float(String n, float* v){
  var_alog s;
  s.nombre=n;
  s.inx=ad_flo(v);
  s.var=&flo;
  vars.push_back(s);
}

float in(int inx){
  return *vals_int[inx];
}
int ad_in(int* dire){
  vals_int.push_back(dire);
  return vals_int.size()-1;
}

void log_set_canal_int(String n, int* v){
  var_alog s;
  s.nombre=n;
  s.inx=ad_in(v);
  s.var=&in;
  vars.push_back(s);
}

float esp(int inx){
  return vals_especial[inx]();
}
int ad_esp(float (*dire)()){
  vals_especial.push_back(dire);
  return vals_especial.size()-1;
}

void log_set_canal_esp(String n, float (*v)()){
  var_alog s;
  s.nombre=n;
  s.inx=ad_esp(v);
  s.var=&esp;
  vars.push_back(s);
}


void log(float dt){
  data_buf[dir_act]="";
  data_buf[dir_act]+=String(millis());
  for(int i=0;i<vars.size();i++){
    data_buf[dir_act]+=",";
    data_buf[dir_act]+=String(vars[i].var(vars[i].inx));
  }
  Serial.println(data_buf[dir_act]);
  dir_act++;
}

void SD_write(){
  Serial.println("Iniciando proceso de salvado");

  char nombre_archivo[10];
  int num_archivos=0;
  sprintf(nombre_archivo, "Vuelo%02d.csv", num_archivos);
  while(SD.exists(nombre_archivo)){
    num_archivos++;
    sprintf(nombre_archivo, "Vuelo%02d.csv", num_archivos);
  }

  Serial.print("Salvando a: ");
  Serial.println(nombre_archivo);

  File dataFile = SD.open(nombre_archivo, FILE_WRITE);

  dataFile.print("time");
  for(int i=0;i<vars.size();i++){
    dataFile.print(",");
    dataFile.print(vars[i].nombre);
  }
  dataFile.println();

  for(int i=0;i<dir_act;i++){
    dataFile.println(data_buf[i]);
  }
  dataFile.close();
  dir_act=0;  
}
