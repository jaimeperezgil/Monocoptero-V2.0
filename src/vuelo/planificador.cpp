
#include "planificador.h"

using namespace std;

struct proceso{
    String nombre;
    void (*funct)(float);
    unsigned long long ultima_eje=0;
    unsigned int dt_ejecucion;
    bool activo;
};

vector<proceso> procesos;

void add_poceso(String s, void (*f)(float), unsigned int dt, bool a=true){
    proceso p;
    p.nombre=s;
    p.funct=f;
    p.dt_ejecucion=dt;
    p.activo=a;
    procesos.push_back(p);
}

void run(){
    for(int i=0;i<procesos.size();i++){
        unsigned int dt=millis()-procesos[i].ultima_eje;
        if(dt>procesos[i].dt_ejecucion && procesos[i].activo){
            if(dt/1000.f<=0)dt=1;
            procesos[i].funct(dt/1000.f);
            procesos[i].ultima_eje=millis();
        }
    }
}

void proceso_activo(String s, bool e){
    for(int i=0;i<procesos.size();i++){
        if(procesos[i].nombre==s){
            procesos[i].activo=e;
            break;
        }
    }
}
