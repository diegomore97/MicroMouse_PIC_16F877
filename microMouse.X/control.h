#ifndef CONTROL_H
#define	CONTROL_H

#include <math.h>
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define ENA 1 //Canal de PWM que se controla, va conectado al motor Izquierdo
#define ENB 2 //Canal de PWM que se controla, va conectado al motor Derecho

//Ajustar estas variables de control para evitar chocar con las paredes laterales
#define KP 1.0 //Entre mas se aumente esta variable mas brusco sera el cambio para centrar
#define KI 0.1 //Valor para eliminar el error en estado estacionario
#define KD 1.3 //Entre mas aumente esta variabe mas oscilara tratando de centrarse

T_BOOL reiniciarPID;

void PID(void);

void PID(void) {

    T_INT dif = 0;
    T_INT error = 0;
    static T_INT P, I = 0, D;
    static T_INT difAnt = 0;

    if (reiniciarPID) {
        I = 0;
        difAnt = 0;
        reiniciarPID = 0;
    }

    //calcula la diferencia
    dif = sensorDerecha - sensorIzquierda;

    // calculo del error (se redondea)
    P = dif * KP;
    I = (I + dif) * KI;
    D = (dif - difAnt) * KD;

    difAnt = dif;

    error = round(P + I + D);
    // para mantener en memoria la dif anterior

    //recalcula las velocidades de motores
    T_INT velocidadIzquierda = constrain(VELOCIDAD_MOTORES + error, 0, VELOCIDAD_MOTORES); //velocidad izquierda
    T_INT velocidadDerecha = constrain(VELOCIDAD_MOTORES - error, 0, VELOCIDAD_MOTORES); //velcidad derecha

    pwmDuty(velocidadIzquierda, ENA);
    pwmDuty(velocidadDerecha, ENB);

}

#endif