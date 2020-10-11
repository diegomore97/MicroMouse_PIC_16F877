#ifndef CONTROL_H
#define	CONTROL_H

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define ENA 1 //Canal de PWM que se controla, va conectado al motor Izquierdo
#define ENB 2 //Canal de PWM que se controla, va conectado al motor Derecho

//Ajustar estas variables de control para evitar chocar con las paredes laterales
#define KP 0.8 //Entre mas se aumente esta variable mas brusco sera el cambio para centrar
#define KI 0.1 //Valor para eliminar el error en estado estacionario
#define KD 1.9 //Entre mas aumente esta variabe mas oscilara tratando de centrarse

T_BOOL reiniciarPID;

void PID(void);
T_BOOL hayCruceRapidoPID(void);

void PID(void) {

    T_INT dif = 0;
    T_INT error = 0;
    static T_INT P, I = 0, D;
    static T_INT difAnt = 0;

    if (!hayCruceRapidoPID()) { //Si no hay cruce que se lleve a cabo el control PID

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

        error = P + I + D;
        // para mantener en memoria la dif anterior

        //recalcula las velocidades de motores
        T_INT velocidadIzquierda = constrain(VELOCIDAD_MOTORES + error, 0, VELOCIDAD_MOTORES); //velocidad izquierda
        T_INT velocidadDerecha = constrain(VELOCIDAD_MOTORES - error, 0, VELOCIDAD_MOTORES); //velcidad derecha

        pwmDuty(velocidadIzquierda, ENA);
        pwmDuty(velocidadDerecha, ENB);

    }
    else
    {
        
    }

}

T_BOOL hayCruceRapidoPID(void) {

    T_UBYTE contCaminos = 0;

    if (sensorEnfrente > UMBRAL_OBSTACULO_ENFRENTE_CRUCE)
        contCaminos++;
    if (sensorIzquierda > UMBRAL_OBSTACULO_LATERAL)
        contCaminos++;
    if (sensorDerecha > UMBRAL_OBSTACULO_LATERAL)
        contCaminos++;

    if (contCaminos > 1)
        return 1;
    else
        return 0;

}

#endif