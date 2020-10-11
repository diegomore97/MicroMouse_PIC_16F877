#ifndef SENSORINFRARROJOIR_H
#define	SENSORINFRARROJOIR_H

typedef enum {
    ENFRENTE = 1,
    IZQUIERDA,
    DERECHA,
    ALTO
} Direccion;

T_FLOAT sensorDerecha, sensorIzquierda, sensorEnfrente;
T_FLOAT oldSensorDerecha = 0, oldSensorIzquierda = 0, oldSensorEnfrente = 0;

T_FLOAT dameDistancia(T_UBYTE numeroSensor);
T_FLOAT myPow(T_FLOAT x, T_FLOAT y);

T_FLOAT myPow(T_FLOAT x, T_FLOAT y) {
    if (x >= 0)
        return ( exp(y * log(x)));
    else
        if ((int) y % 2) {
        return ( -exp(y * log(-x)));
    } else {
        return ( exp(y * log(-x)));
    }
}

T_FLOAT dameDistancia(T_UBYTE numeroSensor) {

    T_FLOAT distancia; // Variable donde se calcula la distancia recibida
    distancia = myPow(3027.4 / dameLecturaAdc(numeroSensor), 1.2134); // conversión a centímetros

    return distancia;

}



#endif