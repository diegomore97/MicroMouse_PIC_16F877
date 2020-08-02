#include "config.h"
#include "variables.h"
#include "pwm.h"
#include "ultrasonico.h"
#include "UART.h"
#include "adc.h"
#include <stdio.h>
#include <math.h>

//**************MODIFICAR ESTAS CONSTANTES SEGUN LA CONVENIENCIA DEL PLANO****************************
#define UMBRAL_OBSTACULO_LATERAL 3 //expresado en cm | sensibilidad antes de que choque con un objeto
#define UMBRAL_OBSTACULO_ENFRENTE 5 //expresado en cm | sensibilidad antes de que choque con un objeto
#define UMBRAL_SENSOR_OPTICO_REFLEXIVO 100 //Unidad que representa el minimo de luz percibida para detectar negro
#define VELOCIDAD_MOTORES 70 //Porcentaje de ciclo de trabajo a la que trabajaran los motores
#define REPETICIONES_VUELTA 5 //Repeteciones para que el auto gire 90 grados
#define MAX_MOVIMIENTOS_GUARDADOS 50 //Para mapear y regresar a algun lugarsi llegamos a un callejon

T_FLOAT DISTANCIA_PRIORIDAD_ALTA;
T_FLOAT DISTANCIA_PRIORIDAD_MEDIA;
T_FLOAT DISTANCIA_PRIORIDAD_BAJA;
//LAS 3 PRIORIDADES DE ARRIBA DEBEN COINCIDIR CON LAS DE ABAJO
#define SENSOR_PRIORIDAD_ALTA  DERECHA
#define SENSOR_PRIORIDAD_MEDIA IZQUIERDA
#define SENSOR_PRIORIDAD_BAJA  ENFRENTE
//*****************************************************************************************************

#define PIN_IN1  TRISB4
#define PIN_IN2  TRISB5
#define PIN_IN3  TRISB6
#define PIN_IN4  TRISB7

#define IN1 LATB4
#define IN2 LATB5
#define IN3 LATB6
#define IN4 LATB7

#define PIN_BOTON_INICIO_ALTO TRISB0
#define BOTON_INICIO_ALTO PORTBbits.RB0
#define PIN_INDICADOR_ESTADO TRISDbits.RD2
#define INDICADOR_ESTADO LATD2
#define SENSOR_OPTICO_REFLEXIVO 0 //Ubicado en RA0 | canal Analogo 0

#define ENA 1
#define ENB 2

#define RETARDO_ANTIREBOTE 100
#define TAMANO_CADENA 50 //Tama�o de la cadena de la variable para debug

#define DOBLE 2 //Propocional a la contante de REPETICIONES_VUELTA para que el auto gire 180 grados
#define CALLEJON 0
#define MAX_CAMINOS 3 //Hasta 3 caminos puede tener para elegir el carrito
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

typedef struct {
    Direccion curr_state;
    Direccion Next_state;
} ComportamientoBasico;


ComportamientoBasico mouse;
T_UBYTE pausa = 1; //Bit que indica estado del sistema
T_BYTE buffer[TAMANO_CADENA]; //Variable para Debug

void moverCarrito(void);
void inicializarComportamientoBasico(void);
void comportamientoBasico(void);
void antiRebote(T_UBYTE pin);
void probarUltrasonico(T_UBYTE numeroSensor);
void probarSensores(void);
void regresarCruceAnterior(T_UBYTE* movimientos, T_UBYTE numMovimientos);
T_BOOL hayCruce(T_UBYTE* caminosRecorrer);
void limpiarMovimientos(T_UBYTE* movimientos, T_UBYTE* numMovimientos);
T_BOOL seLlegoAlDestino(void);
void leerSensores(void);
void PID(void);
void velocidadEstandar(void);
T_UBYTE decidirDireccion(T_UBYTE* caminosRecorrer, T_UBYTE* investigandoCruce,
        T_UBYTE* posicionInvCruce, T_UBYTE* contCaminosRecorridos);

void __interrupt() boton(void) {

    if (INT0IF) //Si la bandera de interrupci�n es 1
    {
        antiRebote(1); //Funcion de antirebote
        if (pausa) {
            pausa = 0;
            INDICADOR_ESTADO = 1;
            __delay_ms(3000); //3 segundos de espera para posicionar el carro
        } else {
            INDICADOR_ESTADO = 0;
            pausa = 1;
        }

        INT0IF = 0;
    }
}

void probarSensores(void) {
    probarUltrasonico(ENFRENTE);
    probarUltrasonico(IZQUIERDA);
    probarUltrasonico(DERECHA);
}

void probarUltrasonico(T_UBYTE numeroSensor) {

    switch (numeroSensor) {

        case ENFRENTE:
            UART_printf("\rEnfrente: \r\n");
            break;

        case IZQUIERDA:
            UART_printf("\rIzquierda: \r\n");
            break;

        case DERECHA:
            UART_printf("\rDerecha: \r\n");
            break;

    }

    sprintf(buffer, "\rDistancia: %.2f cm\r\n\n", dameDistancia(numeroSensor));
    UART_printf(buffer);
    __delay_ms(1000);

}

void antiRebote(T_UBYTE pin) {

    switch (pin) {
        case 1:
            while (!BOTON_INICIO_ALTO);
            while (BOTON_INICIO_ALTO);
            __delay_ms(RETARDO_ANTIREBOTE);
            break;

        default:
            break;

    }
}

void inicializarComportamientoBasico(void) {

    mouse.curr_state = ENFRENTE;

    oldSensorDerecha = dameDistancia(DERECHA);
    oldSensorIzquierda = dameDistancia(IZQUIERDA);
    oldSensorEnfrente = dameDistancia(ENFRENTE);

    velocidadEstandar();

}

void comportamientoBasico(void) {

    static T_UBYTE espejearCarroY = 0;
    static T_UBYTE movimientosRealizados[MAX_MOVIMIENTOS_GUARDADOS];
    static T_UBYTE contRepeticiones = 0;
    static T_UBYTE numMovimientos = 0;
    static T_UBYTE mapear = 0;
    static T_UBYTE cruceDetectado = 0;
    static T_UBYTE caminosRecorrer[MAX_CAMINOS];
    static T_UBYTE investigandoCruce = 0;
    static T_UBYTE posicionInvCruce = 0;
    static T_UBYTE contCaminosRecorridos = 0;
    static T_BOOL carroRotado = 0;

    leerSensores();

    switch (mouse.curr_state) {

        case ENFRENTE:

            switch (decidirDireccion(caminosRecorrer, &investigandoCruce,
                    &posicionInvCruce, &contCaminosRecorridos)) {

                case CALLEJON:
                    mapear = 0;
                    espejearCarroY = 1;
                    mouse.Next_state = IZQUIERDA;
                    break;

                case ENFRENTE:
                    PID();
                    mouse.Next_state = ENFRENTE;
                    break;

                case IZQUIERDA:
                    velocidadEstandar();
                    mouse.Next_state = IZQUIERDA;
                    break;

                case DERECHA:
                    velocidadEstandar();
                    mouse.Next_state = DERECHA;
                    break;


                case ALTO:
                    mouse.Next_state = ALTO;
                    break;

            }

            if (mapear)
                carroRotado = 1;

            break;

        case IZQUIERDA:

            contRepeticiones++;

            if ((contRepeticiones < REPETICIONES_VUELTA) && !espejearCarroY)
                mouse.Next_state = IZQUIERDA;
            else {
                carroRotado = 1;
                contRepeticiones = 0;
                mouse.Next_state = ENFRENTE;
            }

            if ((contRepeticiones < (REPETICIONES_VUELTA * DOBLE)) && espejearCarroY)
                mouse.Next_state = IZQUIERDA;
            else {
                //Se acabo el espejeo
                espejearCarroY = 0;
                contRepeticiones = 0;

                regresarCruceAnterior(movimientosRealizados, numMovimientos);
                limpiarMovimientos(movimientosRealizados, &numMovimientos);

                cruceDetectado = 0;
                posicionInvCruce = 1;
                contCaminosRecorridos++;
                mouse.curr_state = ENFRENTE;
            }

            break;

        case DERECHA:

            contRepeticiones++;

            if (contRepeticiones < REPETICIONES_VUELTA)
                mouse.Next_state = DERECHA;
            else {
                carroRotado = 1;
                contRepeticiones = 0;
                mouse.Next_state = ENFRENTE;
            }

            break;

        case ALTO:

            break;

    }

    mouse.curr_state = mouse.Next_state;

    if (hayCruce(caminosRecorrer) && !cruceDetectado) {

        if (!investigandoCruce)
            posicionInvCruce = 1;

        mapear = 1;
        cruceDetectado = 1;
        carroRotado = 0;
        investigandoCruce = 1;
    }

    if (mapear && carroRotado)
        movimientosRealizados[numMovimientos++] = mouse.curr_state;

    moverCarrito(); //Mandar se�ales al Puente H

}

void moverCarrito(void) {

    switch (mouse.curr_state) {

        case ENFRENTE:

            IN1 = 1;
            IN2 = 0;
            IN3 = 1;
            IN4 = 0;

            break;

        case IZQUIERDA:

            IN1 = 1;
            IN2 = 0;
            IN3 = 0;
            IN4 = 0;

            break;

        case DERECHA:

            IN1 = 0;
            IN2 = 0;
            IN3 = 1;
            IN4 = 0;

            break;

        case ALTO:

            IN1 = 0;
            IN2 = 0;
            IN3 = 0;
            IN4 = 0;

            break;

    }

}

void regresarCruceAnterior(T_UBYTE* movimientos, T_UBYTE numMovimientos) {

    for (int i = numMovimientos - 1; i >= 0; i--) { //Del final al Principio

        if (movimientos[i] == IZQUIERDA) {
            velocidadEstandar();
            mouse.curr_state = DERECHA;
        } else if (movimientos[i] == DERECHA) {
            velocidadEstandar();
            mouse.curr_state = IZQUIERDA;
        } else {
            PID();
            mouse.curr_state = movimientos[i];
        }

        moverCarrito(); //Mandar se�ales al Puente H
    }
}

T_BOOL hayCruce(T_UBYTE* caminosRecorrer) {

    T_UBYTE contCaminos = 0;
    T_BOOL paredEnfrente = 0, paredDerecha = 0, paredIzquierda = 0;

    if (sensorEnfrente > UMBRAL_OBSTACULO_ENFRENTE) {
        paredEnfrente = 1;
        contCaminos++;
    }

    if (sensorIzquierda > UMBRAL_OBSTACULO_LATERAL) {
        paredIzquierda = 1;
        contCaminos++;
    }

    if (sensorDerecha > UMBRAL_OBSTACULO_LATERAL) {
        paredDerecha = 1;
        contCaminos++;
    }

    if (SENSOR_PRIORIDAD_MEDIA == DERECHA) {

        if (paredDerecha)
            caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1] = 1;
        else
            caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1] = 0;

    } else if (SENSOR_PRIORIDAD_BAJA == DERECHA) {

        if (paredDerecha)
            caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] = 1;
        else
            caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] = 0;

    } else {

        if (paredDerecha)
            caminosRecorrer[SENSOR_PRIORIDAD_ALTA - 1] = 1;
        else
            caminosRecorrer[SENSOR_PRIORIDAD_ALTA - 1] = 0;

    }

    if (SENSOR_PRIORIDAD_ALTA == IZQUIERDA) {

        if (paredIzquierda)
            caminosRecorrer[SENSOR_PRIORIDAD_ALTA - 1] = 1;
        else
            caminosRecorrer[SENSOR_PRIORIDAD_ALTA - 1] = 0;

    } else if (SENSOR_PRIORIDAD_BAJA == IZQUIERDA) {

        if (paredIzquierda)
            caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] = 1;
        else
            caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] = 0;

    } else {

        if (paredIzquierda)
            caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1] = 1;
        else
            caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1] = 0;

    }

    if (SENSOR_PRIORIDAD_ALTA == ENFRENTE) {

        if (paredEnfrente)
            caminosRecorrer[SENSOR_PRIORIDAD_ALTA - 1] = 1;
        else
            caminosRecorrer[SENSOR_PRIORIDAD_ALTA - 1] = 0;

    } else if (SENSOR_PRIORIDAD_MEDIA == ENFRENTE) {

        if (paredEnfrente)
            caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1] = 1;
        else
            caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1] = 0;

    } else {

        if (paredEnfrente)
            caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] = 1;
        else
            caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] = 0;

    }


    if (contCaminos > 1)
        return 1;

    else
        return 0;

}

void limpiarMovimientos(T_UBYTE* movimientos, T_UBYTE* numMovimientos) {
    for (int i = 0; i < *numMovimientos; i++)
        movimientos[i] = 0;

    *numMovimientos = 0;
}

T_BOOL seLlegoAlDestino(void) {

    T_BOOL llegoDestino = 0;

    //Leer sensor optico
    if (dameLecturaAdc(SENSOR_OPTICO_REFLEXIVO) < UMBRAL_SENSOR_OPTICO_REFLEXIVO)
        llegoDestino = 1;

    return llegoDestino;

}

T_UBYTE decidirDireccion(T_UBYTE* caminosRecorrer, T_UBYTE* investigandoCruce,
        T_UBYTE* posicionInvCruce, T_UBYTE* contCaminosRecorridos) {

    T_UBYTE direccionElegida;
    static T_BOOL cambioOrientacionCarro = 0;


    if (*posicionInvCruce && *investigandoCruce) {

        if (*posicionInvCruce)
            *posicionInvCruce = 0;

        if (seLlegoAlDestino()) {
            direccionElegida = ALTO;

        } else {

            switch (*contCaminosRecorridos) {
                case 1:
                    if (caminosRecorrer[SENSOR_PRIORIDAD_ALTA - 1 ] == 1)
                        caminosRecorrer[SENSOR_PRIORIDAD_ALTA - 1 ] = 'X'; //Este camino no es el correcto

                    else {
                        if (caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1 ] == 1)
                            caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1 ] = 'X'; //Este camino no es el correcto  
                    }
                    break;

                case 2:
                    if (caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1 ] == 1)
                        caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1 ] = 'X'; //Este camino no es el correcto
                    else
                        caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1 ] = 'X'; //Este camino no es el correcto


                    break;

                case 3:
                    if (caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1 ] == 1)
                        caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1 ] = 'X'; //Este camino no es el correcto


                    break;

            }

        }

        if (!cambioOrientacionCarro) {

            if (caminosRecorrer[SENSOR_PRIORIDAD_ALTA - 1 ] == 1)
                direccionElegida = SENSOR_PRIORIDAD_ALTA;
            else if (caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1] == 1)
                direccionElegida = SENSOR_PRIORIDAD_MEDIA;
            else if (caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] == 1)
                direccionElegida = SENSOR_PRIORIDAD_BAJA;


            cambioOrientacionCarro = 1;

        } else {

            switch (*contCaminosRecorridos) {
                case 1:
                    if (DERECHA == SENSOR_PRIORIDAD_ALTA) {

                        if (caminosRecorrer[SENSOR_PRIORIDAD_ALTA - 1 ] == 'X') {

                            if (SENSOR_PRIORIDAD_MEDIA == IZQUIERDA &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1] == 1) {
                                direccionElegida = ENFRENTE;
                            } else if (SENSOR_PRIORIDAD_MEDIA == ENFRENTE &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1] == 1) {
                                direccionElegida = DERECHA;
                            }

                        }

                    } else if (IZQUIERDA == SENSOR_PRIORIDAD_ALTA) {

                        if (caminosRecorrer[SENSOR_PRIORIDAD_ALTA - 1 ] == 'X') {

                            if (SENSOR_PRIORIDAD_MEDIA == DERECHA &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1] == 1) {
                                direccionElegida = ENFRENTE;
                            } else if (SENSOR_PRIORIDAD_MEDIA == ENFRENTE &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1] == 1) {
                                direccionElegida = IZQUIERDA;
                            }

                        }

                    } else if (ENFRENTE == SENSOR_PRIORIDAD_ALTA) {

                        if (caminosRecorrer[SENSOR_PRIORIDAD_ALTA - 1 ] == 'X') {

                            if (SENSOR_PRIORIDAD_MEDIA == IZQUIERDA &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1] == 1) {
                                direccionElegida = DERECHA;
                            } else if (SENSOR_PRIORIDAD_MEDIA == DERECHA &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1] == 1) {
                                direccionElegida = IZQUIERDA;
                            }

                        }

                    }

                    if (DERECHA == SENSOR_PRIORIDAD_MEDIA) {

                        if (caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1 ] == 'X') {

                            if (SENSOR_PRIORIDAD_BAJA == IZQUIERDA &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] == 1) {
                                direccionElegida = ENFRENTE;
                            } else if (SENSOR_PRIORIDAD_BAJA == ENFRENTE &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] == 1) {
                                direccionElegida = DERECHA;
                            }

                        }

                    } else if (IZQUIERDA == SENSOR_PRIORIDAD_MEDIA) {

                        if (caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1 ] == 'X') {

                            if (SENSOR_PRIORIDAD_BAJA == DERECHA &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] == 1) {
                                direccionElegida = ENFRENTE;
                            } else if (SENSOR_PRIORIDAD_BAJA == ENFRENTE &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] == 1) {
                                direccionElegida = IZQUIERDA;
                            }

                        }

                    } else if (ENFRENTE == SENSOR_PRIORIDAD_MEDIA) {

                        if (caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1 ] == 'X') {

                            if (SENSOR_PRIORIDAD_BAJA == DERECHA &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] == 1) {
                                direccionElegida = IZQUIERDA;
                            } else if (SENSOR_PRIORIDAD_BAJA == IZQUIERDA &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] == 1) {
                                direccionElegida = DERECHA;
                            }

                        }

                    }

                    break;

                case 2:

                    if (DERECHA == SENSOR_PRIORIDAD_MEDIA) {

                        if (caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1 ] == 'X') {

                            if (SENSOR_PRIORIDAD_BAJA == IZQUIERDA &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] == 1) {
                                direccionElegida = ENFRENTE;
                            } else if (SENSOR_PRIORIDAD_BAJA == ENFRENTE &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] == 1) {
                                direccionElegida = DERECHA;
                            } else
                                *contCaminosRecorridos = 3;

                        }

                    } else if (IZQUIERDA == SENSOR_PRIORIDAD_MEDIA) {

                        if (caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1 ] == 'X') {

                            if (SENSOR_PRIORIDAD_BAJA == DERECHA &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] == 1) {
                                direccionElegida = ENFRENTE;
                            } else if (SENSOR_PRIORIDAD_BAJA == ENFRENTE &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] == 1) {
                                direccionElegida = IZQUIERDA;
                            } else
                                *contCaminosRecorridos = 3;

                        }

                    } else if (ENFRENTE == SENSOR_PRIORIDAD_MEDIA) {

                        if (caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1 ] == 'X') {

                            if (SENSOR_PRIORIDAD_BAJA == DERECHA &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] == 1) {
                                direccionElegida = IZQUIERDA;
                            } else if (SENSOR_PRIORIDAD_BAJA == IZQUIERDA &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] == 1) {
                                direccionElegida = DERECHA;
                            } else
                                *contCaminosRecorridos = 3;

                        }

                    }

                    break;

                case 3:
                    cambioOrientacionCarro = 0;
                    *contCaminosRecorridos = 0;
                    *investigandoCruce = 0; //Dejar de investigar y tomar otra desicion
                    direccionElegida = ALTO; //Ningun camino es la salida

                    break;
            }
        }

    } else {

        if (seLlegoAlDestino()) {

            direccionElegida = ALTO;

        } else {


            if (DISTANCIA_PRIORIDAD_ALTA > UMBRAL_OBSTACULO_ENFRENTE) //No hay obstaculo             
                direccionElegida = SENSOR_PRIORIDAD_ALTA;
            else if (DISTANCIA_PRIORIDAD_MEDIA > UMBRAL_OBSTACULO_LATERAL)
                direccionElegida = SENSOR_PRIORIDAD_MEDIA;
            else if (DISTANCIA_PRIORIDAD_BAJA > UMBRAL_OBSTACULO_LATERAL)
                direccionElegida = SENSOR_PRIORIDAD_BAJA;
            else
                direccionElegida = CALLEJON;

        }

    }

    return direccionElegida;
}

void leerSensores(void) {

    sensorDerecha = (dameDistancia(DERECHA) + oldSensorDerecha) / 2;
    sensorIzquierda = (dameDistancia(IZQUIERDA) + oldSensorIzquierda) / 2;
    sensorEnfrente = (dameDistancia(ENFRENTE) + oldSensorEnfrente) / 2;

    oldSensorDerecha = sensorDerecha;
    oldSensorIzquierda = sensorIzquierda;
    oldSensorEnfrente = sensorEnfrente;

    if (SENSOR_PRIORIDAD_MEDIA == DERECHA)
        DISTANCIA_PRIORIDAD_MEDIA = sensorDerecha;
    else if (SENSOR_PRIORIDAD_BAJA == DERECHA)
        DISTANCIA_PRIORIDAD_BAJA = sensorDerecha;
    else
        DISTANCIA_PRIORIDAD_ALTA = sensorDerecha;

    if (SENSOR_PRIORIDAD_ALTA == IZQUIERDA)
        DISTANCIA_PRIORIDAD_ALTA = sensorIzquierda;
    else if (SENSOR_PRIORIDAD_BAJA == IZQUIERDA)
        DISTANCIA_PRIORIDAD_BAJA = sensorIzquierda;
    else
        DISTANCIA_PRIORIDAD_MEDIA = sensorIzquierda;

    if (SENSOR_PRIORIDAD_ALTA == ENFRENTE)
        DISTANCIA_PRIORIDAD_ALTA = sensorEnfrente;
    else if (SENSOR_PRIORIDAD_MEDIA == ENFRENTE)
        DISTANCIA_PRIORIDAD_MEDIA = sensorEnfrente;
    else
        DISTANCIA_PRIORIDAD_BAJA = sensorEnfrente;
}

void PID(void) {

    T_BYTE dif = 0;
    T_INT error;
    static T_INT difAnt = 0;
    T_FLOAT Kp = 1.1;
    T_FLOAT Kd = 1.3;

    //calcula la diferencia
    dif = sensorIzquierda - sensorDerecha;
    // calculo del error (se redondea)
    error = round(Kp * (dif) + Kd * (difAnt - dif));
    // para mantener en memoria la dif anterior
    difAnt = dif;
    //recalcula las velocidades de motores
    T_BYTE velocidadIzquierda = constrain(VELOCIDAD_MOTORES - error, 0, VELOCIDAD_MOTORES); //velocidad izquierda
    T_BYTE velocidadDerecha = constrain(VELOCIDAD_MOTORES + error, 0, VELOCIDAD_MOTORES); //velcidad derecha

    pwmDuty(velocidadIzquierda, ENA);
    pwmDuty(velocidadDerecha, ENB);
}

void velocidadEstandar(void) {
    pwmDuty(VELOCIDAD_MOTORES, ENA);
    pwmDuty(VELOCIDAD_MOTORES, ENB);

}

void main(void) {

    //Configuraci�n de INT0
    INTCONbits.GIE = 1; //Habilitando interrupciones
    INTCONbits.INT0IE = 1; //Habilitar INT0
    INTCON2bits.INTEDG0 = 1; //Interrupci�n se activa en flanco de subida

    PIN_TRIGGER = 0; //Pin Trigger Salida
    PIN_ECHO_1 = 1; //Pin Echo Entrada Sensor 1
    PIN_ECHO_2 = 1; //Pin Echo Entrada Sensor 2
    PIN_ECHO_3 = 1; //Pin Echo Entrada Sensor 3

    PIN_IN1 = 0; //Declarando como salida
    PIN_IN2 = 0; //Declarando como salida
    PIN_IN3 = 0; //Declarando como salida
    PIN_IN4 = 0; //Declarando como salida

    PIN_INDICADOR_ESTADO = 0; //Salida
    PIN_BOTON_INICIO_ALTO = 1; //Pin como entrada

    TRIGGER = 0; // Trigguer apagado
    IN1 = 0; //Motor 1 Apagado
    IN2 = 0; //Motor 2 Apagado
    IN3 = 0; //Motor 3 Apagado
    IN4 = 0; //Motor 4 Apagado
    INDICADOR_ESTADO = 0; //Led apagado


    T1CON = 0b00000000; // FOSC / 4 Y que el preescaler 1:1; Iniciamoa Con el TMR1ON Apagado

    configPwm(1); //Frencuencia de PWM de 500 Hz para el canal 1
    configPwm(2); //Frencuencia de PWM de 500 Hz para el canal 2
    configurarAdc(); //Configurar AN0 como analogo

    UART_init(9600); //9600 Baudios

    while (1) {

        if (!pausa) {

            probarSensores();
            inicializarComportamientoBasico();
            //comportamientoBasico();

        } else {

        }


    }
    return;
}
