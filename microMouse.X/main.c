//HECHO EN CUCEI 2020

#include "config.h"
#include "variables.h"
#include "pwm.h"
#include "ultrasonico.h"
#include "UART.h"
#include "adc.h"
#include <stdio.h>
#include <math.h>

//**************MODIFICAR ESTAS CONSTANTES SEGUN LA CONVENIENCIA DEL PLANO****************************
#define UMBRAL_OBSTACULO_LATERAL 25 //expresado en cm | sensibilidad antes de que choque con un objeto
#define UMBRAL_OBSTACULO_ENFRENTE 7 //expresado en cm | sensibilidad antes de que choque con un objeto
#define UMBRAL_SENSOR_OPTICO_REFLEXIVO 100 //Unidad que representa el minimo de luz percibida para detectar negro
#define VELOCIDAD_MOTORES 100 //Porcentaje de ciclo de trabajo a la que trabajaran los motores
#define TIEMPO_REVERSA 400 //Tiempo en milisegundos que avanzara el carro en reversa
#define TIEMPO_AVANCE_IZQUIERDA 410 //Tiempo en milisegundos que avanzara el carro al girar
#define TIEMPO_AVANCE_DERECHA 410 //Tiempo en milisegundos que avanzara el carro al girar
#define MAX_MOVIMIENTOS_GUARDADOS 200 //Para mapear y regresar a algun lugar si llegamos a un callejon
#define MAX_MOVIMIENTOS_CAMINO_FINAL 1000 //El maximo de movimientos a realizar para llegar al destino

//Ajustar estas variables de control para evitar chocar con las paredes laterales
#define KP 0.9 //Entre mas se aumente esta variable mas brusco sera el cambio para centrar
#define KD 0 //Entre mas aumente esta variabe mas oscilara tratando de centrarse

//Constantes que indican a que direccion deber girar el auto
T_UBYTE SENSOR_PRIORIDAD_ALTA = ENFRENTE; //La mayor prioridad siempre debe ser enfrente (NO MODIFICAR)
T_UBYTE SENSOR_PRIORIDAD_MEDIA = IZQUIERDA;
T_UBYTE SENSOR_PRIORIDAD_BAJA = DERECHA;
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
#define TAMANO_CADENA 50 //Tamaño de la cadena de la variable para debug

#define RETARDO_PID 55
#define CALLEJON 0
#define PRIMER_DIRECCION 0
#define MAX_CAMINOS 3 //Hasta 3 caminos puede tener para elegir el carrito
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

typedef struct {
    Direccion curr_state;
    Direccion Next_state;
} ComportamientoBasico;


ComportamientoBasico mouse;
T_UBYTE pausa = 1; //Bit que indica estado del sistema
T_BYTE buffer[TAMANO_CADENA]; //Variable para Debug
T_BOOL llegoDestino;

T_FLOAT DISTANCIA_PRIORIDAD_ALTA;
T_FLOAT DISTANCIA_PRIORIDAD_MEDIA;
T_FLOAT DISTANCIA_PRIORIDAD_BAJA;

T_UBYTE posicionCarroX, posicionCarroY;
T_UBYTE posicionDestinoX, posicionDestinoY;

void moverCarrito(T_UBYTE espejearCarroY, T_UBYTE* carroEspejeado);
void mover(void);
void inicializarComportamientoBasico(void);
void comportamientoBasico(void);
void antiRebote(T_UBYTE pin);
void probarUltrasonico(T_UBYTE numeroSensor);
void probarSensores(void);
void regresarPuntoPartida(T_UBYTE* movimientos, T_UINT numMovimientos);
void regresarAlCruce(T_UBYTE* movimientos, T_UINT numMovimientos);
T_BOOL hayCruce(T_UBYTE* caminosRecorrer, T_UBYTE investigandoCruce);
void limpiarMovimientos(T_UBYTE* movimientos, T_UINT* numMovimientos);
T_BOOL seLlegoAlDestino(void);
void leerSensores(void);
void PID(void);
void velocidadEstandar(void);
void probarGirosAuto(void);
void visualizarPasosRealizados(T_UINT numMovimientos);
void recorrerCaminoEncontrado(T_UBYTE* movimientos, T_UINT numMovimientos);
void forzarParoAuto(void);
void forzarEspejeoAuto(void);
void finalizarRecorrido(void);
void probarPID(void);
void forzarReversa(void);
void forzarGiroIzquierda(void);

//Vladi
void registraPosicionActual(void); //Posicion Actual del auto
void decidirDireccionPosicion(T_UBYTE* caminosRecorrer); //Decidir direccion segun el camino que acerque mas al destino
T_BOOL caminoElegidoCorrecto(T_UBYTE investigandoCruce); //Segun si la posicion del carro se acerco al destino

void caminoCorrecto(T_UBYTE* movimientos, T_UBYTE* caminoFinal, T_UINT numMovimientos,
        T_UINT* numMovimientosFinal, T_UBYTE caminoActual);

void combinarArreglos(T_UBYTE* movimientos, T_UBYTE* caminoFinal, T_UINT numMovimientos,
        T_UINT* numMovimientosFinal);

T_UBYTE decidirDireccion(T_UBYTE* caminosRecorrer, T_UBYTE* investigandoCruce, T_UBYTE* posicionInvCruce,
        T_UBYTE* contCaminosRecorridos, T_UBYTE* caminoActual, T_BOOL* cambioOrientacionCarro);

void __interrupt() boton(void) {

    if (INT0IF) //Si la bandera de interrupción es 1
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

    leerSensores();

    probarUltrasonico(ENFRENTE);
    probarUltrasonico(IZQUIERDA);
    probarUltrasonico(DERECHA);
    T_WORD lecturaSensorOptico = dameLecturaAdc(SENSOR_OPTICO_REFLEXIVO);

    if (lecturaSensorOptico < UMBRAL_SENSOR_OPTICO_REFLEXIVO)
        UART_printf("\rSe llego al destino \r\n");
    else
        UART_printf("\rDestino no detectado \r\n");

    sprintf(buffer, "\rLectura de sensor Optico = %d\r\n\n", lecturaSensorOptico);
    UART_printf(buffer);

}

void probarUltrasonico(T_UBYTE numeroSensor) {

    switch (numeroSensor) {

        case ENFRENTE:
            sprintf(buffer, "\rEnfrente: %.2f\r\n", sensorEnfrente);
            break;

        case IZQUIERDA:
            sprintf(buffer, "\rIzquierda: %.2f\r\n", sensorIzquierda);
            break;

        case DERECHA:
            sprintf(buffer, "\rDerecha: %.2f\r\n", sensorDerecha);
            break;

    }


    UART_printf(buffer);
    __delay_ms(1000);

}

void probarGirosAuto(void) {

    for (T_INT i = 0; i < 4; i++) //Al finalizar la secuencia el auto debio girar sobre 
    { //su propopio eje
        mouse.curr_state = DERECHA;
        mover();

        mouse.curr_state = ALTO;
        mover();
        __delay_ms(1000); //Tiempo de espera entre cada giro de 90 grados
    }

    __delay_ms(3000); //Esperar para el cambio de giro en sentido opuesto

    for (T_INT i = 0; i < 4; i++) //Al finalizar la secuencia el auto debio girar sobre 
    { //su propopio eje
        mouse.curr_state = IZQUIERDA;
        mover();

        mouse.curr_state = ALTO;
        mover();
        __delay_ms(1000); //Tiempo de espera entre cada giro de 90 grados
    }

    __delay_ms(3000); //Esperar para el cambio de giro en sentido opuesto
}

void visualizarPasosRealizados(T_UINT numMovimientos) {

    switch (mouse.curr_state) {
        case ENFRENTE:
            UART_printf("Enfrente\n");
            break;

        case IZQUIERDA:
            UART_printf("Izquierda\n");
            break;

        case DERECHA:
            UART_printf("Derecha\n");
            break;

        case ALTO:
            UART_printf("Alto\n");
            break;
    }

    sprintf(buffer, "\rMovimientos Realizados = %d\r\n\n", numMovimientos);
    UART_printf(buffer);
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

void caminoCorrecto(T_UBYTE* movimientos, T_UBYTE* caminoFinal, T_UINT numMovimientos,
        T_UINT* numMovimientosFinal, T_UBYTE caminoActual) {

    movimientos[PRIMER_DIRECCION] = caminoActual;
    combinarArreglos(movimientos, caminoFinal, numMovimientos, numMovimientosFinal);
    limpiarMovimientos(movimientos, &numMovimientos);
}

void comportamientoBasico(void) {

    static T_UBYTE espejearCarroY = 0;
    static T_UBYTE carroEspejeado = 0;
    static T_UBYTE movimientosRealizados[MAX_MOVIMIENTOS_GUARDADOS];
    static T_UBYTE caminoFinal[MAX_MOVIMIENTOS_CAMINO_FINAL];
    static T_UINT numMovimientos = 0;
    static T_UINT numMovimientosTotales = 0;
    static T_UBYTE mapear = 0;
    static T_UBYTE cruceDetectado = 0;
    static T_UBYTE caminosRecorrer[MAX_CAMINOS];
    static T_UBYTE investigandoCruce = 0;
    static T_UBYTE posicionInvCruce = 0;
    static T_UBYTE contCaminosRecorridos = 0;
    static T_BOOL caminoEncontrado = 0;
    static T_UBYTE caminoActual = 0;
    static T_BOOL cambioOrientacionCarro = 0;
    T_UBYTE direccionElegida = 0;

    if (!caminoEncontrado) { //Si aun no se ha encontrado el camino

        moverCarrito(espejearCarroY, &carroEspejeado); //Mandar señales al Puente H
        //registraPosicionActual();

        if (!llegoDestino) {

            if (mapear) //Mapeo de cruces
            {
                if (numMovimientos < MAX_MOVIMIENTOS_GUARDADOS)
                    movimientosRealizados[numMovimientos++] = mouse.curr_state;
                else {
                    UART_printf("\n\nOVERFLOW en el mapeo de pasos Realizados para cruce\n\n");
                    forzarParoAuto();
                    pausa = 1;
                }

                /*if(caminoElegidoCorrecto(investigandoCruce)) //Si el camino del cruce es el correcto
                                             //Si nos acercamos mas al destino
                {
                    investigandoCruce = 0;
                    posicionInvCruce = 0;
                    mapear = 0;
                    cruceDetectado = 0;      
                                
                    contCaminosRecorridos = 0;
                    cambioOrientacionCarro = 0;
                 
                    caminoCorrecto(movimientosRealizados, caminoFinal, numMovimientos,
                            &numMovimientosTotales, caminoActual);
                    
                }
                 */

            } else { //Mapeo General
                if (!investigandoCruce) {
                    if (numMovimientosTotales < MAX_MOVIMIENTOS_CAMINO_FINAL)
                        caminoFinal[numMovimientosTotales++] = mouse.curr_state;
                    else {
                        UART_printf("\n\nOVERFLOW en el mapeo de pasos Realizados para camino Total\n\n");
                        forzarParoAuto();
                        pausa = 1;
                    }
                }
            }

        }

        leerSensores();

        if (hayCruce(caminosRecorrer, investigandoCruce) && !cruceDetectado) {

            if (!investigandoCruce)
                posicionInvCruce = 1;

            mapear = 1;
            cruceDetectado = 1;
            investigandoCruce = 1;
        }

        direccionElegida = decidirDireccion(caminosRecorrer, &investigandoCruce,
                &posicionInvCruce, &contCaminosRecorridos, &caminoActual, &cambioOrientacionCarro);


        switch (mouse.curr_state) {

            case ENFRENTE:

                switch (direccionElegida) {

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

                break;

            case IZQUIERDA:

                if (carroEspejeado && espejearCarroY) {

                    espejearCarroY = 0;
                    carroEspejeado = 0;

                    regresarAlCruce(movimientosRealizados, numMovimientos);
                    limpiarMovimientos(movimientosRealizados, &numMovimientos);

                    cruceDetectado = 0;
                    posicionInvCruce = 1;
                    contCaminosRecorridos++;
                    mouse.Next_state = ALTO;
                } else if (espejearCarroY && carroEspejeado && llegoDestino) {
                    espejearCarroY = 0;
                    mouse.Next_state = ALTO;

                } else {
                    mouse.Next_state = ENFRENTE;
                }

                break;

            case DERECHA:
                mouse.Next_state = ENFRENTE;
                break;


            case ALTO:
                if (llegoDestino && carroEspejeado) {

                    carroEspejeado = 0;
                    regresarPuntoPartida(caminoFinal, numMovimientosTotales); //Regresar al punto de partida
                    llegoDestino = 0;
                    caminoEncontrado = 1;
                    finalizarRecorrido();
                } else if (llegoDestino && !carroEspejeado) { //El auto esta parado en el destino

                    caminoCorrecto(movimientosRealizados, caminoFinal, numMovimientos,
                            &numMovimientosTotales, caminoActual);

                    espejearCarroY = 1;
                    __delay_ms(3000); //Esperar 3 segundos en la meta
                    mouse.Next_state = IZQUIERDA;
                } else {

                    if (direccionElegida == IZQUIERDA || direccionElegida == DERECHA)
                        velocidadEstandar();

                    mouse.Next_state = direccionElegida;
                }
                break;

        }

        mouse.curr_state = mouse.Next_state;

    } else { //Ya Tenemos un camino mapeado para resolver el laberinto
        recorrerCaminoEncontrado(caminoFinal, numMovimientosTotales);
        __delay_ms(3000); //Esperar 3 segundos en la meta
        forzarEspejeoAuto(); //Cuando se llega a la meta nos regresamos
        regresarPuntoPartida(caminoFinal, numMovimientosTotales); //Regresar al punto de partida
        finalizarRecorrido();
    }

}

void finalizarRecorrido(void) {
    forzarEspejeoAuto();
    forzarParoAuto();
    pausa = 1;

}

void forzarParoAuto(void) {

    IN1 = 0;
    IN2 = 0;
    IN3 = 0;
    IN4 = 0;
}

void forzarReversa(void) {
    IN1 = 0;
    IN2 = 1;
    IN3 = 0;
    IN4 = 1;

    __delay_ms(TIEMPO_REVERSA);

}

void forzarGiroIzquierda(void) {
    IN1 = 0;
    IN2 = 0;
    IN3 = 1;
    IN4 = 0;
    __delay_ms(TIEMPO_AVANCE_IZQUIERDA);

}

void forzarEspejeoAuto(void) {

    forzarParoAuto();
    velocidadEstandar();
    forzarReversa();
    forzarGiroIzquierda();
    forzarReversa();
    forzarGiroIzquierda();
}

void moverCarrito(T_UBYTE espejearCarroY, T_UBYTE* carroEspejeado) {

    switch (mouse.curr_state) {

        case ENFRENTE:

            IN1 = 1;
            IN2 = 0;
            IN3 = 1;
            IN4 = 0;

            break;

        case IZQUIERDA:

            if (espejearCarroY) {
                forzarEspejeoAuto(); //Giro 180 grados
                *carroEspejeado = 1;
            } else {
                IN1 = 0;
                IN2 = 0;
                IN3 = 1;
                IN4 = 0;
                __delay_ms(TIEMPO_AVANCE_IZQUIERDA); //Giro 90 grados

            }

            break;

        case DERECHA:

            IN1 = 1;
            IN2 = 0;
            IN3 = 0;
            IN4 = 0;

            __delay_ms(TIEMPO_AVANCE_DERECHA); //Giro 90 grados

            break;

        case ALTO:

            IN1 = 0;
            IN2 = 0;
            IN3 = 0;
            IN4 = 0;

            break;

    }

}

void mover(void) {

    switch (mouse.curr_state) {

        case ENFRENTE:

            IN1 = 1;
            IN2 = 0;
            IN3 = 1;
            IN4 = 0;

            break;

        case IZQUIERDA:

            IN1 = 0;
            IN2 = 0;
            IN3 = 1;
            IN4 = 0;

            __delay_ms(TIEMPO_AVANCE_IZQUIERDA); //Giro 90 grados

            break;

        case DERECHA:

            IN1 = 1;
            IN2 = 0;
            IN3 = 0;
            IN4 = 0;

            __delay_ms(TIEMPO_AVANCE_DERECHA); //Giro 90 grados

            break;

        case ALTO:

            IN1 = 0;
            IN2 = 0;
            IN3 = 0;
            IN4 = 0;

            break;

    }

}

void regresarPuntoPartida(T_UBYTE* movimientos, T_UINT numMovimientos) {

    for (T_INT i = numMovimientos - 1; i >= 0; i--) { //Del final al Principio

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

        mover(); //Mandar señales al Puente H
    }
}

void regresarAlCruce(T_UBYTE* movimientos, T_UINT numMovimientos) {

    for (T_INT i = numMovimientos - 1; i > 0; i--) { //Del final al Principio
        //En este caso no realizara el primer movimientos porque en ese elegimos la direccion
        //del camino del cruce que estamos explorando.

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

        mover(); //Mandar señales al Puente H
    }
}

void recorrerCaminoEncontrado(T_UBYTE* movimientos, T_UINT numMovimientos) {

    for (T_INT i = 0; i < numMovimientos; i++) { //Del Principio al Final

        if (movimientos[i] == IZQUIERDA || movimientos[i] == DERECHA)
            velocidadEstandar();

        else
            PID();

        mouse.curr_state = movimientos[i];
        mover(); //Mandar señales al Puente H
    }
}

T_BOOL hayCruce(T_UBYTE* caminosRecorrer, T_UBYTE investigandoCruce) {

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

    if (contCaminos > 1 && !investigandoCruce) {

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

        //decidirDireccionPosicion(caminosRecorrer);

    }


    if (contCaminos > 1) //Si hay mas de un camino disponible 
        return 1;

    else
        return 0;

}

void decidirDireccionPosicion(T_UBYTE* caminosRecorrer) {
    //Pal Vladi
}

T_BOOL caminoElegidoCorrecto(T_UBYTE investigandoCruce) { //Si nos acercamos mas al punto de destino
    //Pal Vladi
    T_BOOL caminoCorrecto = 0;

    return caminoCorrecto;

}

void registraPosicionActual(void) {
    //Pal Vladi
}

void limpiarMovimientos(T_UBYTE* movimientos, T_UINT* numMovimientos) {

    for (T_INT i = 0; i < *numMovimientos; i++)
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

T_UBYTE decidirDireccion(T_UBYTE* caminosRecorrer, T_UBYTE* investigandoCruce, T_UBYTE* posicionInvCruce,
        T_UBYTE* contCaminosRecorridos, T_UBYTE* caminoActual, T_BOOL* cambioOrientacionCarro) {

    T_UBYTE direccionElegida;
    llegoDestino = seLlegoAlDestino();


    if (*posicionInvCruce && *investigandoCruce) { //Estamos en el cruce que se esta investigando

        if (*posicionInvCruce)
            *posicionInvCruce = 0;

        if (llegoDestino) {
            direccionElegida = ALTO;

        } else {

            switch (*contCaminosRecorridos) { //Eliminar caminos sin salida
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

        if (!(*cambioOrientacionCarro)) { //Elegir primer Camino en un cruce

            if (caminosRecorrer[SENSOR_PRIORIDAD_ALTA - 1 ] == 1)
                direccionElegida = SENSOR_PRIORIDAD_ALTA;
            else if (caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1] == 1)
                direccionElegida = SENSOR_PRIORIDAD_MEDIA;
            else if (caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] == 1)
                direccionElegida = SENSOR_PRIORIDAD_BAJA;


            *caminoActual = direccionElegida;
            *cambioOrientacionCarro = 1;

        } else {

            switch (*contCaminosRecorridos) { //Apartir de que ya se recorrio un camino en el cruce
                case 1:
                    if (DERECHA == SENSOR_PRIORIDAD_ALTA) { //Viene de la Derecha

                        if (caminosRecorrer[SENSOR_PRIORIDAD_ALTA - 1 ] == 'X') {

                            if (SENSOR_PRIORIDAD_MEDIA == IZQUIERDA &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1] == 1) {
                                *caminoActual = IZQUIERDA;
                                direccionElegida = ENFRENTE;
                            } else if (SENSOR_PRIORIDAD_MEDIA == ENFRENTE &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1] == 1) {
                                *caminoActual = ENFRENTE;
                                direccionElegida = DERECHA;
                            } else if (SENSOR_PRIORIDAD_BAJA == IZQUIERDA &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] == 1) {
                                *caminoActual = IZQUIERDA;
                                direccionElegida = ENFRENTE;
                            } else if (SENSOR_PRIORIDAD_BAJA == ENFRENTE &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] == 1) {
                                *caminoActual = ENFRENTE;
                                direccionElegida = DERECHA;
                            }

                        }

                    } else if (IZQUIERDA == SENSOR_PRIORIDAD_ALTA) { //Viene de la Izquierda

                        if (caminosRecorrer[SENSOR_PRIORIDAD_ALTA - 1 ] == 'X') {

                            if (SENSOR_PRIORIDAD_MEDIA == DERECHA &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1] == 1) {
                                *caminoActual = DERECHA;
                                direccionElegida = ENFRENTE;
                            } else if (SENSOR_PRIORIDAD_MEDIA == ENFRENTE &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1] == 1) {
                                *caminoActual = ENFRENTE;
                                direccionElegida = IZQUIERDA;
                            } else if (SENSOR_PRIORIDAD_BAJA == DERECHA &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] == 1) {
                                *caminoActual = DERECHA;
                                direccionElegida = ENFRENTE;
                            } else if (SENSOR_PRIORIDAD_BAJA == ENFRENTE &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] == 1) {
                                *caminoActual = ENFRENTE;
                                direccionElegida = IZQUIERDA;
                            }

                        }

                    } else if (ENFRENTE == SENSOR_PRIORIDAD_ALTA) { //Viene de Enfrente

                        if (caminosRecorrer[SENSOR_PRIORIDAD_ALTA - 1 ] == 'X') {

                            if (SENSOR_PRIORIDAD_MEDIA == IZQUIERDA &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1] == 1) {
                                *caminoActual = IZQUIERDA;
                                direccionElegida = DERECHA;
                            } else if (SENSOR_PRIORIDAD_MEDIA == DERECHA &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1] == 1) {
                                *caminoActual = DERECHA;
                                direccionElegida = IZQUIERDA;
                            } else if (SENSOR_PRIORIDAD_BAJA == IZQUIERDA &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] == 1) {
                                *caminoActual = IZQUIERDA;
                                direccionElegida = DERECHA;
                            } else if (SENSOR_PRIORIDAD_BAJA == DERECHA &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] == 1) {
                                *caminoActual = DERECHA;
                                direccionElegida = IZQUIERDA;
                            }

                        }

                    }

                    if (DERECHA == SENSOR_PRIORIDAD_MEDIA) { //Viene de la Derecha

                        if (caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1 ] == 'X') {

                            if (SENSOR_PRIORIDAD_BAJA == IZQUIERDA &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] == 1) {
                                *caminoActual = IZQUIERDA;
                                direccionElegida = ENFRENTE;
                            } else if (SENSOR_PRIORIDAD_BAJA == ENFRENTE &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] == 1) {
                                *caminoActual = ENFRENTE;
                                direccionElegida = DERECHA;
                            }

                        }

                    } else if (IZQUIERDA == SENSOR_PRIORIDAD_MEDIA) { //Viene de la Izquierda

                        if (caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1 ] == 'X') {

                            if (SENSOR_PRIORIDAD_BAJA == DERECHA &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] == 1) {
                                *caminoActual = DERECHA;
                                direccionElegida = ENFRENTE;
                            } else if (SENSOR_PRIORIDAD_BAJA == ENFRENTE &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] == 1) {
                                *caminoActual = ENFRENTE;
                                direccionElegida = IZQUIERDA;
                            }

                        }

                    } else if (ENFRENTE == SENSOR_PRIORIDAD_MEDIA) { //Viene de Enfrente

                        if (caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1 ] == 'X') {

                            if (SENSOR_PRIORIDAD_BAJA == DERECHA &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] == 1) {
                                *caminoActual = DERECHA;
                                direccionElegida = IZQUIERDA;
                            } else if (SENSOR_PRIORIDAD_BAJA == IZQUIERDA &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] == 1) {
                                *caminoActual = IZQUIERDA;
                                direccionElegida = DERECHA;
                            }

                        }

                    }

                    break;

                case 2:

                    if (DERECHA == SENSOR_PRIORIDAD_MEDIA) { //Viene de la Derecha

                        if (caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1 ] == 'X') {

                            if (SENSOR_PRIORIDAD_BAJA == IZQUIERDA &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] == 1) {
                                *caminoActual = IZQUIERDA;
                                direccionElegida = ENFRENTE;
                            } else if (SENSOR_PRIORIDAD_BAJA == ENFRENTE &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] == 1) {
                                *caminoActual = ENFRENTE;
                                direccionElegida = DERECHA;
                            } else
                                *contCaminosRecorridos = 3;

                        }

                    } else if (IZQUIERDA == SENSOR_PRIORIDAD_MEDIA) { //Viene de la Izquierda

                        if (caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1 ] == 'X') {

                            if (SENSOR_PRIORIDAD_BAJA == DERECHA &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] == 1) {
                                *caminoActual = DERECHA;
                                direccionElegida = ENFRENTE;
                            } else if (SENSOR_PRIORIDAD_BAJA == ENFRENTE &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] == 1) {
                                *caminoActual = ENFRENTE;
                                direccionElegida = IZQUIERDA;
                            } else
                                *contCaminosRecorridos = 3;

                        }

                    } else if (ENFRENTE == SENSOR_PRIORIDAD_MEDIA) { //Viene de Enfrente

                        if (caminosRecorrer[SENSOR_PRIORIDAD_MEDIA - 1 ] == 'X') {

                            if (SENSOR_PRIORIDAD_BAJA == DERECHA &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] == 1) {
                                *caminoActual = DERECHA;
                                direccionElegida = IZQUIERDA;
                            } else if (SENSOR_PRIORIDAD_BAJA == IZQUIERDA &&
                                    caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1] == 1) {
                                *caminoActual = IZQUIERDA;
                                direccionElegida = DERECHA;
                            } else
                                *contCaminosRecorridos = 3;

                        }

                    } else if (DERECHA == SENSOR_PRIORIDAD_BAJA) { //Viene de la Derecha

                        if (caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1 ] == 'X')
                            *contCaminosRecorridos = 3;
                    } else if (IZQUIERDA == SENSOR_PRIORIDAD_BAJA) {

                        if (caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1 ] == 'X')
                            *contCaminosRecorridos = 3;
                    } else if (ENFRENTE == SENSOR_PRIORIDAD_BAJA) {

                        if (caminosRecorrer[SENSOR_PRIORIDAD_BAJA - 1 ] == 'X')
                            *contCaminosRecorridos = 3;

                    }

                    break;

                case 3:
                    *cambioOrientacionCarro = 0;
                    *contCaminosRecorridos = 0;
                    *investigandoCruce = 0; //Dejar de investigar y tomar otra desicion
                    direccionElegida = ALTO; //Ningun camino es la salida

                    break;
            }
        }

    } else { //Ya no estamos en el cruce que se esta investigando

        if (llegoDestino) {
            direccionElegida = ALTO;

        } else { //Elegir el camino cuando no se esta investigando un cruce


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

    T_INT dif = 0;
    T_INT error = 0;
    static T_INT difAnt = 0;

    //calcula la diferencia
    dif = sensorIzquierda - sensorDerecha;
    // calculo del error (se redondea)
    error = round(KP * (dif) + KD * (difAnt - dif));
    // para mantener en memoria la dif anterior
    difAnt = dif;
    //recalcula las velocidades de motores
    T_INT velocidadIzquierda = constrain(VELOCIDAD_MOTORES - error, 0, VELOCIDAD_MOTORES); //velocidad izquierda
    T_INT velocidadDerecha = constrain(VELOCIDAD_MOTORES + error, 0, VELOCIDAD_MOTORES); //velcidad derecha

    pwmDuty(velocidadIzquierda, ENA);
    pwmDuty(velocidadDerecha, ENB);
}

void probarPID(void) {

    leerSensores();

    if (sensorEnfrente > UMBRAL_OBSTACULO_ENFRENTE) { //Hay espacio hacia enfrente?
        PID();
        mouse.curr_state = ENFRENTE;
        mover();
    } else
        finalizarRecorrido();

    __delay_ms(RETARDO_PID); //Tiempo que el carro avanzara, si estas imprimiendo via UART
    //al mismo tiempo, comenta este delay

}

void velocidadEstandar(void) {

    pwmDuty(VELOCIDAD_MOTORES, ENA);
    pwmDuty(VELOCIDAD_MOTORES, ENB);

}

void combinarArreglos(T_UBYTE* movimientos, T_UBYTE* caminoFinal, T_UINT numMovimientos,
        T_UINT* numMovimientosFinal) {
    for (T_INT i = 0; i < numMovimientos; i++) {

        caminoFinal[*numMovimientosFinal] = movimientos[i];
        *numMovimientosFinal += 1;
    }
}

void main(void) {

    T_BOOL iniciado = 0;
    T_INT numMovimientosTotales = 0;

    //Configuración de INT0
    INTCONbits.GIE = 1; //Habilitando interrupciones
    INTCONbits.INT0IE = 1; //Habilitar INT0
    INTCON2bits.INTEDG0 = 1; //Interrupción se activa en flanco de subida

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

            if (!iniciado) {
                iniciado = 1;
                inicializarComportamientoBasico();
            }

            //probarSensores();
            //probarGirosAuto();
            probarPID();
            //visualizarPasosRealizados(numMovimientosTotales++); //Para visualizarlo por Bluetooth
            //comportamientoBasico();
            forzarParoAuto();

        } else {

            iniciado = 0;

        }


    }
    return;
}
