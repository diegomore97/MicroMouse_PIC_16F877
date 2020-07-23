#include "config.h"
#include "pwm.h"
#include "ultrasonico.h"
#include "UART.h"
#include <stdio.h>

#define TAMANO_CADENA 50 //Tamaño de la cadena de la variable para debug
#define UMBRAL_OBSTACULO 3 //expresado en cm | sensibilidad antes de que choque con un objeto
#define UMBRAL_OBSTACULO_ENFRENTE 5 //expresado en cm | sensibilidad antes de que choque con un objeto
#define VELOCIDAD_MOTORES 80 //Porcentaje de ciclo de trabajo a la que trabajaran los motores
#define REPETICIONES_VUELTA 5 //Repeteciones para que el auto gire 90 grados
#define DOBLE 2 //Propocional a la contante de REPETICIONES_VUELTA para que el auto gire 180 grados
#define MAX_MOVIMIENTOS_GUARDADOS 50 //Para mapear y regresar a algun lugarsi llegamos a un callejon
#define CALLEJON 0

#define PIN_IN1  TRISB4
#define PIN_IN2  TRISB5
#define PIN_IN3  TRISB6
#define PIN_IN4  TRISB7

#define IN1 LATB4
#define IN2 LATB5
#define IN3 LATB6
#define IN4 LATB7

#define ENA 1
#define ENB 2

#define PIN_BOTON_INICIO_ALTO TRISB0
#define BOTON_INICIO_ALTO PORTBbits.RB0
#define PIN_INDICADOR_ESTADO TRISDbits.RD2
#define INDICADOR_ESTADO LATD2
#define RETARDO_ANTIREBOTE 100

typedef struct {
    Direccion curr_state;
    Direccion Next_state;
} ComportamientoBasico;

typedef enum {
    SALIR_CALLEJON = 1,
    PUNTO_INICIAL
} Espejeo;

ComportamientoBasico mouse;
unsigned char pausa = 1; //Bit que indica estado del sistema

char buffer[TAMANO_CADENA]; //Variable para Debug

void moverCarrito(void);

void inicializarComportamientoBasico(void);
void comportamientoBasico(void);
void antiRebote(unsigned char pin);
void probarUltrasonico(unsigned char numeroSensor);
void probarSensores(void);
void regresarCruceAnterior(unsigned char* movimientos, unsigned char numMovimientos);
unsigned char hayCruce(void);
void limpiarMovimientos(unsigned char* movimientos, unsigned char* numMovimientos);
unsigned char decidirDireccion(void);

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
    probarUltrasonico(ENFRENTE);
    probarUltrasonico(IZQUIERDA);
    probarUltrasonico(DERECHA);
}

void probarUltrasonico(unsigned char numeroSensor) {

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

    sprintf(buffer, "\rDistancia: %d cm\r\n\n", dameDistancia(numeroSensor));
    UART_printf(buffer);
    __delay_ms(1000);

}

void antiRebote(unsigned char pin) {

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

    pwmDuty(VELOCIDAD_MOTORES, ENA); //Iniciar ciclo de trabajo MOTOR 1
    pwmDuty(VELOCIDAD_MOTORES, ENB); //Iniciar ciclo de trabajo MOTOR 2

}

void comportamientoBasico(void) {

    static unsigned char espejearCarroY = 0;
    static unsigned char movimientosRealizados[MAX_MOVIMIENTOS_GUARDADOS];
    static unsigned char contRepeticiones = 0;
    static unsigned char numMovimientos = 0;
    static unsigned char mapear = 0;
    static unsigned char contEspejeo = 0;

    switch (mouse.curr_state) {

        case ENFRENTE:

            switch (decidirDireccion()) {

                case ENFRENTE:
                    mouse.Next_state = ENFRENTE;
                    break;

                case IZQUIERDA:
                    mouse.Next_state = IZQUIERDA;
                    break;

                case DERECHA:
                    mouse.Next_state = DERECHA;
                    break;

                case CALLEJON:
                    mapear = 0;
                    espejearCarroY = 1;
                    contEspejeo++;
                    mouse.Next_state = IZQUIERDA;
                    break;

            }

            break;

        case IZQUIERDA:

            contRepeticiones++;

            if ((contRepeticiones < REPETICIONES_VUELTA) && !espejearCarroY)
                mouse.Next_state = IZQUIERDA;
            else {
                contRepeticiones = 0;
                mouse.Next_state = ENFRENTE;
            }

            if ((contRepeticiones < (REPETICIONES_VUELTA * DOBLE)) && espejearCarroY)
                mouse.Next_state = IZQUIERDA;
            else {
                //Se acabo el espejeo

                switch (contEspejeo) {

                    case SALIR_CALLEJON:
                        regresarCruceAnterior(movimientosRealizados, numMovimientos);
                        limpiarMovimientos(movimientosRealizados, &numMovimientos);
                        espejearCarroY = 1;
                        contEspejeo++;
                        mouse.curr_state = IZQUIERDA;
                        break;

                    case PUNTO_INICIAL:
                        contEspejeo = 0;
                        mouse.curr_state = ENFRENTE;
                        break;

                }

                espejearCarroY = 0;
                contRepeticiones = 0;

            }

            break;

        case DERECHA:

            contRepeticiones++;

            if (contRepeticiones < REPETICIONES_VUELTA)
                mouse.Next_state = DERECHA;
            else {
                mouse.Next_state = ENFRENTE;
                contRepeticiones = 0;
            }

            break;

        case ALTO:

            break;

    }

    mouse.curr_state = mouse.Next_state;

    if (hayCruce()) {
        mapear = 1;
    }

    if (mapear)
        movimientosRealizados[numMovimientos++] = mouse.curr_state;

    moverCarrito(); //Mandar señales al Puente H

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

void regresarCruceAnterior(unsigned char* movimientos, unsigned char numMovimientos) {

    for (int i = numMovimientos - 1; i >= 0; i--) { //Del final al Principio

        if (movimientos[i] == IZQUIERDA)
            mouse.curr_state = DERECHA;
        else if (movimientos[i] == DERECHA)
            mouse.curr_state = IZQUIERDA;
        else
            mouse.curr_state = movimientos[i];

        moverCarrito(); //Mandar señales al Puente H
    }
}

unsigned char hayCruce(void) {
    return (dameDistancia(IZQUIERDA) > UMBRAL_OBSTACULO) &&
            (dameDistancia(DERECHA) > UMBRAL_OBSTACULO);
}

void limpiarMovimientos(unsigned char* movimientos, unsigned char* numMovimientos) {
    for (int i = 0; i < *numMovimientos; i++)
        movimientos[i] = 0;

    *numMovimientos = 0;
}

unsigned char decidirDireccion(void) {

    unsigned char mayorPrioridad = ENFRENTE;
    unsigned char prioridadMedia = IZQUIERDA;
    unsigned char prioridadBaja = DERECHA;

    unsigned char direccionElegida;

    if (dameDistancia(mayorPrioridad) > UMBRAL_OBSTACULO_ENFRENTE) //No hay obstaculo             
        direccionElegida = mayorPrioridad;
    else if (dameDistancia(prioridadMedia) > UMBRAL_OBSTACULO)
        direccionElegida = prioridadMedia;
    else if (dameDistancia(prioridadBaja) > UMBRAL_OBSTACULO)
        direccionElegida = prioridadBaja;
    else
        direccionElegida = CALLEJON;

    return direccionElegida;
}

void main(void) {

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

    UART_init(9600); //9600 Baudios

    inicializarComportamientoBasico();

    while (1) {

        if (!pausa) {

            probarSensores();
            //comportamientoBasico();

        } else {

        }


    }
    return;
}
