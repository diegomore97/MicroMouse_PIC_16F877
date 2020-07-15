#include "config.h"
#include "pwm.h"
#include "ultrasonico.h"
#include "UART.h"
#include <stdio.h>

#define TAMANO_CADENA 50 //Tamaño de la cadena de la variable para debug
#define UMBRAL_OBSTACULO 3 //expresado en cm | sensibilidad antes de que choque con un objeto
#define UMBRAL_OBSTACULO_ENFRENTE 5 //expresado en cm | sensibilidad antes de que choque con un objeto
#define VELOCIDAD_MOTORES 80 //Porcentaje de ciclo de trabajo a la que trabajaran los motores
#define REPETICIONES_VUELTA 1 //Repeteciones para que el auto gire 90 grados

#define PIN_IN1  TRISB4
#define PIN_IN2  TRISB5
#define PIN_IN3  TRISB6
#define PIN_IN4  TRISB7

#define IN1 LATB4
#define IN2 LATB5
#define IN3 LATB6
#define IN4 LATB7

typedef enum {
    ENFRENTE = 1,
    ATRAS,
    IZQUIERDA,
    DERECHA,
    ALTO
} Direccion;

typedef struct {
    Direccion curr_state;
    Direccion Next_state;
} ComportamientoBasico;

ComportamientoBasico mouse;


char buffer[TAMANO_CADENA]; //Variable para Debug

void moverCarrito(void);

void inicializarComportamientoBasico(void);
void comportamientoBasico(void);

void inicializarComportamientoBasico(void) {

    mouse.curr_state = ENFRENTE;

    pwmDuty(VELOCIDAD_MOTORES, 1); //Iniciar ciclo de trabajo MOTOR 1
    pwmDuty(VELOCIDAD_MOTORES, 2); //Iniciar ciclo de trabajo MOTOR 2

}

void comportamientoBasico(void) {

    unsigned char contRepeticiones = 0;

    switch (mouse.curr_state) {

        case ENFRENTE:

            if (dameDistancia(ENFRENTE) < UMBRAL_OBSTACULO_ENFRENTE) //Ultrasonico ENFRENTE
                mouse.Next_state = ATRAS;
            else
                mouse.Next_state = ENFRENTE;

            break;

        case ATRAS:

            if (dameDistancia(ENFRENTE) < UMBRAL_OBSTACULO_ENFRENTE) //Ultrasonico ENFRENTE
                mouse.Next_state = ATRAS;
            else {
                if (dameDistancia(IZQUIERDA) < UMBRAL_OBSTACULO) {

                    if (dameDistancia(DERECHA) < UMBRAL_OBSTACULO) //callejon
                        mouse.Next_state = ATRAS;
                    else
                        mouse.Next_state = DERECHA;
                } else
                    mouse.Next_state = IZQUIERDA;
            }

            break;

        case IZQUIERDA:

            contRepeticiones++;

            if (contRepeticiones < REPETICIONES_VUELTA)
                mouse.Next_state = IZQUIERDA;
            else
                mouse.Next_state = ENFRENTE;

            break;

        case DERECHA:

            contRepeticiones++;

            if (contRepeticiones < REPETICIONES_VUELTA)
                mouse.Next_state = DERECHA;
            else
                mouse.Next_state = ENFRENTE;

            break;

        case ALTO:

            break;

    }

    mouse.curr_state = mouse.Next_state;
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

        case ATRAS:

            IN1 = 0;
            IN2 = 1;
            IN3 = 0;
            IN4 = 1;

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

void main(void) {

    PIN_TRIGGER = 0; //Pin Trigger Salida
    PIN_ECHO_1 = 1; //Pin Echo Entrada Sensor 1
    PIN_ECHO_2 = 1; //Pin Echo Entrada Sensor 2
    PIN_ECHO_3 = 1; //Pin Echo Entrada Sensor 3

    PIN_IN1 = 0; //Declarando como salida
    PIN_IN2 = 0; //Declarando como salida
    PIN_IN3 = 0; //Declarando como salida
    PIN_IN4 = 0; //Declarando como salida

    TRIGGER = 0; // Trigguer apagado
    IN1 = 0; //Motor 1 Apagado
    IN2 = 0; //Motor 2 Apagado
    IN3 = 0; //Motor 3 Apagado
    IN4 = 0; //Motor 4 Apagado

    T1CON = 0b00000000; // FOSC / 4 Y que el preescaler 1:1; Iniciamoa Con el TMR1ON Apagado

    configPwm(1); //Frencuencia de PWM de 500 Hz para el canal 1
    configPwm(2); //Frencuencia de PWM de 500 Hz para el canal 2

    UART_init(9600); //9600 Baudios

    inicializarComportamientoBasico();

    while (1) {

        sprintf(buffer, "\rDistancia: %d cm\r\n", dameDistancia(1));
        UART_printf(buffer);
        __delay_ms(1000);


    }
    return;
}
