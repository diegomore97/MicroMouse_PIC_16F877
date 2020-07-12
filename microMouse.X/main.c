#include "config.h"
#include "pwm.h"
#include "ultrasonico.h"
#include "UART.h"
#include <stdio.h>

#define TAMANO_CADENA 50

char buffer[TAMANO_CADENA]; //Variable para Debug

void main(void) {

    PIN_TRIGGER = 0; //Pin Trigger Salida
    PIN_ECHO_1  = 1; //Pin Echo Entrada Sensor 1
    PIN_ECHO_2  = 1; //Pin Echo Entrada Sensor 2
    PIN_ECHO_3  = 1; //Pin Echo Entrada Sensor 3

    TRIGGER = 0; // Trigguer apagado
    T1CON = 0b00000000; // FOSC / 4 Y que el preescaler 1:1; Iniciamoa Con el TMR1ON Apagado

    configPwm(1); //Frencuencia de PWM de 300 Hz para el canal 1
    configPwm(2); //Frencuencia de PWM de 300 Hz para el canal 2

    UART_init(9600); //9600 Baudios

    pwmDuty(0, 1); //Iniciar en ciclo de trabajo 0 MOTOR 1
    pwmDuty(0, 2); //Iniciar en ciclo de trabajo 0 MOTOR 2

    while (1) {

        sprintf(buffer, "\rDistancia: %d cm\r\n", dameDistancia(1));
        UART_printf(buffer);
        __delay_ms(1000);


    }
    return;
}
