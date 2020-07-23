#ifndef ULTRASONICO_H
#define	ULTRASONICO_H

#define PIN_TRIGGER TRISD3
#define PIN_ECHO_1  TRISB1
#define PIN_ECHO_2  TRISB2
#define PIN_ECHO_3  TRISB3

#define TRIGGER LATD3
#define ECHO_1  PORTBbits.RB1
#define ECHO_2  PORTBbits.RB2
#define ECHO_3  PORTBbits.RB3

typedef enum {
    ENFRENTE = 1,
    IZQUIERDA,
    DERECHA,
    ALTO
} Direccion;

T_UWORD dameDistancia(T_UBYTE numeroSensor);

T_UWORD dameDistancia(T_UBYTE numeroSensor) {

    T_UINT conteo; // Variable para poner el valor del Timer 1
    T_UWORD distancia; // Variable donde se calcula la distancia recibida

    TMR1H = 0x00; // Se carga La parte Alta del Timer 1;
    TMR1L = 0x00; // Se carga la Parte Baja del Timer 1
    conteo = 0; // Garantizo que conteo sea 0;
    TRIGGER = 1; // Activamos el Pulso en el Trigguer
    __delay_us(12); //Esperamos 12 Micro Segundos, Minimo deben ser 10.
    TRIGGER = 0; // Apagamos el Pulso del trigguer

    switch (numeroSensor) {

        case ENFRENTE: //ENFRENTE

            while (!ECHO_1); // Esperamos que el Echo se active para poder empezar la cuenta
            TMR1ON = 1; // Ponemos a contar el Timer 1
            while (ECHO_1 && !TMR1IF); // Hasta que no se apague el ECHO o Se Desborde el Timer esperamos

            break;

        case IZQUIERDA: //IZQUIERDA

            while (!ECHO_2); // Esperamos que el Echo se active para poder empezar la cuenta
            TMR1ON = 1; // Ponemos a contar el Timer 1
            while (ECHO_2 && !TMR1IF); // Hasta que no se apague el ECHO o Se Desborde el Timer esperamos

            break;

        case DERECHA: //DERECHA

            while (!ECHO_3); // Esperamos que el Echo se active para poder empezar la cuenta
            TMR1ON = 1; // Ponemos a contar el Timer 1
            while (ECHO_3 && !TMR1IF); // Hasta que no se apague el ECHO o Se Desborde el Timer esperamos

            break;

    }

    TMR1ON = 0; // Apagamos el Conteo.

    if (!TMR1IF) {
        conteo |= TMR1H << 8; // Tomamos la parte alta de Tmr1 y lo ponemos en el nibble alto de conteo
        conteo |= TMR1L; // Tomamos la parte baja de TMR1 y lo ponemos en el nibble bajo de conteo

        distancia = conteo / 58; //convirtiendo micro segundos a cm
    } else {
        conteo = 0;
        TMR1IF = 0;
        distancia = 0;
    }


    return distancia;

}


#endif	

