#ifndef ULTRASONICO_H
#define	ULTRASONICO_H

unsigned short dameDistancia(unsigned char numeroSensor);

unsigned short dameDistancia(unsigned char numeroSensor) {

    unsigned int conteo; // Variable para poner el valor del Timer 1
    unsigned short distancia; // Variable donde se calcula la distancia recibida

    TMR1H = 0x00; // Se carga La parte Alta del Timer 1;
    TMR1L = 0x00; // Se carga la Parte Baja del Timer 1
    conteo = 0; // Garantizo que conteo sea 0;
    PORTBbits.RB0 = 1; // Activamos el Pulso en el Trigguer
    __delay_us(12); //Esperamos 12 Micro Segundos, Minimo deben ser 10.
    PORTBbits.RB0 = 0; // Apagamos el Pulso del trigguer

    switch (numeroSensor) {

        case 1:

            while (!PORTBbits.RB1); // Esperamos que el Echo se active para poder empezar la cuenta
            TMR1ON = 1; // Ponemos a contar el Timer 1
            while (PORTBbits.RB1 && !TMR1IF); // Hasta que no se apague el ECHO o Se Desborde el Timer esperamos

            break;

        case 2:

            while (!PORTBbits.RB2); // Esperamos que el Echo se active para poder empezar la cuenta
            TMR1ON = 1; // Ponemos a contar el Timer 1
            while (PORTBbits.RB2 && !TMR1IF); // Hasta que no se apague el ECHO o Se Desborde el Timer esperamos

            break;

        case 3:

            while (!PORTBbits.RB3); // Esperamos que el Echo se active para poder empezar la cuenta
            TMR1ON = 1; // Ponemos a contar el Timer 1
            while (PORTBbits.RB3 && !TMR1IF); // Hasta que no se apague el ECHO o Se Desborde el Timer esperamos

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

