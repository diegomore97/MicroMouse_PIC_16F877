#ifndef PWM_H
#define	PWM_H

#define BITS_PWM 1023

#define  TMR2PRESCALE 16
#define  FRECUENCIA_PWM 500

#define PIN_PWM_1 TRISC2
#define PIN_PWM_2 TRISC1

void configPwm(T_UBYTE channel);
void pwmDuty(T_UINT cicloTrabajo, T_UBYTE channel);
T_LONG map(T_LONG x, T_LONG in_min, T_LONG in_max, T_LONG out_min, T_LONG out_max);

T_LONG map(T_LONG x, T_LONG in_min, T_LONG in_max, T_LONG out_min, T_LONG out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void pwmDuty(T_UINT cicloTrabajo, T_UBYTE channel) {

    T_LONG duty = map(cicloTrabajo, 0, 100, 0, 1023);

    if (duty < 1024) {

        duty = ((T_FLOAT) duty / BITS_PWM)*(_XTAL_FREQ / (FRECUENCIA_PWM * TMR2PRESCALE)); //duty = (((float)duty/1023)*(1/PWM_freq)) / ((1/_XTAL_FREQ) * TMR2PRESCALE);

        switch (channel) {

            case 1:
                CCPR1L = duty >> 2; //PWM Ciclo Trabajo - Primeros 8 bits (MSb)
                CCP1CON &= 0xCF; //Bits 5,4  a cero (DC1B1:DC1B0 = 00)
                CCP1CON |= ((duty << 4)&0x30); //  PWM Ciclo Trabajo - Ultimos 2 bits (LSb) in CCP1CON 5,4 bits 
                break;

            case 2:
                CCPR2L = duty >> 2; //PWM Ciclo Trabajo - Primeros 8 bits (MSb)
                CCP2CON &= 0xCF; //Bits 5,4  a cero (DC1B1:DC1B0 = 00)
                CCP2CON |= ((duty << 4)&0x30); //  PWM Ciclo Trabajo - Ultimos 2 bits (LSb) in CCP1CON 5,4 bits 
                break;

        }
    }

}

void configPwm(T_UBYTE channel) {

    if (TMR2PRESCALE == 1) {
        T2CKPS0 = 0;
        T2CKPS1 = 0;
    } else if (TMR2PRESCALE == 4) {
        T2CKPS0 = 1;
        T2CKPS1 = 0;
    } else { //Prescaler 16
        T2CKPS0 = 1;
        T2CKPS1 = 1;
    }


    PR2 = (_XTAL_FREQ / (FRECUENCIA_PWM * 4 * TMR2PRESCALE)) - 1; // Configurar las fórmulas PR2 usando la Hoja de datos

    switch (channel) {

        case 1:
            PIN_PWM_1 = 0; // hacer pin de puerto en C como salida           
            CCP1M3 = 1; //Modo PWM
            CCP1M2 = 1; //Modo PWM


            break;

        case 2:
            PIN_PWM_2 = 0; // hacer pin de puerto en C como salida
            CCP2M3 = 1; //Modo PWM
            CCP2M2 = 1; //Modo PWM
            break;

    }

    TMR2ON = 1; // Configurar el módulo temporizador
}


#endif	

