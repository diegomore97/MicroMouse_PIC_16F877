#ifndef PWM_H
#define	PWM_H

#define BITS_PWM 1023

#define  TMR2PRESCALE 16
#define  FRECUENCIA_PWM 500

#define PIN_PWM_1 TRISC2
#define PIN_PWM_2 TRISC1

void configPwm(unsigned char channel);
void pwmDuty(unsigned int cicloTrabajo, unsigned char channel);
long map(long x, long in_min, long in_max, long out_min, long out_max);

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void pwmDuty(unsigned int cicloTrabajo, unsigned char channel) {

    long duty = map(cicloTrabajo, 0, 100, 0, 1023);

    if (duty < 1024) {

        duty = ((float) duty / BITS_PWM)*(_XTAL_FREQ / (FRECUENCIA_PWM * TMR2PRESCALE)); // On reducing //duty = (((float)duty/1023)*(1/PWM_freq)) / ((1/_XTAL_FREQ) * TMR2PRESCALE);

        switch (channel) {

            case 1:
                CCPR1L = duty >> 2; //  PWM duty cycle - first 8-bits (MSb)
                CCP1CON &= 0xCF; //  5,4 bits zeroed (DC1B1:DC1B0 = 00)
                CCP1CON |= ((duty << 4)&0x30); //  PWM duty cycle - last 2-bits (LSb) in CCP1CON 5,4 bits 
                break;

            case 2:
                CCPR2L = duty >> 2; //  PWM duty cycle - first 8-bits (MSb)
                CCP2CON &= 0xCF; //  5,4 bits zeroed (DC1B1:DC1B0 = 00)
                CCP2CON |= ((duty << 4)&0x30); //  PWM duty cycle - last 2-bits (LSb) in CCP1CON 5,4 bits 
                break;

        }
    }

}

void configPwm(unsigned char channel) {

    if (TMR2PRESCALE == 1) {
        T2CKPS0 = 0;
        T2CKPS1 = 0;
    } else if (TMR2PRESCALE == 4) {
        T2CKPS0 = 1;
        T2CKPS1 = 0;
    } else if (TMR2PRESCALE == 16) {
        T2CKPS0 = 1;
        T2CKPS1 = 1;
    }


    PR2 = (_XTAL_FREQ / (FRECUENCIA_PWM * 4 * TMR2PRESCALE)) - 1; // Configurar las f�rmulas PR2 usando la Hoja de datos

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

    TMR2ON = 1; // Configurar el m�dulo temporizador
}


#endif	

