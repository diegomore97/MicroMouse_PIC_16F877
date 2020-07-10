#include "config.h"
#include "pwm.h"

void main(void) {

    configPwm(1); //Frencuencia de PWM de 300 Hz para el canal 1
    configPwm(2); //Frencuencia de PWM de 300 Hz para el canal 2

    while (1) {

        for (int i = 0; i < 100; i += 10) {
            pwmDuty(i, 1); //Variando Duty Cicle del channel 1
            pwmDuty(i, 2); //Variando Duty Cicle del channel 2
        }

    }
    return;
}
