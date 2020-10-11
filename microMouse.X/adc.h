#ifndef ADC_H
#define	ADC_H

T_WORD dameLecturaAdc(T_BYTE canalLeer);
void configurarAdc(void);

void configurarAdc(void) {
    ADCON1 = 0b00001011; //VSS REFERENCIA|CANAL 0,1,2,3 COMO ANALOGO
    ADCON2 = 0b10100101; //TIEMPO DE ADQUISICION 8 TAD, JUSTIFICADO A LA DERECHA, FOSC/16
}

T_WORD dameLecturaAdc(T_BYTE canalLeer) {

    __delay_us(20);

    ADCON0bits.ADON = 1; //INICIAR ADC
    ADCON0bits.CHS = canalLeer;
    ADCON0bits.GO = 1;
    ADCON0bits.GO_DONE = 1; //Bandera en 1

    while (ADCON0bits.GO_DONE);

    ADCON0bits.ADON = 0; //APAGAR ADC

    return (ADRESH << 8) +ADRESL;

}


#endif	