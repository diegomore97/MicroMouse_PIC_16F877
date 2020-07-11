#ifndef UART_H
#define	UART_H

void UART_init(long BAUD);
unsigned char UART_read(void);
void UART_write(char dato);
void UART_printf(char* cadena);


void UART_init(long BAUD) {
    
    long frecuenciaCristal = _XTAL_FREQ;
    TRISCbits.TRISC6 = 0; //TX OUTPUT
    TRISCbits.TRISC7 = 1; //RX INPUT

    //Baudios
    SPBRG = (frecuenciaCristal / 16 / BAUD) - 1;

    //Configuración 
    TXSTAbits.BRGH = 1; //High Speed
    TXSTAbits.SYNC = 0; //Asincrono
    RCSTAbits.SPEN = 1; //Habilitar Tx y Rx

    //Transmisión
    TXSTAbits.TX9 = 0; //8 bits
    TXSTAbits.TXEN = 1; //Activamos transmisión

    //Recepción
    RCSTAbits.RC9 = 0; //8 bits
    RCSTAbits.CREN = 1; //Activamos recepción
}

unsigned char UART_read(void) {
    while (!RCIF);
    RCIF = 0;
    return RCREG;
}

void UART_write(char dato) {
    TXREG = dato;
    while (!TXSTAbits.TRMT);
}

void UART_printf(char* cadena) {
    while (*cadena) {
        UART_write(*cadena++);
    }
}

#endif	