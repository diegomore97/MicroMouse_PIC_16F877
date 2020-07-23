#ifndef UART_H
#define	UART_H

#define RX TRISC7
#define TX TRISC6

void UART_init(T_LONG BAUD);
T_UBYTE UART_read(void);
void UART_write(T_BYTE dato);
void UART_printf(T_BYTE* cadena);


void UART_init(T_LONG BAUD) {
    
    T_LONG frecuenciaCristal = _XTAL_FREQ;
    TX = 0; //TX OUTPUT
    RX = 1; //RX INPUT

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

T_UBYTE UART_read(void) {
    while (!RCIF);
    RCIF = 0;
    return RCREG;
}

void UART_write(T_BYTE dato) {
    TXREG = dato;
    while (!TXSTAbits.TRMT);
}

void UART_printf(T_BYTE* cadena) {
    while (*cadena) {
        UART_write(*cadena++);
    }
}

#endif	