#ifndef CONSTANTESIMPORTANTES_H
#define	CONSTANTESIMPORTANTES_H

//**************MODIFICAR ESTAS CONSTANTES SEGUN LA CONVENIENCIA DEL PLANO, 
//DIMENSIONES Y PESO DEL CARRO ****************************

#define DIFERENCIA_SENSOR_LLANTAS 1.7 //Diferencia en cm del sensor para detectar obstaculos a la llanta
#define ETAPA_PRUEBAS 0 //Habilitar cuando se ejecute una funcion de pruebas
#define MOSTRAR_INFORMACION_UART 0 //Habilita y deshabilita la comunicacion via UART
#define UMBRAL_OBSTACULO_LATERAL 30 //expresado en cm | sensibilidad antes de que choque con un objeto
#define UMBRAL_OBSTACULO_ENFRENTE_CRUCE 35//expresado en cm | sensibilidad antes de que choque con un objeto
#define UMBRAL_OBSTACULO_ENFRENTE_PID 14//expresado en cm | sensibilidad antes de que choque con un objeto
#define UMBRAL_SENSOR_OPTICO_REFLEXIVO 20 //Unidad que representa el minimo de luz percibida para detectar negro
#define RETARDO_PARO_AUTO 30 //Tiempo que se quedara parado el auto
#define RETARDO_PID 55 //Tiempo que avanzara el carrito en linea recta cuando esta por control PID 
#define VELOCIDAD_MOTORES 100 //Porcentaje de ciclo de trabajo a la que trabajaran los motores
#define VELOCIDAD_MOTORES_BAJA 70 //Porcentaje de ciclo de trabajo a la que trabajaran los motores
#define TIEMPO_REVERSA 400 //Tiempo en milisegundos que avanzara el carro en reversa
#define TIEMPO_AVANCE_LATERAL 410 //Tiempo en milisegundos que avanzara el carro al girar
#define TIEMPO_AVANCE_LATERAL_MINIMO 50 //Para cuando queramos alinear despues de llegar a un cruce
#define TIEMPO_AVANCE_RECTO 550 //Tiempo en milisegundos que avanzara el carro en linea recta
#define RETARDO_MOV_ESPEJEO 100
#define MAX_MOVIMIENTOS_GUARDADOS 200 //Para mapear y regresar a algun lugar si llegamos a un callejon
#define MAX_MOVIMIENTOS_CAMINO_FINAL 20 //El maximo de movimientos a realizar para llegar al destino

//Constantes que indican a que direccion deber girar el auto
T_UBYTE SENSOR_PRIORIDAD_ALTA = ENFRENTE; //La mayor prioridad siempre debe ser enfrente (NO MODIFICAR)
T_UBYTE SENSOR_PRIORIDAD_MEDIA = IZQUIERDA;
T_UBYTE SENSOR_PRIORIDAD_BAJA = DERECHA;
//*****************************************************************************************************


#endif


