/*
 * libreria.c
 *
 *  Created on: 3 mayo. 2022
 *  Author: maria
 *
 **/

#include <libreria.h>
#include <msp432p401r.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "lib_PAE.h"


void init_uart(void)
{
#ifdef EMULADA
    UCA0CTLW0 |= UCSWRST;                   //hacemos un reset de la USCI, desactiva la USCI
    UCA0CTLW0 |= UCSSEL__SMCLK;             //seleccionamos la fuente de reloj SMCLK
    UCA0MCTLW = UCOS16;                     //oversampling habilitado
    UCA0BRW = 3;                            //baud rate = 500000b/s
    P1SEL0 |= BIT2 | BIT3;                  //configuramos los pines de la UART
    P1SEL1 &= ~ (BIT2 | BIT3);              //P1.3 = UART0TX, P1.2 = UART0RX
    UCA0CTLW0 &= ~UCSWRST;                  //reactivamos la línea de comunicaciones serie
    EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;    //limpiamos el flag de interrupciones de la UART
    EUSCI_A0->IE |= EUSCI_A_IE_RXIE;        //habilitamos a nivel de dispositivo las interrupciones de USCI_A0 RX
                                            //cuando tengamos la recepción
#endif

#ifdef REAL
    UCA2CTLW0 |= UCSWRST;                   //hacemos un reset de la USCI, desactiva la USCI
    UCA2CTLW0 |= UCSSEL__SMCLK;             //seleccionamos la fuente de reloj SMCLK
    UCA2MCTLW = UCOS16;                     //oversampling habilitado
    UCA2BRW = 3;                            //baud rate = 500000b/s
    P3SEL0 |= BIT2 | BIT3;                  //configuramos los pines de la UART
    P3SEL1 &= ~ (BIT2 | BIT3);              //P3.3 = UART2TX, P3.2 = UART2RX
    UCA2CTLW0 &= ~UCSWRST;                  //reactivamos la línea de comunicaciones serie
    EUSCI_A2->IFG &= ~EUSCI_A_IFG_RXIFG;    //limpiamos el flag de interrupciones de la UART
    EUSCI_A2->IE |= EUSCI_A_IE_RXIE;        //habilitamos a nivel de dispositivo las interrupciones de USCI_A2 RX
                                            //cuando tengamos la recepción
#endif
    P3SEL0 &= ~BIT0;                        //DIRECTION PORT es un GPIO
    P3SEL1 &= ~BIT0;                        //DIRECTION PORT es un GPIO
    P3DIR |= BIT0;                          //DIRECTION PORT es un salida
    P3OUT &= ~BIT0;                         //DIRECTION PORT en un principio indica que se van a recibir bytes
}

//funciones para cambiar el sentido de las comunicaciones
//configuracion del Half Duplex de los motores: recepción
void sentidoDatos_RX(void)
{
    P3OUT &= ~BIT0;                         //DIRECTION PORT indica que se van a recibir bytes
}

//configuracion del Half Duplex de los motores: transmisión
void sentidoDatos_TX(void)
{
    P3OUT |= BIT0;                          //DIRECTION PORT indica que se van a transmitir bytes
}

void init_interrupciones()
{
    //interrupcuiones en el puerto 5
    NVIC->ICPR[1] |= (BIT7);
    NVIC->ISER[1] |= (BIT7);

    //interrupciones en el puerto 4
    NVIC->ICPR[1] |= (BIT6);
    NVIC->ISER[1] |= (BIT6);

    //interrupciones en el puerto 3
    //Int. puerto 3 = 37 corresponde al bit 5 del segundo registro ISER1
    NVIC->ICPR[1] |= (BIT5); //Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para este puerto
    NVIC->ISER[1] |= (BIT5); //y habilito las interrupciones de los puertos

    //timer A0
    //Int. timer A0 corresponde al bit 8 del primer registro ISER0
    NVIC->ICPR[0] |= 1 << 8; //Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para el timer
    NVIC->ISER[0] |= 1 << 8; //y habilito las interrupciones del timer

    //timer A1
    //Int. timer A1 corresponde al bit 10 del primer registro ISER0
    NVIC->ICPR[0] |= 1 << 10; //Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para el timer
    NVIC->ISER[0] |= 1 << 10; //y habilito las interrupciones del timer

    //ACTIVAR INTERRUPCION UART
#ifdef EMULADA
    //eUSCI_A0
    //Int. eUSCI_A0 corresponde al bit 16 del primer registro ISER0
    NVIC->ICPR[0] |= 1 << EUSCIA0_IRQn; //Primero, me aseguro de que no quede ninguna interrupcion residual pendiente
    NVIC->ISER[0] |= 1 << EUSCIA0_IRQn; //y habilito las interrupciones
#endif
#ifdef REAL
    //eUSCI_A2
    //Int. eUSCI_A2 corresponde al bit 18 del primer registro ISER0
    NVIC->ICPR[0] |= 1 << EUSCIA2_IRQn; //Primero, me aseguro de que no quede ninguna interrupcion residual pendiente
    NVIC->ISER[0] |= 1 << EUSCIA2_IRQn; //y habilito las interrupciones
#endif
}


#ifdef EMULADA
//función que envía un byte de datos por la UART 0
void TxUAC0(uint8_t bTxdData)
{
    while(!TXD0_READY);     //espera a que esté preparado el buffer de transmisión
    UCA0TXBUF = bTxdData;   //ponemos el dato en el buffer de transmisión
}

#endif
#ifdef REAL
//función que envía un byte de datos por la UART 2
void TxUAC2(uint8_t bTxdData)
{
    while(!TXD2_READY);     //espera a que esté preparado el buffer de transmisión
    UCA2TXBUF = bTxdData;   //ponemos el dato en el buffer de transmisión
}
#endif

//INSTRUCTION PACKET
byte TxPacket(byte bID, byte bParameterLength, byte bInstruction, byte Parametros[16])
{
    byte bCount,bCheckSum,bPacketLength;
    byte TxBuffer[32];
    char error[] = "adr. no permitida";
    if ((Parametros[0] < 6) && (bInstruction == 3)){        //si se intenta escribir en una dirección <0x05
          halLcdPrintLine(error, 8, INVERT_TEXT);           //se emite un mensaje de error de dirección prohibida
          return 0;                                         //y se sale de la función
    }
    sentidoDatos_TX();                                      //TRANSMISIÓN de datos
TxBuffer[0] = 0xff;                                         //los primeros dos bytes indican el inicio de la trama y siempre son 0xff 0xff
    TxBuffer[1] = 0xff;
    TxBuffer[2] = bID;                                      //ID del módulo al que queremos enviar el mensaje
    TxBuffer[3] = bParameterLength+2;                       //longitud del paquete
    TxBuffer[4] = bInstruction;                             //instrucción que enviamos al módulo
    for(bCount = 0; bCount < bParameterLength; bCount++)    //emepzamos a generar la trama que quermeos enviar
    {
        TxBuffer[bCount+5] = Parametros[bCount];
    }
    bCheckSum = 0;
    bPacketLength = bParameterLength+4+2;
    for(bCount = 2; bCount < bPacketLength-1; bCount++)     //calculamos el checksum
    {
        bCheckSum += TxBuffer[bCount];
    }
    TxBuffer[bCount] = ~bCheckSum;                          //escribimos el checksum (complemetno a1)
    for(bCount = 0; bCount < bPacketLength; bCount++)       //bucle que envía la trama
    {
#ifdef EMULADA
        TxUAC0(TxBuffer[bCount]);                           //pone el dato en el buffer de transmisión
        while((UCA0STATW & UCBUSY));                        //espera a que esté preparado el buffer de transmisión
#endif
#ifdef REAL
        TxUAC2(TxBuffer[bCount]);                           //pone el dato en el buffer de transmisión
        while((UCA2STATW & UCBUSY));                        //espera a que esté preparado el buffer de transmisión
#endif
    }
    sentidoDatos_RX();                                      //ponemos la línea de dayos en Rx para que el módulo dynamixel envíe respuesta
    return(bPacketLength);
}


//STATUS PACKET
struct RxReturn RxPacket(void)
{
    struct RxReturn respuesta;
    byte bCount, bLenght, bChecksum, bPacketLength;
    byte Rx_time_out=0;

    sentidoDatos_RX();                                      //ponemos la línea half duplex en Rx
    Activa_TimerA0_TimeOut();

    for(bCount = 0; bCount < 4; bCount++)                   //leemos primero los 4 primero bytes del status packet ya que
    {                                                       //el cuarto byte indica cuantos bytes quedan por leer
        Reset_Timeout();                                    //reseteamos el timeout cada vez que leemos un dato nuevo
        Byte_Recibido = No;
        while (!Byte_Recibido)                              //esperamos a recibir el byte
        {
            Rx_time_out=TimeOut(1000);
            if (Rx_time_out)break;                          //si ha habido timeout salimos del bucle
        }
        if (Rx_time_out)break;                              //si ha habido timeout salimos del bucle
        respuesta.StatusPacket[bCount] = DatoLeido_UART;    //si no, leemos un dato
    }

    if (!Rx_time_out){                                      //si no ha habido timeout:
        bLenght = DatoLeido_UART;
        for(bCount = 0; bCount < bLenght; bCount++){        //leemos los bytes que nos quedan por leer
            Reset_Timeout();                                //reseteamos el timeout cada vez que leemos un dato nuevo
            Byte_Recibido=No;
            while (!Byte_Recibido)                          //esperamos a recibir el byte
            {
                Rx_time_out=TimeOut(1000);
                if (Rx_time_out)break;                      //si ha habido timeout salimos del bucle
            }
            if (Rx_time_out)break;                          //si ha habido timeout salimos del bucle
            respuesta.StatusPacket[bCount+4] = DatoLeido_UART; //si no, leemos un dato
        }
        bChecksum = 0;
        bPacketLength = bLenght+4;
        for(bCount = 2; bCount < bPacketLength-1; bCount++) //calculamos el checksum sumando todos los bytes menos el inicio de la trama
        {                                                   //y el CS para ver si coincide con el enviado en el Status Packet
            bChecksum += respuesta.StatusPacket[bCount];
        }
        respuesta.StatusPacket[bCount] = ~ bChecksum;       //guardamos el CS (complemento a1)
        respuesta.checkSum = respuesta.StatusPacket[bLenght+3] != respuesta.StatusPacket[bCount];   //compramos si el CS calculado y el recibido coinciden

    }
    if(Rx_time_out){                                        //si ha habido algún timeout
        respuesta.timeout = Si;                             //lo indicamos
    }
    Stop_Timeout();                                         //detenemos el timer A0
    return respuesta;
}

#ifdef EMULADA
//gestión de la interrupción de recepción en la UART A0
void EUSCIA0_IRQHandler(){
    UCA0IE &= ~UCRXIE;                                      //interrupciones desactivadas en Rx
    EUSCI_A0->IFG &=~ EUSCI_A_IFG_RXIFG;                    //limpiamos el flag de interrupciones
    DatoLeido_UART = UCA0RXBUF;                             //leemos el dato recibido del buffer de recepción
    Byte_Recibido=Si;                                       //indicamos que se ha recibido el byte
    UCA0IE |= UCRXIE;                                       //interrupciones reactivadas en Rx
}
#endif

#ifdef REAL
//gestión de la interrupción de recepción en la UART A2
void EUSCIA2_IRQHandler()
{
    UCA2IE &= ~UCRXIE;                                      //interrupciones desactivadas en Rx
    EUSCI_A2->IFG &=~ EUSCI_A_IFG_RXIFG;                    //limpiamos el flag de interrupciones
    DatoLeido_UART = UCA2RXBUF;                             //leemos el dato recibido del buffer de recepción
    Byte_Recibido=Si;                                       //indicamos que se ha recibido el byte
    UCA2IE |= UCRXIE;                                       //interrupciones reactivadas en Rx

}
#endif



//transacción instruccion packet - status packet
struct RxReturn RxTxPacket(byte bID, byte bParameterLength, byte bInstruction, byte Parametros[16])
{
    struct RxReturn respuesta;
    TxPacket(bID, bParameterLength, bInstruction, Parametros);  //enviamos un Instruction Packet
    respuesta = RxPacket();                                     //recibimos un Status Packet
    return respuesta;
}


void Activa_TimerA0_TimeOut()
{
    TA0CTL |= MC_1;     //el timer A0 cuenta de forma cíclica hasta TA0CCR0
    tiempo=0;           //inicializamos el tiempo a 0
}

void Reset_Timeout()
{
    TA0CTL &= ~MC_0;    //el timer A0 está parado
    TA0CTL |= MC_1;     //el timer A0 cuenta de forma cíclica hasta TA0CCR0
    tiempo=0;           //reseteamos el tiempo a 0
}

byte TimeOut(uint16_t t)
{
    return tiempo >= t; //comprobamos si ha pasado demasiado tiempo comparando el valor
                        //pasado por parámetro con la variable global tiempo que se incrementa
                        //en uno en cada interrupción del timer A0
}

void Stop_Timeout()
{
    TA0CTL &= ~MC_0;    //el timer A0 está parado
    tiempo=0;           //reseteamos el tiempo a 0
}

void init_timers()
{
    //Timer A0
    TIMER_A0->CTL = TIMER_A_CTL_ID__1 | TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_CLR
                | TIMER_A_CTL_MC__UP;

    //TA0CTL = xxxxxx100001x1xx
    //BIT2 = 1 (CLR) indica que el timer se resetea
    //BIT5 BIT4 = 01 (MC) indica que el timer funciona en modo UP, es decir, indica que
    //el timer cuneta de forma ciclica desde 0 hasta TA0CCR0
    //BIT7 BIT6 = 00 (ID) indica que que la fuente del reloj es dividida por 1
    //BIT9 BIT8 = 10 (TASSEL) indica que la fuente del timer es SMCLK

    TIMER_A0->CCR[0] = 2400 - 1;
    //SMCLK trabaja a 24MHz (24*10^6), por lo que de esta manera generamos una base
    //de tiempo de 10kHz (24000000/10000 = 2400)

    TIMER_A0->CCTL[0] |= TIMER_A_CCTLN_CCIE;
    //Interrupciones activadas en CCR0


    //Timer A1

    TIMER_A1->CTL = TIMER_A_CTL_ID__1 | TIMER_A_CTL_SSEL__ACLK | TIMER_A_CTL_CLR
                | TIMER_A_CTL_MC__UP;
    //TA1CTL = xxxxxx010001x1xx
    //BIT2 = 1 (CLR) indica que el timer se resetea
    //BIT5 BIT4 = 01 (MC) indica que el timer funciona en modo UP, es decir, indica que
    //el timer cuneta de forma ciclica desde 0 hasta TA1CCR0
    //BIT7 BIT6 = 00 (ID) indica que que la fuente del reloj es dividida por 1
    //BIT9 BIT8 = 01 (TASSEL) indica que la fuente del timer es ACLK

    TIMER_A1->CCR[0] = (1 << 15) - 1;
    //ACLK trabaja a 32768Hz por lo que de esta manera generamos una base de tiempo
    //de 1Hz ya que 32768/32768 = 1

    TIMER_A1->CCTL[0] |= TIMER_A_CCTLN_CCIE;
    //Interrupciones activadas en CCR0
}

//gestión de interrupción del timer A0
void TA0_0_IRQHandler(void)
{
    //TODO
    TA0CCTL0 &= ~CCIE;                  //desactivamos las interrupciones del timer A0
    TA0CCTL0 &= ~CCIFG;                 //limpiamos el flag de interrupción

    tiempo++;                           //incrementamos en uno la variable global tiempo

    TA0CCTL0 |= CCIE;                   //activo laa interrupciones del timer A0
}


//LEDs
void encender_LED_derecho(void) {
    byte parametros[2] = {REG_LED, 0x01};
    RxTxPacket(MOTOR_RIGHT, 2, INSTR_WRITE, parametros);
}

void encender_LED_izquierdo(void) {
    byte parametros[2] = {REG_LED, 0x01};
    RxTxPacket(MOTOR_LEFT, 2, INSTR_WRITE, parametros);
}

void encender_LEDs(void) {
    encender_LED_derecho();
    encender_LED_izquierdo();
}

void apagar_LED_derecho(void) {
    byte parametros[2] = {REG_LED, 0x00};
    RxTxPacket(MOTOR_RIGHT, 2, INSTR_WRITE, parametros);
}

void apagar_LED_izquierdo(void) {
    byte parametros[2] = {REG_LED, 0x00};
    RxTxPacket(MOTOR_LEFT, 2, INSTR_WRITE, parametros);
}

void apagarLEDs(void) {
    apagar_LED_derecho();
    apagar_LED_izquierdo();
}

void leer_LED_derecho(byte *readVal) {
    struct RxReturn respuesta;
    byte parametros[2] = {REG_LED, 0x01};
    respuesta = RxTxPacket(MOTOR_RIGHT, 2, INSTR_READ, parametros);
    *readVal = respuesta.StatusPacket[5];
}

void leer_LED_izquierdo(byte *readVal) {
    struct RxReturn respuesta;
    byte parametros[2] = {REG_LED, 0x01};
    respuesta = RxTxPacket(MOTOR_LEFT, 2, INSTR_READ, parametros);
    *readVal = respuesta.StatusPacket[5];
}



//movimiento
void setEndlessTurn(void)
{
    byte parametros[5] = {REG_CW_L, 0x00, 0x00, 0x00, 0x00};
    RxTxPacket(MOTOR_RIGHT, 5, INSTR_WRITE, parametros);
    RxTxPacket(MOTOR_LEFT, 5, INSTR_WRITE, parametros);
}

//la idea es que una rueda vaya hacia delante y otra hacia atrás ya que al
//estar contrapuestas es como si las 2 fueran hacia la misma dirección, además la
//diferencia de valores en el último parámetro se debe a que queremos evitar que el robot
//pivote sobre un eje, que es lo que pasaría en caso de que tuvieran el mismo valor

void delante(void)
{
    byte parametros_right[3] = {REG_GOAL_SPEED_L, 0xff, 0x05};
    RxTxPacket(MOTOR_RIGHT, 3, INSTR_WRITE, parametros_right);
    byte parametros_left[3] = {REG_GOAL_SPEED_L, 0xff, 0x01};
    RxTxPacket(MOTOR_LEFT, 3, INSTR_WRITE, parametros_left);
}

//función contraria a la anterior, lo que hacia la rueda derecha antes lo hace ahora la
//izquierda, y viceversa
void atras(void)
{
    byte parametros_right[3] = {REG_GOAL_SPEED_L, 0xff, 0x01};
    RxTxPacket(MOTOR_RIGHT, 3, INSTR_WRITE, parametros_right);
    byte parametros_left[3] = {REG_GOAL_SPEED_L, 0xff, 0x05};
    RxTxPacket(MOTOR_LEFT, 3, INSTR_WRITE, parametros_left);
}

//setear a 0 la velocidad de giro de las ruedas
void parar()
{
    byte parametros[3] = {REG_GOAL_SPEED_L, 0x00, 0x00};
    RxTxPacket(MOTOR_RIGHT, 3, INSTR_WRITE, parametros);
    RxTxPacket(MOTOR_LEFT, 3, INSTR_WRITE, parametros);
}

void delanteV(byte velocidad)
{
    if (velocidad < 1024) {
        byte parametros_right[3] = {REG_GOAL_SPEED_L, velocidad, 0x05};
        RxTxPacket(MOTOR_RIGHT, 3, INSTR_WRITE, parametros_right);
        byte parametros_left[3] = {REG_GOAL_SPEED_L, velocidad, 0x01};
        RxTxPacket(MOTOR_LEFT, 3, INSTR_WRITE, parametros_left);
    }
}

void atrasV(byte velocidad)
{
    if (velocidad < 1024) {
        byte parametros_right[3] = {REG_GOAL_SPEED_L, velocidad, 0x01};
        RxTxPacket(MOTOR_RIGHT, 3, INSTR_WRITE, parametros_right);
        byte parametros_left[3] = {REG_GOAL_SPEED_L, velocidad, 0x05};
        RxTxPacket(MOTOR_LEFT, 3, INSTR_WRITE, parametros_left);
    }
}

//giro
//la idea es que la rueda de la derecha vaya a mayor velocidad que la rueda de la izquierda
//y en sentido contrario(porque están contrapuestas)
void giroDerecha(void)
{
    byte parametros[3] = {REG_GOAL_SPEED_L, 0xff, 0x05};
    RxTxPacket(MOTOR_RIGHT, 3, INSTR_WRITE, parametros);
    byte parametros_[3] = {REG_GOAL_SPEED_L, 0xff, 0x03};
    RxTxPacket(MOTOR_LEFT, 3, INSTR_WRITE, parametros_);
}

//la idea es que la rueda de la izquierda vaya a mayor velocidad que la rueda de la derecha
//y en sentido contrario(porque están contrapuestas)

void giroIzquierda(void)
{

    byte parametros_right[3] = {REG_GOAL_SPEED_L, 0xff, 0x03};
    RxTxPacket(MOTOR_RIGHT, 3, INSTR_WRITE, parametros_right);
    byte parametros_left[3] = {REG_GOAL_SPEED_L, 0xff, 0x05};
    RxTxPacket(MOTOR_LEFT, 3, INSTR_WRITE, parametros_left);
}

//para pivotar hacia la derecha usamos los mismos valores en las 2 ruedas pero en sentido contrario, estando la derecha en sentido clockwise.
void pivotarIzquierda(void)
{
    byte parametros_right[3] = {REG_GOAL_SPEED_L, 0xff, 0x05};
    RxTxPacket(MOTOR_RIGHT, 3, INSTR_WRITE, parametros_right);
    byte parametros_left[3] = {REG_GOAL_SPEED_L, 0xff, 0x05};
    RxTxPacket(MOTOR_LEFT, 3, INSTR_WRITE, parametros_left);
}

//para pivotar hacia la derecha usamos los mismos valores en las 2 ruedas pero en sentido contrario, estando la derecha en sentido anticlockwise.
void pivotarDerecha(void)
{
    byte parametros_right[3] = {REG_GOAL_SPEED_L, 0xff, 0x01};
    RxTxPacket(MOTOR_RIGHT, 3, INSTR_WRITE, parametros_right);
    byte parametros_left[3] = {REG_GOAL_SPEED_L, 0xff, 0x01};
    RxTxPacket(MOTOR_LEFT, 3, INSTR_WRITE, parametros_left);
}

//sensores de distancia
//cuanto mayor sea el valor obtenido más cerca estará el robot de un obstáculo
void distanciaDelante(byte *readVal)
{
    struct RxReturn respuesta;
    byte parametros[2] = {REG_IR_CENTER, 0x01};
    respuesta = RxTxPacket(SENSOR, 2, INSTR_READ, parametros);
    *readVal = respuesta.StatusPacket[5];
}

void distanciaDerecha(byte *readVal)
{
    struct RxReturn respuesta;
    byte parametros[2] = {REG_IR_RIGHT, 0x01};
    respuesta = RxTxPacket(SENSOR, 2, INSTR_READ, parametros);
    *readVal = respuesta.StatusPacket[5];
}

void distanciaIzquierda(byte *readVal)
{
    struct RxReturn respuesta;
    byte parametros[2] = {REG_IR_LEFT, 0x01};
    respuesta = RxTxPacket(SENSOR, 2, INSTR_READ, parametros);
    *readVal = respuesta.StatusPacket[5];
}

void distanciaGeneral(byte *delante, byte *derecha, byte *izquierda) {
     struct RxReturn respuesta;
    byte parametros[2] = {REG_IR_LEFT, 0x03};
    respuesta = RxTxPacket(SENSOR, 2, INSTR_READ, parametros);
    *izquierda = respuesta.StatusPacket[5];
    *delante = respuesta.StatusPacket[6];
    *derecha = respuesta.StatusPacket[7];
}

//función que envía un mensaje al LCD sobreescribiendo la linea indicada
void escribir(char mensaje[], uint8_t linea)
{
    halLcdPrintLine(mensaje, linea, NORMAL_TEXT);
}

//función que escribe una línea en blanco
void borrar(uint8_t linea)
{
    halLcdPrintLine("               ", linea, NORMAL_TEXT);
}

//función que hace sonar una melodia haciendo solar una serie de notas durante una serie de tiempos
//se pueden hacer 52 notas musicales:
//      - 0 - la        - 13 - la#      - 26 - si       - 39 - do
//      - 1 - la#       - 14 - si       - 27 - do       - 40 - do#
//      - 2 - si        - 15 - do       - 28 - do#      - 41 - re
//      - 3 - do        - 16 - do#      - 29 - re       - 42 - re#
//      - 4 - do#       - 17 - re       - 30 - re#      - 43 - mi
//      - 5 - re        - 18 - re#      - 31 - mi       - 44 - fa
//      - 6 - re#       - 19 - mi       - 32 - fa       - 45 - fa#
//      - 7 - mi        - 20 - fa       - 33 - fa#      - 46 - sol
//      - 8 - fa        - 21 - fa#      - 34 - sol      - 47 - sol#
//      - 9 - fa#       - 22 - sol      - 35 - sol#     - 48 - la
//      - 10 - sol      - 23 - sol#     - 36 - la       - 49 - la#
//      - 11 - sol#     - 24 - la       - 37 - la#      - 50 - si
//      - 12 - la       - 25 - la#      - 38 - si       - 51 - do
//tiempo:
//      - 0-3: 0.3 segundos
//      - > 50: 5 segundos
//      - 240: la melodia suena sin parar
//      - 0: la melodia para
//      - 255 && notas pertenece a (0, 26) pueden sonar 27 melodias correspondientes a cada número
void melodia() {
    byte parametros[3];
    parametros[0] = REG_BUZZER_NOTES;

    uint8_t lenght;
    uint8_t i;
    contadorCancion = 0;

    lenght = 1;
    byte notas[] = {15};
    byte tiempos[] = {255};
    for (i=0; i < lenght; i++) {
        parametros[1] = notas[i];
        parametros[2] = tiempos[i];
        RxTxPacket(SENSOR, 3, INSTR_WRITE, parametros);

    }
}

//función que detecta el sonido que llega al micrófono de AX-S1
//el sonido se convierte en un valor numérico; si no hay sonido su valor será 127-128
//cuanto el valor más se acerque a 0 o a 255 más alto habrá sido el sonido detectado
void sonido(byte *readVal) {
    struct RxReturn respuesta;
    byte parametros[2] = {REG_SOUND, 0x01};
    respuesta = RxTxPacket(SENSOR, 2, INSTR_READ, parametros);
    *readVal = respuesta.StatusPacket[5];
}

//000 - no se ha detectado ningún objeto/luz
//1xx - se ha detectado un objeto/luz a la derecha
//x1x - se ha detectado un objeto/luz delante
//xx1 - se ha detectado un objeto/luz a la izquierda
void obstaculoDetectado(byte *readVal) {
    struct RxReturn respuesta;
    byte parametros[2] = {REG_GOAL_SPEED_L, 0x01};
    respuesta = RxTxPacket(SENSOR, 2, INSTR_READ, parametros);
    *readVal = respuesta.StatusPacket[5];
}

//funciones que detectan la luz
void luzIzquierda(byte *readVal) {
    struct RxReturn respuesta;
    byte parametros[2] = {REG_LIGHT_LEFT, 0x01};
    respuesta = RxTxPacket(SENSOR, 2, INSTR_READ, parametros);
    *readVal = respuesta.StatusPacket[5];
}

void luzDelante(byte *readVal) {
    struct RxReturn respuesta;
    byte parametros[2] = {REG_LIGHT_CENTER, 0x01};
    respuesta = RxTxPacket(SENSOR, 2, INSTR_READ, parametros);
    *readVal = respuesta.StatusPacket[5];
}

void luzDerecha(byte *readVal) {
    struct RxReturn respuesta;
    byte parametros[2] = {REG_LIGHT_RIGHT, 0x01};
    respuesta = RxTxPacket(SENSOR, 2, INSTR_READ, parametros);
    *readVal = respuesta.StatusPacket[5];
}

void leerVelocidad(uint16_t *der, uint16_t *izq) {
    struct RxReturn respuestaDerecha;
    struct RxReturn respuestaIzquierda;
    byte parametros[3] = {REG_GOAL_SPEED_L, 0x01, 0x01};
    respuestaDerecha = RxTxPacket(MOTOR_RIGHT, 3, INSTR_READ, parametros);
    respuestaIzquierda = RxTxPacket(MOTOR_LEFT, 3, INSTR_READ, parametros);

    uint16_t respDerechaL = (uint16_t) (respuestaDerecha.StatusPacket[5]);
    uint16_t respDerechaH = (uint16_t) (respuestaDerecha.StatusPacket[6]);
    uint16_t respIzqL = (uint16_t) (respuestaIzquierda.StatusPacket[5]);
    uint16_t respIzqH = (uint16_t) (respuestaIzquierda.StatusPacket[6]);
    der = respDerechaL + 256*respDerechaH;
    izq = respIzqL + 256*respIzqH;
}

void temperatura(byte *readVal) {
    struct RxReturn respuesta;
    byte parametros[2] = {REG_TEMP, 0x01};
    respuesta = RxTxPacket(SENSOR, 2, INSTR_READ, parametros);
    *readVal = respuesta.StatusPacket[5];
}

void TA1_0_IRQHandler(void)
{
    //TODO
    TA1CCTL0 &= ~CCIE;                          //desactivamos las interrupciones del timer A1
    TA1CCTL0 &= ~TIMER_A_CCTLN_CCIFG;           //limpiamos el flag de interrupción

    contadorCancion ++;

    TA1CCTL0 |= CCIE;                           //activo las interrupciones del timer A1
}

void init_GPIOs(void)
{
    //Pulsador S1 del MK II:
    P5SEL0 &= ~0x02;   //Pin P5.1 como I/O digital,
    P5SEL1 &= ~0x02;   //Pin P5.1 como I/O digital,
    P5DIR &= ~0x02; //Pin P5.1 como entrada
    P5IES &= ~0x02;   // con transicion L->H
    P5IE |= 0x02;     //Interrupciones activadas en P5.1,
    P5IFG = 0;    //Limpiamos todos los flags de las interrupciones del puerto 5
    //P5REN: Ya hay una resistencia de pullup en la placa MK II

    //Pulsador S2 del MK II:
    P3SEL0 &= ~0x20;   //Pin P3.5 como I/O digital,
    P3SEL1 &= ~0x20;   //Pin P3.5 como I/O digital,
    P3DIR &= ~0x20; //Pin P3.5 como entrada
    P3IES &= ~0x20;   // con transicion L->H
    P3IE |= 0x20;   //Interrupciones activadas en P3.5
    P3IFG = 0;  //Limpiamos todos los flags de las interrupciones del puerto 3
    //P3REN: Ya hay una resistencia de pullup en la placa MK II

    //Configuramos los GPIOs del joystick del MK II:
    P4DIR &= ~(BIT1 + BIT5 + BIT7);   //Pines P4.1, 4.5 y 4.7 como entrades,
    P4SEL0 &= ~(BIT1 + BIT5 + BIT7);  //Pines P4.1, 4.5 y 4.7 como I/O digitales,
    P4SEL1 &= ~(BIT1 + BIT5 + BIT7);
    P4REN |= BIT1 + BIT5 + BIT7;  //con resistencia activada
    P4OUT |= BIT1 + BIT5 + BIT7;  // de pull-up
    P4IE |= BIT1 + BIT5 + BIT7;   //Interrupciones activadas en P4.1, 4.5 y 4.7,
    P4IES &= ~(BIT1 + BIT5 + BIT7);   //las interrupciones se generaran con transicion L->H
    P4IFG = 0;    //Limpiamos todos los flags de las interrupciones del puerto 4

    P5DIR &= ~(BIT4 + BIT5);  //Pines P5.4 y 5.5 como entrades,
    P5SEL0 &= ~(BIT4 + BIT5); //Pines P5.4 y 5.5 como I/O digitales,
    P5SEL1 &= ~(BIT4 + BIT5);
    P5IE |= BIT4 + BIT5;  //Interrupciones activadas en 5.4 y 5.5,
    P5IES &= ~(BIT4 + BIT5);  //las interrupciones se generaran con transicion L->H
    P5IFG = 0;    //Limpiamos todos los flags de las interrupciones del puerto 4
    // - Ya hay una resistencia de pullup en la placa MK II
}


//para girar x grados hay que poner la velocidad a 150 en uno de los motores y
//el otro a 0 y esperar (28.5ms) * (x grados)
/*void girarGradosDerecha (byte grados) {
    pivotarDerechaV(150);
    usleep(grados*28.5*1000);
}


void girarGradosIzquierda (byte grados) {
    pivotarDerechaV(150);
    usleep(grados*28.5*1000);
}*/

void pivotarDerechaV (byte velocidad) {
    parar();
    byte parametros_right[3] = {REG_GOAL_SPEED_L, velocidad, 0x05};
    RxTxPacket(MOTOR_RIGHT, 3, INSTR_WRITE, parametros_right);
    byte parametros_left[3] = {REG_GOAL_SPEED_L, velocidad, 0x05};
    RxTxPacket(MOTOR_LEFT, 3, INSTR_WRITE, parametros_left);
}

void pivotarIzquierdaV (byte velocidad) {
    parar();
    byte parametros_right[3] = {REG_GOAL_SPEED_L, velocidad, 0x01};
    RxTxPacket(MOTOR_RIGHT, 3, INSTR_WRITE, parametros_right);
    byte parametros_left[3] = {REG_GOAL_SPEED_L, velocidad, 0x01};
    RxTxPacket(MOTOR_LEFT, 3, INSTR_WRITE, parametros_left);
}

void girarDerechaV (byte velocidad) {
    byte parametros[3] = {REG_GOAL_SPEED_L, velocidad/2, 0x05};
    RxTxPacket(MOTOR_RIGHT, 3, INSTR_WRITE, parametros);
    byte parametros_[3] = {REG_GOAL_SPEED_L, velocidad, 0x03};
    RxTxPacket(MOTOR_LEFT, 3, INSTR_WRITE, parametros_);
}

void girarIzquierdaV (byte velocidad) {
    byte parametros_right[3] = {REG_GOAL_SPEED_L, velocidad, 0x03};
    RxTxPacket(MOTOR_RIGHT, 3, INSTR_WRITE, parametros_right);
    byte parametros_left[3] = {REG_GOAL_SPEED_L, velocidad/2, 0x05};
    RxTxPacket(MOTOR_LEFT, 3, INSTR_WRITE, parametros_left);
}

void luzDetectada(byte *readVal) {
    struct RxReturn respuesta;
    byte parametros[2] = {REG_GOAL_SPEED_H, 0x01};
    respuesta = RxTxPacket(SENSOR, 2, INSTR_READ, parametros);
    *readVal = respuesta.StatusPacket[5];
}

void silencio(void) {
    byte parametros[3];
    parametros[0] = REG_BUZZER_NOTES;

    uint8_t lenght;
    uint8_t i;
    contadorCancion = 0;

    lenght = 1;
    byte notas[] = {15};
    byte tiempos[] = {0};
    for (i=0; i < lenght; i++) {
        parametros[1] = notas[i];
        parametros[2] = tiempos[i];
        RxTxPacket(SENSOR, 3, INSTR_WRITE, parametros);

    }
}

void sonidoConstante(void) {
    byte parametros[3];
    parametros[0] = REG_BUZZER_NOTES;

    uint8_t lenght;
    uint8_t i;
    contadorCancion = 0;

    lenght = 1;
    byte notas[] = {15};
    byte tiempos[] = {254};
    for (i=0; i < lenght; i++) {
        parametros[1] = notas[i];
        parametros[2] = tiempos[i];
        RxTxPacket(SENSOR, 3, INSTR_WRITE, parametros);

    }
}


