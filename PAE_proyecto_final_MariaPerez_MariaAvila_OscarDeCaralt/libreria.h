/*
 * libreria.h
 *
 *  Created on: 3 mayo. 2022
*   Author: maria
 */
#include <stdbool.h>

#ifndef LIBRERIA_H_
#define LIBRERIA_H_


#include <stdint.h>

#define REAL 1
//#define EMULADA 1

typedef uint8_t byte;
#define Si 1
#define No 0

#define TXD0_READY (UCA0IFG & UCTXIFG)
#define TXD2_READY (UCA2IFG & UCTXIFG)

#ifdef EMULADA
typedef enum _motor_type {
    MOTOR_LEFT = 1,
    MOTOR_RIGHT = 2,
} MOTOR_t;
#define SENSOR 3
#endif

#ifdef REAL
typedef enum _motor_type {
    MOTOR_LEFT = 4,
    MOTOR_RIGHT = 2,
} MOTOR_t;
#define SENSOR 100
#endif


typedef enum _instr_type {
    INSTR_READ = 2,
    INSTR_WRITE = 3,
} INSTR_t;

typedef enum _reg_type {
    REG_CW_L = 0x06,
    REG_CW_H = 0x07,
    REG_CCW_L = 0x08,
    REG_CCW_H = 0x09,
    REG_LED = 0x19,
    REG_IR_LEFT = 0x1A,
    REG_IR_CENTER = 0x1B,
    REG_IR_RIGHT = 0x1C,
    REG_GOAL_SPEED_L = 0x20, //coincide con el registro del sensor que indica si se ha detectado un obstáculo
    REG_GOAL_SPEED_H = 0x21, //coincide con el registro del sensor que indica si se ha detectado luz
    REG_BUZZER_NOTES = 0x28,
    REG_BUZZER_TIME = 0x29,
    REG_SOUND = 0x23,
    REG_LIGHT_LEFT = 0x1D,
    REG_LIGHT_CENTER = 0x1E,
    REG_LIGHT_RIGHT = 0x1F,
    REG_TEMP = 0x2B,
} REG_t;


void sentidoDatos_RX(void);
void sentidoDatos_TX(void);

struct RxReturn{
    byte StatusPacket[16];
    byte checkSum;
    byte timeout;
};

uint16_t DatoLeido_UART;
uint16_t tiempo;
byte Byte_Recibido;
byte contadorCancion;

byte TxPacket (byte bID, byte bParameterLength, byte bInstruction, byte Parametros[16]);
void TxUAC0(byte bTxdata);
void TxUAC2(byte bTxdata);
struct RxReturn RxPacket(void);
struct RxReturn RxTxPacket(byte bID, byte bParameterLength, byte bInstruction, byte Parametros[16]);

void Activa_TimerA0_TimeOut(void);
void Reset_Timeout(void);
void Stop_Timeout(void);
byte TimeOut(uint16_t t);
void EUSCIA0_IRQHandler(void);
void EUSCIA2_IRQHandler(void);
void init_uart(void);
void init_interrupciones(void);
void init_timers(void);
void init_GPIOs(void);

//LEDs
void encender_LED_derecho(void);
void encender_LED_izquierdo(void);
void encender_LEDs(void);
void apagar_LED_derecho(void);
void apagar_LED_izquierdo(void);
void apagar_LEDs(void);
void leer_LED_derecho(byte *readVal);
void leer_LED_izquierdo(byte *readVal);

//movimiento
void setEndlessTurn(void);
void delante(void);
void atras(void);
void delanteV(byte velocidad);
void atrasV(byte velociad);
void parar(void);

//giro
void giroDerecha(void);
void giroIzquierda(void);
void pivotarDerecha(void);
void pivotarIzquierda(void);

//sensores de distancia
void distanciaDelante(byte *readVal);
void distanciaDerecha(byte *readVal);
void distanciaIzquierda(byte *readVal);
void distanciaGeneral(byte *delante, byte *derecha, byte *izquerda);

void escribir(char mensaje[], uint8_t linea);
void borrar(uint8_t linea);
void melodia();
void sonido(byte *readVal);
void obstaculoDetectado(byte *readVal);
void luzIzquierda(byte *readVal);
void luzDelante(byte *readVal);
void luzDerecha(byte *readVal);
void leerVelocidad(uint16_t *derecho, uint16_t *izquierdo);
void temperatura(byte *readVal);
void luzDetectada(byte *readVal);
void silencio();
void sonidoConstante(void);

//void girarGradosDerecha (byte grados);
//void girarGradosIzquierda (byte grados);
void pivotarDerechaV (byte velocidad);
void pivotarIzquierdaV (byte velocidad);
void girarDerechaV (byte velocidad);
void girarIzquierdaV (byte velocidad);

#endif /* LIBRERIA_H_ */

