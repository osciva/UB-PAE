#include <libreria.h>
#include "msp.h"
#include "lib_PAE.h"
#include <stdio.h>
#include <stdlib.h>

#define pulsadorS1 1
#define pulsadorS2 2
#define JArriba 3
#define JAbajo 4
#define JCentro 5
#define JDerecho 6
#define JIzquierdo 7

uint8_t linea = 0;
char cadena[16];
uint8_t estado = 0;
uint8_t estado_anterior = 0;
uint8_t contadorLuz = 0;
uint8_t opcion = 0;

typedef enum pared {
    NINGUNA = 0,
    IZQUIERDA = 1,
    DERECHA = 2,
} PARED;

void menu (uint8_t opcion);
void recorrido (void);
void recorridoLuz (void);

void main(void){
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; //stop watch dog timer
    init_ucs_24MHz();

    init_uart();                //inicializamos la UART y habilitamos sus interrupciones a nivel de dispositivo
    init_timers();              //inicializamos los timers y habilitamos sus interrupciones a nivel de dispositivo
    init_interrupciones();      //habilitamos las interrupciones a nivel de controlador
    __enable_interrupts();      //habilitamos las interrupciones a nivek global
    init_GPIOs();
    halLcdInit();
    halLcdClearScreenBkg();


    setEndlessTurn();

    char menuOpciones[16] = "  -- MENU --  ";
    char mensajeArriba[16] = "Distancia - U";
    char mensajeAbajo[16] = "Luminosidad - D";
    char mensajeDerecha[16] = "Velocidad - R";
    char mensajeIzquierda[16] = "Temperatura - L";

    escribir(menuOpciones, 1);
    escribir(mensajeArriba, 2);
    escribir(mensajeAbajo, 3);
    escribir(mensajeDerecha, 4);
    escribir(mensajeIzquierda, 5);

    while (1) {
        if (estado != estado_anterior) {
            parar();
            estado_anterior = estado;
            switch(estado)
            {
            case pulsadorS1:
                parar();
                recorrido();
                break;
            case pulsadorS2:
                parar();
                recorridoLuz();
                break;
            case JArriba:
                opcion = 1;
                menu(opcion);
                break;
            case JAbajo:
                opcion = 2;
                menu(opcion);
                break;
            case JDerecho:
                opcion = 3;
                menu(opcion);
                break;
            case JIzquierdo:
                opcion = 4;
                menu(opcion);
                break;
            }
        }
    }
}


//interrupcion del pulsador S2
void PORT3_IRQHandler(void)
{
    uint8_t flag = P3IV;
    P3IE &= 0xDF;
    estado_anterior=0;
    switch(flag){
    case 0x0C:  //BIT 5
        estado = pulsadorS2;
        break;
    }
    P3IE |= 0x20;
}

//interrupciones de los joysticks derecho, izquierdo y centro
void PORT4_IRQHandler(void)
{
    uint8_t flag = P4IV;
    P4IE &= 0x5D;
    estado_anterior = 0;
    switch(flag){
    case 0x04:  //BIT 1 - no lo vamos a utilizar
        estado = JCentro;
        break;
    case 0x0C:  //BIT 5
        estado = JDerecho;
        break;
    case 0x10:  //BIT 7
        estado = JIzquierdo;
        break;
    }
    P4IE |= 0xA2;
}

//interrupcioens del pulsador S1 y de los joysticks arriba y abajo
void PORT5_IRQHandler(void){
    uint8_t flag = P5IV;
    P5IE &= 0xCD;
    estado_anterior = 0;
    switch(flag){
    case 0x04:  //BIT 1
        estado = pulsadorS1;
        break;
    case 0x0A:  //BIT 4
        estado = JArriba;
        break;
    case 0x0C:  //BIT 5
        estado = JAbajo;
        break;
    }
    P5IE |= 0x32;
}

void recorridoLuz (void)
{
    if (contadorLuz % 4 == 0) {
        if (opcion == 2) {
            byte a, b, c;
            char A[16];
            char B[16];
            char C[16];
            luzDelante(&a);
            luzDerecha(&b);
            luzIzquierda(&c);
            sprintf(A, "DELANTE: %03d", a);
            sprintf(B, "DERECHA: %03d", b);
            sprintf(C, "IZQUIERDA: %03d", c);
            escribir(A, 2);
            escribir(B, 3);
            escribir(C, 4);
        }
        byte d, der, izq;
        while (contadorLuz % 4 == 0) {
            luzDelante(&d);
            if (d != 0) {
                delanteV(100);
            }
            luzIzquierda(&izq);
            if (izq != 0) {
                pivotarIzquierda();
            }
            luzDerecha(&der);
            if (der != 0) {
                pivotarDerecha();
            }
        }
        parar();
    } else if (contadorLuz % 4 == 1){
        byte l;
        luzDetectada(&l);
        while (contadorLuz % 4 == 1) {
            if (l != 0) {
                sonidoConstante();
            } else {
                silencio();
            }
        }
    } else if (contadorLuz % 4 == 2){
        byte t;
        temperatura(&t);
        while (contadorLuz % 4 == 2) {
            if (t > 45) {
                temperatura(&t);
                atras();
            }

        }
    } else {
        byte son;
        byte ss = 0;
        while (1) {
            melodia();
            sonido(&son);
            if (son < 100 || son > 200) {
                if (ss % 2 == 0) {
                    pivotarDerecha();
                } else  {
                    pivotarIzquierda();
                }
                ss += 1;
            }
        }
    }
    contadorLuz += 1;

}

void recorrido(void) {
    //byte palmada;
    //sonido(&palmada);
    //while (palmada > 100 && palmada < 200) {
        //sonido(&palmada);
    //}
    PARED pared;
    byte obstaculo;
    byte del, derecha, izquierda;
    byte margen = 15;
    setEndlessTurn();
    while (1) {
        distanciaGeneral(&del, &derecha, &izquierda);
        obstaculoDetectado(&obstaculo);

        if (derecha > izquierda) {
          pared = DERECHA;
        } else if (izquierda > derecha) {
          pared = IZQUIERDA;
        }
       switch (pared){
       case DERECHA:
           if (del > margen || derecha > margen) {
             pivotarIzquierda();
          }
          else if (del < margen && derecha < margen) {
             pivotarDerecha();
          } else {
              delanteV(100);
          }

          if (del > margen && izquierda > margen && derecha > margen) {
              distanciaGeneral(&del, &derecha, &izquierda);
              while (del > 0) {
                  distanciaGeneral(&del, &derecha, &izquierda);
                  pivotarDerecha();
              }

          }
          break;
      case IZQUIERDA:
          if (del > margen || izquierda > margen) {
             pivotarDerecha();
         }
         else if (del < margen && izquierda < margen) {
             pivotarIzquierda();
         } else {
             delanteV(100);
         }
         break;
    }
}


void menu (uint8_t opcion) {
    halLcdClearScreenBkg();
    switch (opcion) {
    case 1:
        escribir("- DISTANCIAS -", 1);
        char mensajeDelante[16];
        char mensajeDerecha[16];
        char mensajeIzquierda[16];
        byte del, derecha, izquierda;
        distanciaGeneral(&del, &derecha, &izquierda);
        sprintf(mensajeDelante, "DELANTE: %03d", del);
        sprintf(mensajeDerecha, "DERECHA: %03d", derecha);
        sprintf(mensajeIzquierda, "IZQUIERDA: %03d", izquierda);
        escribir(mensajeDelante, 2);
        escribir(mensajeDerecha, 3);
        escribir(mensajeIzquierda, 4);
        break;
    case 2:
        escribir(" -LUMINOSIDAD-", 1);
        byte a, b, c;
        char A[16];
        char B[16];
        char C[16];
        luzDelante(&a);
        luzDerecha(&b);
        luzIzquierda(&c);
        sprintf(A, "DELANTE: %03d", a);
        sprintf(B, "DERECHA: %03d", b);
        sprintf(C, "IZQUIERDA: %03d", c);
        escribir(A, 2);
        escribir(B, 3);
        escribir(C, 4);
        break;
    case 3:
        escribir(" - VELOCIDAD -", 1);
        uint16_t der;
        uint16_t izq;
        char mensajeDerecho[16];
        char mensajeIzquierdo[16];
        leerVelocidad(&der, &izq);
        sprintf(mensajeDerecho, "M. R.: %03d", der);
        sprintf(mensajeIzquierdo, "M. L.: %03d", izq);
        escribir(mensajeDerecho, 2);
        escribir(mensajeIzquierdo, 3);
        break;
    case 4:
        escribir("- TEMPERATURA -", 1);
        char mensajeTemperatura[16];
        byte temp;
        temperatura(&temp);
        sprintf(mensajeTemperatura, "    %03d C", temp);
        escribir(mensajeTemperatura, 2);
        break;
    default:
        break;
    }
}


