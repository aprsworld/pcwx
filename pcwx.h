#include <18F46K22.h>
#device ADC=12
#device *=16
#device HIGH_INTS=TRUE /* allow high priority "FAST" interrutps */

#include <stdlib.h>
#FUSES HSM,NOPROTECT,PUT,NOLVP,BROWNOUT,NOMCLR,WDT32768 
#FUSES NOPLLEN, NOFCMEN, NOIESO, NOXINST, NODEBUG, NOPROTECT
#use delay(clock=12000000, restart_wdt)

/* 
Parameters are stored in EEPROM
*/
#define PARAM_CRC_ADDRESS  0x000
#define PARAM_ADDRESS      PARAM_CRC_ADDRESS+2

#define EE_FOR_HOST_ADDRESS 512


#define WEATHER_X_BAUD 57600


/* UART1 - async serial connection connection to Pi */
#use rs232(UART1,stream=STREAM_PI,baud=WEATHER_X_BAUD,xmit=PIN_C6,rcv=PIN_C7,errors)	


#byte TXSTA=GETENV("SFR:txsta1")
#bit  TRMT=TXSTA.1

#byte TXSTA2=GETENV("SFR:txsta2")
#bit  TRMT2=TXSTA2.1

#byte PORTB=GETENV("SFR:portb")
#byte INTCON2=GETENV("SFR:intcon2")
#bit RBPU=INTCON2.7

/* UART2 - RS-485 network */
#use rs232(UART2, stream=STREAM_RS485, baud=9600,errors)	




#use fast_io(A)
#use fast_io(B)
#use fast_io(C)
#use fast_io(D)
#use fast_io(E)

#define LED_GREEN                PIN_A3
#define BUTTON                   PIN_B3
#define BUTTON_BIT               3
#define PI_POWER_EN              PIN_C0
#define PIC_BOOTLOAD_REQUEST     PIN_B4
#define PIC_BOOTLOAD_REQUEST_BIT 4
#define WATCHDOG_FROM_PI         PIN_B5
#define WATCHDOG_FROM_PI_BIT     5
#define SYNC_OUT                 PIN_D4
#define SER_TO_NET               PIN_D6
#define SER_FROM_NET             PIN_D7
#define PI_POWER_FLAG            PIN_C1

#define ADC_CLK                  PIN_C3
#define ADC_DOUT                 PIN_C4
#define ADC_DIN                  PIN_C5
#define ADC_NCS                  PIN_D2

#define RS485_NRE                PIN_D0
#define RS485_DE                 PIN_D1
#define PIC_TO_PI                PIN_D3
#define SER_TO_PI                PIN_C6
#define SER_FROM_PI              PIN_C7

typedef union {
	int16 l[2];
    int8 b[4];
    int32 word;
} u_lblock;

#byte port_b=GETENV("SFR:portb")
#byte port_c=GETENV("SFR:portc")

