#include <18F46K22.h>
#device ADC=12
#device *=16
#device HIGH_INTS=TRUE /* allow high priority "FAST" interrutps */

#include <stdlib.h>
#FUSES HS,NOPROTECT,PUT,NOLVP,BROWNOUT,NOMCLR,WDT32768
#use delay(clock=12000000, restart_wdt)

/* 
Parameters are stored in EEPROM
*/
#define PARAM_CRC_ADDRESS  0x000
#define PARAM_ADDRESS      PARAM_CRC_ADDRESS+2

#define WEATHER_X_BAUD 57600


/* UART1 - async serial connection connection to Pi */
#use rs232(UART1,stream=MODBUS_SERIAL,baud=WEATHER_X_BAUD,xmit=PIN_C6,rcv=PIN_C7,errors)	


#byte TXSTA=GETENV("SFR:txsta")
#bit  TRMT=TXSTA.1

#byte PORTB=GETENV("SFR:portb")
#byte INTCON2=GETENV("SFR:intcon2")
#bit RBPU=INTCON2.7

/* FTDI cable on software serial port */
#use rs232(stream=DEBUG, baud=9600,xmit=PIN_D6,rcv=PIN_D7,errors)	




#use fast_io(A)
#use fast_io(B)
#use fast_io(C)
#use fast_io(D)
#use fast_io(E)

#define LED_GREEN                PIN_C2
#define BUTTON                   PIN_B3
#define BUTTON_BIT               3
#define PI_POWER_EN              PIN_C0
#define PIC_BOOTLOAD_REQUEST     PIN_D2 
#define PIC_BOOTLOAD_REQUEST_BIT 2
#define WATCHDOG_FROM_PI         PIN_C5
#define WATCHDOG_FROM_PI_BIT     5


/* analog channels */
#define AN_USER_USER_0 1
#define AN_USER_USER_1 2
#define AN_USER_USER_2 4
#define AN_USER_USER_3 5

#define AN_TEMPERATURE 0 /* not connected */
#define AN_IN_VOLTS    0
#define AN_WIND_DIR_0  6
#define AN_WIND_DIR_1  7

typedef union {
	int16 l[2];
    int8 b[4];
    int32 word;
} u_lblock;

#byte port_b=GETENV("SFR:portb")
#byte port_c=GETENV("SFR:portc")


#define TP_BLACK PIN_D5
#define TP_BROWN PIN_D7
#define TP_RED   PIN_D6