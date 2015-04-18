#include <18F45K80.h>
#device ADC=12
#device *=16
#device HIGH_INTS=TRUE /* allow high priority "FAST" interrutps */
#fuses NOXINST

#if 0
/* no boot loader */
#fuses HSH
#fuses NOPLLEN
#fuses NOFCMEN
#fuses NOIESO
#fuses PUT
#fuses BORV30
#fuses WDT32768
#fuses NOMCLR
#fuses STVREN
#fuses SOSC_DIG
#fuses NOXINST
#fuses NODEBUG

#else

/* DS30 boot loader version 1.5.1 - engine 2.2.2 */
/* leave last nine pages alone for boot loader. first two words do the jump to the boot loader */
/* max mem address - 0x243, max mem address - 0x240 */
#build(reset=0x7dbc:0x7dbf)
/* max mem address - 0x23f, max mem address - see memory organization in datasheet */
//#org 0x7dc0,0x7fff {}
#org 0x7dc0,0x7fff {}

#endif


#include <stdlib.h>
#use delay(clock=12000000, restart_wdt)

/* 
Parameters are stored in EEPROM
*/
#define PARAM_CRC_ADDRESS  0x000
#define PARAM_ADDRESS      PARAM_CRC_ADDRESS+2


/* UART1 - async serial connection connection to Pi */
#use rs232(UART1,stream=MODBUS_SERIAL,baud=115200,xmit=PIN_C6,rcv=PIN_C7,errors)	


#byte TXSTA=GETENV("SFR:txsta1")
#bit  TRMT=TXSTA.1
#byte ANCON0=GETENV("SFR:ancon0")
#byte ANCON1=GETENV("SFR:ancon1")

#byte PORTB=GETENV("SFR:portb")
#byte INTCON2=GETENV("SFR:intcon2")
#bit RBPU=INTCON2.7

/* UART2 - FTDI cable */
#use rs232(UART2,stream=DEBUG, baud=115200,xmit=PIN_D6,rcv=PIN_D7,errors)	


#use standard_io(A)
#use standard_io(B)
#use standard_io(C)
#use standard_io(D)
#use standard_io(E)

#define LED_GREEN                PIN_C2
#define BUTTON                   PIN_B3
#define BUTTON_BIT               3
#define PI_POWER_EN              PIN_C0
#define PIC_BOOTLOAD_REQUEST     PIN_D2 
#define PIC_BOOTLOAD_REQUEST_BIT 2
#define WATCHDOG_FROM_PI         PIN_C5


/* analog channels */
#define AN_USER_USER_0 1
#define AN_USER_USER_1 2
#define AN_USER_USER_2 4
#define AN_USER_USER_3 5

#define AN_TEMPERATURE 0
#define AN_IN_VOLTS    9
#define AN_WIND_DIR_0  6
#define AN_WIND_DIR_1  7

typedef union {
	int16 l[2];
    int8 b[4];
    int32 word;
} u_lblock;

#byte port_b=GETENV("SFR:portb")
#byte port_c=GETENV("SFR:portc")