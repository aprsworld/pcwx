#include "piCameraWeatherX.h"

typedef struct {
	int8 modbus_address;
	int8 modbus_mode;

	int8 serial_prefix;
	int16 serial_number;

	int8 adc_average_mode[8];
	int16 adc_sample_ticks;
} struct_config;



typedef struct {
	/* most recent valid */
	int16 pulse_period[3];
	int16 pulse_min_period[3];
	int16 pulse_max_period[3];
	int16 pulse_count[3];
	int32 pulse_sum[3];

	int16 adc_std_dev[8];

	/* circular buffer for ADC readings */
	int16 adc_buffer[8][16];
	int8  adc_buffer_index;

	int16 modbus_our_packets;
	int16 modbus_other_packets;
	int16 modbus_last_error;

	int16 sequence_number;
	int16 uptime_minutes;
	int16 interval_milliseconds;

	int8 factory_unlocked;
} struct_current;

typedef struct {
	int16 pulse_period[4];
	int16 pulse_count[4];
	int8 pulse_ch_en;

	int8 led_on_green;

	int1 now_adc_sample;
	int1 now_adc_reset_count;

	int1 now_millisecond;
	int1 now_bridged;

	int8 port_b;
} struct_time_keep;


/* global structures */
struct_config config;
struct_current current;
struct_time_keep timers;


#include "adc_piCameraWeatherX.c"
#include "interrupt_piCameraWeatherX.c"
#include "param_piCameraWeatherX.c"

#include "modbus_slave_piCameraWeatherX.c"
#include "modbus_handler_piCameraWeatherX.c"

void init() {
	int8 i;

	setup_oscillator(OSC_16MHZ|OSC_INTRC);
	setup_adc_ports(sAN0 | sAN1 | sAN2 | sAN4 | sAN5 | sAN6 | sAN7 | sAN9, VSS_VREF);
	setup_adc(ADC_CLOCK_INTERNAL);


	/* data structure initialization */
	timers.led_on_green=0;
	timers.now_adc_sample=0;
	timers.now_adc_reset_count=0;
	timers.now_millisecond=0;
	timers.now_bridged=1;
	timers.port_b=0b11111111;

	for ( i=0 ; i<3 ; i++ ) {
		current.pulse_period[i]=0;
		current.pulse_min_period[i]=65535;
		current.pulse_max_period[i]=0;
		current.pulse_count[i]=0;
		current.pulse_sum[i]=0;
	}

	current.modbus_our_packets=0;
	current.modbus_other_packets=0;
	current.modbus_last_error=0;
	current.sequence_number=0;
	current.uptime_minutes=0;
	current.interval_milliseconds=0;
	current.adc_buffer_index=0;
	current.factory_unlocked=0;




	/* interrupts */

	/* external interrupts for anemometers */
	ext_int_edge(0,H_TO_L);
	enable_interrupts(INT_EXT);
	ext_int_edge(1,H_TO_L);
	enable_interrupts(INT_EXT1);
	ext_int_edge(2,H_TO_L);
	enable_interrupts(INT_EXT2);

	/* one periodic interrupt @ 100uS. Generated from internal 16 MHz clock */
	setup_timer_2(T2_DIV_BY_16,24,1); /* prescale=16, match=24, postscale=1. Match is 24 because when match occurs, one cycle is lost */
	enable_interrupts(INT_TIMER2);



}


void periodic_millisecond(void) {
	/* button debouncing */
	static int16 b0_state=0;

	timers.now_millisecond=0;

	/* button must be down for 12 milliseconds */
	b0_state=(b0_state<<1) | !bit_test(timers.port_b,BUTTON_0_BIT) | 0xe000;
	if ( b0_state==0xf000) {
		/* button pressed */
		timers.now_bridged = !timers.now_bridged;
	}

	/* anemometers quit moving */
	if ( 0xffff == timers.pulse_period[0] )
				current.pulse_period[0]=0;
	if ( 0xffff == timers.pulse_period[1] )
				current.pulse_period[1]=0;
	if ( 0xffff == timers.pulse_period[2] )
				current.pulse_period[2]=0;


	/* read port_b pin states */
	timers.port_b=port_b;
}


void main(void) {
//	int8 c;
//	int1 bridged=false;
	int8 i;

	init();

	fprintf(DEBUG,"# piCameraWeatherX %s\r\n",__DATE__);
	fprintf(DEBUG,"# restart_cause()=%u\r\n",restart_cause());

	read_param_file();


	if ( config.modbus_address > 127 ) {
		write_default_param_file();
	}

	/* start Modbus slave */
	setup_uart(TRUE);
	/* modbus_init turns on global interrupts */
	modbus_init();


	/* Prime ADC filter */
	for ( i=0 ; i<30 ; i++ ) {
		adc_update();
	}



	for ( ; ; ) {
		restart_wdt();

		if ( timers.now_millisecond ) {
			periodic_millisecond();
		}


		if ( timers.now_adc_sample ) {
			timers.now_adc_sample=0;
			adc_update();
		}


#if 0
		/* button switches between bridged and non bridged */		
		if ( ! input(PUSH_BUTTON) ) {
			bridged=!bridged;
			delay_ms(250);

			if ( ! bridged ) {
				fprintf(DEBUG,"# g=LED off G=LED on p=PI off P=PI on r=reset\r\n");
				output_low(LED_GREEN);
			} else {
				fprintf(DEBUG,"# entering bridge mode\r\n");
				output_high(LED_GREEN);
			}

		}

		if ( bridged ) {
			if ( kbhit(DEBUG) ) {
				fputc(fgetc(DEBUG),MODBUS_SERIAL);
			}
			if ( kbhit(MODBUS_SERIAL) ) {
				fputc(fgetc(MODBUS_SERIAL),DEBUG);
			}

		} else {
			if ( kbhit(DEBUG) ) {
				c = fgetc(DEBUG);

				switch ( c ) {
					case 'g': output_low(LED_GREEN); break;
					case 'G': output_high(LED_GREEN); break;
					case 'p': output_low(PI_POWER_EN); break;
					case 'P': output_high(PI_POWER_EN); break;
					case 'r': delay_ms(1000); reset_cpu(); break;
					case '?': fprintf(DEBUG,"# g=LED off G=LED on p=PI off P=PI on r=reset\r\n"); break;
					default: fprintf(DEBUG,"# invalid command. Valid commands are {g,G,p,P,r,?} (%s)\r\n",__DATE__);
				}

			}
		}
#endif
	}
}