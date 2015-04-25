#include "piCameraWeatherX.h"

typedef struct {
	int8 modbus_address;
	int8 modbus_mode;

	int8 serial_prefix;
	int16 serial_number;

	int16 adc_sample_ticks;

	int8 allow_bootload_request;
	int16 watchdog_seconds_max;
	int8 pi_offtime_seconds;
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
	int1 bridged_uarts;

	int16 watchdog_seconds;
} struct_current;

typedef struct {
	int16 pulse_period[4];
	int16 pulse_count[4];
	int8 pulse_ch_en;

	int8 led_on_green;

	int1 now_adc_sample;
	int1 now_adc_reset_count;

	int1 now_millisecond;

	int8 port_b;
	int8 port_c;
} struct_time_keep;


/* global structures */
struct_config config;
struct_current current;
struct_time_keep timers;


#include "adc_piCameraWeatherX.c"
#include "param_piCameraWeatherX.c"

#include "modbus_slave_piCameraWeatherX.c"
#include "modbus_handler_piCameraWeatherX.c"

#include "interrupt_piCameraWeatherX.c"


void init() {
	int8 i;

//	setup_oscillator(OSC_16MHZ|OSC_INTRC);
	/* oscillator is external crystal, so no software setup required. Just the fuse */


	setup_adc_ports(sAN0 | sAN1 | sAN2 | sAN4 | sAN5 | sAN6 | sAN7 | sAN9, VSS_VREF);
	setup_adc(ADC_CLOCK_INTERNAL);


	/* data structure initialization */
	timers.led_on_green=0;
	timers.now_adc_sample=0;
	timers.now_adc_reset_count=0;
	timers.now_millisecond=0;
	timers.port_b=0b11111111;
	timers.port_c=0b11111111;

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
	current.bridged_uarts=0;
	current.watchdog_seconds=0;



	/* interrupts */

	/* external interrupts for anemometers */
	ext_int_edge(0,H_TO_L);
//	enable_interrupts(INT_EXT);
	ext_int_edge(1,H_TO_L);
//	enable_interrupts(INT_EXT1);
	ext_int_edge(2,H_TO_L);
//	enable_interrupts(INT_EXT2);

	/* one periodic interrupt @ 100uS. Generated from internal 16 MHz clock */
	/* prescale=16, match=24, postscale=1. Match is 24 because when match occurs, one cycle is lost */
	// setup_timer_2(T2_DIV_BY_16,24,1); 

	/* one periodic interrupt @ 100uS. Generated from system 12 MHz clock */
	/* prescale=4, match=74, postscale=1. Match is 74 because when match occurs, one cycle is lost */
	setup_timer_2(T2_DIV_BY_4,74,1);

	enable_interrupts(INT_TIMER2);
	enable_interrupts(INT_RDA2); /* debug cable */
	/* RDA - PI is turned on in modbus_slave_piCameraWeatherX's init */
}


void periodic_millisecond(void) {
	static int8 uptimeticks=0;
	static int16 adcTicks=0;
	static int16 ticks=0;
	/* button debouncing */
	static int16 b0_state=0; /* bridge push button */
	static int16 b1_state=0; /* reset line from PI */
	static int16 b2_state=0; /* watchdog line from PI */

	timers.now_millisecond=0;

//	fputc('.',DEBUG);

	/* button must be down for 12 milliseconds */
	b0_state=(b0_state<<1) | !bit_test(timers.port_b,BUTTON_BIT) | 0xe000;
	if ( b0_state==0xf000) {
		/* button pressed */
		current.bridged_uarts = !current.bridged_uarts;
		fprintf(DEBUG,"# bridged=%u\r\n",current.bridged_uarts);
	}

	/* reset must be down for 12 milliseconds */
	b1_state=(b1_state<<1) | !bit_test(timers.port_c,PIC_BOOTLOAD_REQUEST_BIT) | 0xe000;
	if ( b1_state==0xf000) {
		/* reset line asserted */
		if ( config.allow_bootload_request ) {
			reset_cpu();
		}
		/* BUG - I think that bootload request should be high for x milliseconds, rather than low */
	}

	/* watchdog must be down for 12 milliseconds for hit to register */
	b2_state=(b2_state<<1) | !bit_test(timers.port_c,WATCHDOG_FROM_PI_BIT) | 0xe000;
	if ( b2_state==0xf000) {
		/* watchdog hit */
		current.watchdog_seconds=0;
	}

	/* anemometers quit moving */
	if ( 0xffff == timers.pulse_period[0] )
				current.pulse_period[0]=0;
	if ( 0xffff == timers.pulse_period[1] )
				current.pulse_period[1]=0;
	if ( 0xffff == timers.pulse_period[2] )
				current.pulse_period[2]=0;


	/* read port_b and c pin states */
	timers.port_b=port_b;
	timers.port_c=port_c;


	/* green LED control */
	if ( current.bridged_uarts ) {
		/* always on when ports are bridged */
		output_high(LED_GREEN);
	} else {
		/* green LED in Modbus mode */
		if ( 0==timers.led_on_green ) {
			output_low(LED_GREEN);
		} else {
			output_high(LED_GREEN);
			timers.led_on_green--;
		}
	}


	/* some other random stuff that we don't need to do every cycle in main */
	if ( current.interval_milliseconds < 65535 ) {
		current.interval_milliseconds++;
	}

	/* seconds */
	ticks++;
	if ( 1000 == ticks ) {
		ticks=0;

		if ( current.watchdog_seconds != 65535 ) {
			current.watchdog_seconds++;
		}

		if ( 0 != config.watchdog_seconds_max && current.watchdog_seconds > config.watchdog_seconds_max ) {
			/* TODO power cycle the PI */
		}
		
		/* uptime counter */
		uptimeTicks++;
		if ( 60 == uptimeTicks ) {
			uptimeTicks=0;
			if ( current.uptime_minutes < 65535 ) 
				current.uptime_minutes++;
		}
	}


	/* ADC sample counter */
	if ( timers.now_adc_reset_count ) {
		timers.now_adc_reset_count=0;
		adcTicks=0;
	}

	adcTicks++;
	if ( adcTicks == config.adc_sample_ticks ) {
		adcTicks=0;
		timers.now_adc_sample=1;
	}
}


void main(void) {
	int8 i;

	i=restart_cause();

	init();

	output_high(LED_GREEN);
	output_high(PI_POWER_EN);
	delay_ms(1000);
	output_low(LED_GREEN);
	output_low(PI_POWER_EN);
	delay_ms(1000);
	output_high(LED_GREEN);
	output_high(PI_POWER_EN);




	fprintf(DEBUG,"# piCameraWeatherX %s\r\n",__DATE__);
	fprintf(DEBUG,"# restart_cause()=%u ",i);
	switch ( i ) {
		case WDT_TIMEOUT: fprintf(DEBUG,"WDT_TIMEOUT"); break;
		case MCLR_FROM_SLEEP: fprintf(DEBUG,"MCLR_FROM_SLEEP"); break;
		case MCLR_FROM_RUN: fprintf(DEBUG,"MCLR_FROM_RUN"); break;
		case NORMAL_POWER_UP: fprintf(DEBUG,"NORMAL_POWER_UP"); break;
		case BROWNOUT_RESTART: fprintf(DEBUG,"BROWNOUT_RESTART"); break;
		case WDT_FROM_SLEEP: fprintf(DEBUG,"WDT_FROM_SLEEP"); break;
		case RESET_INSTRUCTION: fprintf(DEBUG,"RESET_INSTRUCTION"); break;
		default: fprintf(DEBUG,"unknown!");
	}
	fprintf(DEBUG,"\r\n");

	fprintf(DEBUG,"# read_param_file() starting ...");
	read_param_file();
	fprintf(DEBUG," complete\r\n");


	if ( config.modbus_address > 127 ) {
		fprintf(DEBUG,"# write_default_param_file() starting ...");
		write_default_param_file();
		fprintf(DEBUG," complete\r\n");
	}

	/* start Modbus slave */
	setup_uart(TRUE);
	/* modbus_init turns on global interrupts */
	fprintf(DEBUG,"# modbus_init() starting ...");
	modbus_init();
	fprintf(DEBUG," complete\r\n");

	fprintf(DEBUG,"# bridged_uarts=%u\r\n",current.bridged_uarts);

//	enable_interrupts(INT_RDA);
//	enable_interrupts(GLOBAL);

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

		if ( ! current.bridged_uarts ) {
			modbus_process();
		}

	}
}