#include "pcwx.h"

#define RS485_MODE_OFF           0
#define RS485_MODE_MODBUS_BRIDGE 1
#define RS485_MODE_NMEA0183_RX   2

#define N_NMEA0183_SENTENCES 12

typedef struct {
	int8 modbus_address;
	int8 modbus_mode;

	int8 rs485_port_mode;

	int8 serial_prefix;
	int16 serial_number;

	int16 adc_sample_ticks;

	int8 allow_bootload_request;
	int16 watchdog_seconds_max;
	int16 pi_offtime_seconds;


	/* power control switch settings */
	int8 power_startup; /* 0==start with PI off, 1==start with PI on */
	int16 power_off_below_adc;
	int16 power_off_below_delay;
	int16 power_on_above_adc;
	int16 power_on_above_delay;
	int16 power_override_timeout;

	/* sentences we make available via modbus */
	int8 nmea0183_sentence[N_NMEA0183_SENTENCES][6];
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

	/* power control switch */
	int8 p_on;
	int16 power_on_delay;
	int16 power_off_delay;
	int16 power_override_timeout;
} struct_current;

typedef struct {
	int16 pulse_period[4];
	int16 pulse_count[4];
	int8 pulse_ch_en;

	int8 led_on_green;
	int16 load_off_seconds;

	int1 now_adc_sample;
	int1 now_adc_reset_count;

	int1 now_millisecond;

	int8 port_b;
	int8 port_c;

	short now_parse_rda2;
	int8 rda2_buff[256];
	int8 rda2_buff_pos;
	int8 rda2_buff_gap;
} struct_time_keep;



#define NMEA_SENTENCE_LENGTH 80
typedef struct {
	int8 sentence[N_NMEA0183_SENTENCES][80];
} struct_nmea;



/* global structures */
struct_config config;
struct_current current;
struct_time_keep timers;
struct_nmea nmea;

#include "mcp3208_pcwx.c"
#include "adc_pcwx.c"
#include "param_pcwx.c"

#include "modbus_slave_pcwx.c"
#include "modbus_handler_pcwx.c"

#include "interrupt_pcwx.c"


void init() {
	int8 i;

//	setup_adc_ports(AN0_TO_AN7,VSS_VREF);
//	setup_adc(ADC_CLOCK_DIV_16 | ADC_TAD_MUL_20 );

	setup_adc(ADC_OFF);

	port_b_pullups(0b00001000);

	set_tris_a(0b00000000);
	set_tris_b(0b11111111);
	set_tris_c(0b10010010);
	set_tris_d(0b10000000); 
	set_tris_e(0b00000000);


	/* data structure initialization */
	timers.led_on_green=0;
	timers.load_off_seconds=2;
	timers.now_adc_sample=0;
	timers.now_adc_reset_count=0;
	timers.now_millisecond=0;
	timers.port_b=0b11111111;
	timers.port_c=0b11111111;

	timers.rda2_buff_pos=0;
	timers.rda2_buff_gap=255;
	timers.now_parse_rda2=0;

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

	/* zero out NMEA structure */
	memset(&nmea,0,sizeof(nmea));

	/* power control switch */
	current.power_on_delay=config.power_on_above_delay;
	current.power_off_delay=config.power_off_below_delay;
	current.power_override_timeout=0;


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
	/* RDA - PI is turned on in modbus_slave_pcwx's init */
}


void periodic_millisecond(void) {
	static int8 uptimeticks=0;
	static int16 adcTicks=0;
	static int16 ticks=0;
	/* button debouncing */
//	static int16 b0_state=0; /* bridge push button */
//	static int16 b1_state=0; /* reset line from PI */
	static int16 b2_state=0; /* watchdog line from PI */
	/* power control */
	static int16 adcValue; /* updates after each ADC sample run */

	timers.now_millisecond=0;

//	fputc('.',STREAM_RS485);

#if 0
	/* button must be down for 12 milliseconds */
	b0_state=(b0_state<<1) | !bit_test(timers.port_b,BUTTON_BIT) | 0xe000;
	if ( b0_state==0xf000) {
		/* button pressed */
		current.bridged_uarts = !current.bridged_uarts;
		fprintf(STREAM_RS485,"# bridged=%u\r\n",current.bridged_uarts);
	}

	/* if we are in bridged uarts ... only check for button press */
	if ( current.bridged_uarts ) {
		return;
	}
#endif

#if 0
	/* reset must be down for 12 milliseconds */
	b1_state=(b1_state<<1) | !bit_test(timers.port_c,PIC_BOOTLOAD_REQUEST_BIT) | 0xe000;
	if ( b1_state==0xf000) {
		/* reset line asserted */
		if ( config.allow_bootload_request ) {
			reset_cpu();
		}
		/* BUG - I think that bootload request should be high for x milliseconds, rather than low */
	}
#endif

	/* watchdog must be down for 12 milliseconds for hit to register */
	b2_state=(b2_state<<1) | !bit_test(timers.port_c,WATCHDOG_FROM_PI_BIT) | 0xe000;
	if ( b2_state==0xf000) {
		/* watchdog hit */
//		current.watchdog_seconds=0;
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

		/* watchdog power control of pi */
		if ( current.watchdog_seconds != 65535 ) {
			current.watchdog_seconds++;
		}

		/* shut off when:
			a) watchdog_seconds_max != 0 AND watchdog_seconds is greater than watchdog_seconds_max AND it isn't already off 
		*/
		if ( 0 != config.watchdog_seconds_max && current.watchdog_seconds > config.watchdog_seconds_max && 0 == timers.load_off_seconds ) {
			timers.load_off_seconds=config.pi_offtime_seconds;
		}

		/* control power to the raspberrry pi load */
		if ( 0==timers.load_off_seconds ) {
			output_high(PI_POWER_EN);
		} else {
			output_low(PI_POWER_EN);
			timers.load_off_seconds--;

			if ( 0 == timers.load_off_seconds ) {
				/* reset watchdog seconds so we can turn back on */
				current.watchdog_seconds=0;
			}
		}

		
		/* uptime counter */
		uptimeTicks++;
		if ( 60 == uptimeTicks ) {
			uptimeTicks=0;
			if ( current.uptime_minutes < 65535 ) 
				current.uptime_minutes++;
		}
	}


	if ( 65535 == adcValue ) {
		/* signaled that a new ADC sample was taken and we need to run again */
		/* read current ADC value */	
		adcValue=adc_get(0);
	}

#if 0
	if ( adcValue > config.power_on_above_adc ) {
		if ( current.power_on_delay > 0 ) {
			current.power_on_delay--;
		} else {
			current.p_on=1;
		}
	} else {
		current.power_on_delay=config.power_on_above_delay;
	}
			

	if ( adcValue < config.power_off_below_adc ) {
		if ( current.power_off_delay > 0 ) {
			current.power_off_delay--;
		} else {
			current.p_on=0;
		}
	} else {
		current.power_off_delay=config.power_off_below_delay;
	}
#endif	

	/* ADC sample counter */
	if ( timers.now_adc_reset_count ) {
		timers.now_adc_reset_count=0;
		adcTicks=0;
	}

	/* ADC sampling trigger */
	adcTicks++;
	if ( adcTicks == config.adc_sample_ticks ) {
		adcTicks=0;
		timers.now_adc_sample=1;
		adcValue=65535; /* signal power control (above) on next pass to resample */
	}

	/* for RS-485 port */
	if ( timers.rda2_buff_gap < 255 ) {
		timers.rda2_buff_gap++;
	}

	/* RS-485: if we have data and we have >=3 miliseconds gap, we parse */
	if ( timers.rda2_buff_gap >= 10 && timers.rda2_buff_pos>0 ) {
		timers.now_parse_rda2=1;	
	}
}


void rs485_to_host(void) {
	int8 buff[sizeof(timers.rda2_buff)];
	int8 length;
	int16 l;

	/* get a local copy of our data */
	length=timers.rda2_buff_pos;
	timers.rda2_buff_pos=255; /* stop getting more data for a second */
	memcpy(buff,timers.rda2_buff,length);
	timers.rda2_buff_gap=0;
	timers.rda2_buff_pos=0;


	if ( RS485_MODE_MODBUS_BRIDGE==config.rs485_port_mode ) {
		/* transmit back out to the PI */
		for ( l=0 ; l<length ; l++ ) {
			fputc(buff[l],STREAM_PI);
		}
	} else if ( RS485_MODE_NMEA0183_RX==config.rs485_port_mode ) {
		/* do something */
	}
}


void main(void) {
	int8 i;

	i=restart_cause();

	init();


#if 0
	/* debugging messages sent on RS-485 port ... so we will start transmitting */
	output_high(RS485_DE);
	output_high(RS485_NRE);


	fprintf(STREAM_RS485,"# pcwx %s\r\n",__DATE__);
	fprintf(STREAM_RS485,"# restart_cause()=%u ",i);

	switch ( i ) {
		case WDT_TIMEOUT: fprintf(STREAM_RS485,"WDT_TIMEOUT"); break;
		case MCLR_FROM_SLEEP: fprintf(STREAM_RS485,"MCLR_FROM_SLEEP"); break;
		case MCLR_FROM_RUN: fprintf(STREAM_RS485,"MCLR_FROM_RUN"); break;
		case NORMAL_POWER_UP: fprintf(STREAM_RS485,"NORMAL_POWER_UP"); break;
		case BROWNOUT_RESTART: fprintf(STREAM_RS485,"BROWNOUT_RESTART"); break;
		case WDT_FROM_SLEEP: fprintf(STREAM_RS485,"WDT_FROM_SLEEP"); break;
		case RESET_INSTRUCTION: fprintf(STREAM_RS485,"RESET_INSTRUCTION"); break;
		default: fprintf(STREAM_RS485,"unknown!");
	}
	fprintf(STREAM_RS485,"\r\n");
#endif

//	fprintf(STREAM_RS485,"# read_param_file() starting ...");
	read_param_file();
//	fprintf(STREAM_RS485," complete\r\n");


	if ( config.modbus_address > 128 ) {
//		fprintf(STREAM_RS485,"# write_default_param_file() starting ...");
		write_default_param_file();
//		fprintf(STREAM_RS485," complete\r\n");
	}
//	fprintf(STREAM_RS485,"# config.modbus_address=%u\r\n",config.modbus_address);

	/* start Modbus slave */
	setup_uart(TRUE);
	/* modbus_init turns on global interrupts */
//	fprintf(STREAM_RS485,"# modbus_init() starting ...");
	modbus_init();
//	fprintf(STREAM_RS485," complete\r\n");

//	fprintf(STREAM_RS485,"# bridged_uarts=%u\r\n",current.bridged_uarts);

	/* Prime ADC filter */
	for ( i=0 ; i<30 ; i++ ) {
		adc_update();
	}

	/* set power switch to initial state */
	current.p_on=config.power_startup;


#if 0
	/* shut off RS-485 transmit once transmit buffer is empty */
	while ( ! TRMT2 )
		;
	output_low(RS485_DE);
	output_low(RS485_NRE);
	/* done with RS-485 port startup message */
#endif

	fprintf(STREAM_PI,"# pcwx %s\r\n",__DATE__);

	for ( ; ; ) {
		restart_wdt();

#if 0
		if ( current.bridged_uarts ) {
			disable_interrupts(INT_TIMER2);
			if ( kbhit(STREAM_RS485) ) {
				fputc(fgetc(STREAM_RS485),MODBUS_SERIAL);
			}

			if ( !bit_test(timers.port_b,BUTTON_BIT) ) {
				current.bridged_uarts=0;
				enable_interrupts(INT_TIMER2);
			}

			continue;
		} 
#endif

		if ( timers.now_millisecond ) {
			periodic_millisecond();
		}


		if ( timers.now_adc_sample ) {
			timers.now_adc_sample=0;
			adc_update();
		}

//		if ( ! current.bridged_uarts ) {
			modbus_process();
//		}


		if ( timers.now_parse_rda2 ) {
			timers.now_parse_rda2=0;
			rs485_to_host();
		}

	}
}