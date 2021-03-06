#define MAX_STATUS_REGISTER          55

#define MIN_CONFIG_REGISTER          1000
#define MAX_CONFIG_REGISTER          1014

#define MIN_NMEA0183_CONFIG_REGISTER 1100
#define MAX_NMEA0183_CONFIG_REGISTER 1100 + N_NMEA0183_SENTENCES*6

#define MIN_EE_REGISTER              2000
#define MAX_EE_REGISTER              MIN_EE_REGISTER + 512

#define MIN_NMEA0183_BYTE_REGISTER   5000
#define MAX_NMEA0183_BYTE_REGISTER   MIN_NMEA0183_BYTE_REGISTER + N_NMEA0183_SENTENCES*80

#define MIN_NMEA0183_WORD_REGISTER   6000
#define MAX_NMEA0183_WORD_REGISTER   MIN_NMEA0183_WORD_REGISTER + N_NMEA0183_SENTENCES*40

#define MIN_NMEA0183_META_REGISTER   6500
#define MAX_NMEA0183_META_REGISTER   MIN_NMEA0183_META_REGISTER + N_NMEA0183_SENTENCES*2


/* This function may come in handy for you since MODBUS uses MSB first. */
int8 swap_bits(int8 c) {
	return ((c&1)?128:0)|((c&2)?64:0)|((c&4)?32:0)|((c&8)?16:0)|((c&16)?8:0)|((c&32)?4:0)|((c&64)?2:0)|((c&128)?1:0);
}

void reset_modbus_stats(void) {
	current.modbus_our_packets=0;
	current.modbus_other_packets=0;
	current.modbus_last_error=0;
}

void reset_counters(void) {
	disable_interrupts(GLOBAL);

	current.pulse_count[0]=0;
	current.pulse_count[1]=0;
	current.pulse_count[2]=0;

	/* pulse period is reset in interrupt */
	current.pulse_min_period[0]=65535;
	current.pulse_min_period[1]=65535;
	current.pulse_min_period[2]=65535;
	
	current.pulse_max_period[0]=0;
	current.pulse_max_period[1]=0;
	current.pulse_max_period[2]=0;
	

	current.interval_milliseconds=0;

	enable_interrupts(GLOBAL);
}

void reset_pulse_sum(void) {
	disable_interrupts(GLOBAL);
	current.pulse_sum[0]=0;
	current.pulse_sum[1]=0;
	current.pulse_sum[2]=0;
	enable_interrupts(GLOBAL);
}

int32 get_pulse_sum(int8 ch) {
	int32 l;

	disable_interrupts(GLOBAL);
	l=current.pulse_sum[ch];
	enable_interrupts(GLOBAL);

	return l;
}

int16 map_modbus(int16 addr) {
	static u_lblock ps;
	int8 n,o;
	int8 *p;

	if ( addr >= MIN_EE_REGISTER && addr < MAX_EE_REGISTER ) {
		return (int16) read_eeprom(addr - MIN_EE_REGISTER + EE_FOR_HOST_ADDRESS);
	}

	if ( addr >= MIN_NMEA0183_CONFIG_REGISTER && addr < MAX_NMEA0183_CONFIG_REGISTER ) {
		/* get rid of our base */
		n = (addr-MIN_NMEA0183_CONFIG_REGISTER);

		/* configurable sentence parts are the first 6 characters */
		o = n % 6; /* offset into sentence */
		n = n / 6; /* number of sentence */

		return (int16) config.nmea0183_sentence[n][o];
	}

	/* one byte per register for NMEA0183 sentences */
	if ( addr >= MIN_NMEA0183_BYTE_REGISTER && addr < MAX_NMEA0183_BYTE_REGISTER ) {
		/* get rid of our base */
		addr = (addr-MIN_NMEA0183_BYTE_REGISTER);

 		p  = nmea.sentence[0];
		return (int16) p[addr];
	}

	/* two bytes per register for NMEA0183 sentences */
	if ( addr >= MIN_NMEA0183_WORD_REGISTER && addr < MAX_NMEA0183_WORD_REGISTER ) {
		/* get rid of our base */
		addr = (addr-MIN_NMEA0183_WORD_REGISTER);
		addr = addr * 2;

 		p  = nmea.sentence[0];
		return (int16) make16(p[addr],p[addr+1]);
	}


	switch ( addr ) {
		/* counters */
		case 0:  return (int16) current.pulse_count[0];
		case 1:  return (int16) current.pulse_period[0];
		case 2:  return (int16) current.pulse_min_period[0];
		case 3:  return (int16) current.pulse_max_period[0];
		case 4:  ps.word=get_pulse_sum(0); return (int16) ps.l[0];
		case 5:  return (int16) ps.l[1];

		case 6:  return (int16) current.pulse_count[1];
		case 7:  return (int16) current.pulse_period[1];
		case 8:  return (int16) current.pulse_min_period[1];
		case 9:  return (int16) current.pulse_max_period[1];
		case 10: ps.word=get_pulse_sum(1); return (int16) ps.l[0];
		case 11: return (int16) ps.l[1];


		case 12: return (int16) current.pulse_count[2];
		case 13: return (int16) current.pulse_period[2];
		case 14: return (int16) current.pulse_min_period[2];
		case 15: return (int16) current.pulse_max_period[2];
		case 16: ps.word=get_pulse_sum(2); return (int16) ps.l[0];
		case 17: return (int16) ps.l[1];


		/* analog channels */
		/* input voltage */
		case 18: return (int16) current.adc_buffer[0][current.adc_buffer_index];
		case 19: return (int16) adc_get(0);
		case 20: return (int16) current.adc_std_dev[0];
		/* wind dir 0 */
		case 21: return (int16) current.adc_buffer[1][current.adc_buffer_index];
		case 22: return (int16) adc_get(1);
		case 23: return (int16) current.adc_std_dev[1];
		/* wind dir 1 */
		case 24: return (int16) current.adc_buffer[2][current.adc_buffer_index];
		case 25: return (int16) adc_get(2);
		case 26: return (int16) current.adc_std_dev[2];
		/* temperature */
		case 27: return (int16) current.adc_buffer[3][current.adc_buffer_index];
		case 28: return (int16) adc_get(3);
		case 29: return (int16) current.adc_std_dev[3];
		/* user ADC 0 to 3 */
		case 30: return (int16) current.adc_buffer[4][current.adc_buffer_index];
		case 31: return (int16) adc_get(4);
		case 32: return (int16) current.adc_std_dev[4];

		case 33: return (int16) current.adc_buffer[5][current.adc_buffer_index];
		case 34: return (int16) adc_get(5);
		case 35: return (int16) current.adc_std_dev[5];

		case 36: return (int16) current.adc_buffer[6][current.adc_buffer_index];
		case 37: return (int16) adc_get(6);
		case 38: return (int16) current.adc_std_dev[6];

		case 39: return (int16) current.adc_buffer[7][current.adc_buffer_index];
		case 40: return (int16) adc_get(7);
		case 41: return (int16) current.adc_std_dev[7];

		/* status */
		case 42: return (int16) current.sequence_number++;
		case 43: return (int16) current.interval_milliseconds; /* milliseconds since last query */
		case 44: return (int16) current.uptime_minutes; 
		case 45: return (int16) current.watchdog_seconds; 

		/* triggers a new measurement */
		case 46: reset_counters(); return (int16) 0;
		/* triggers a reset of pulse sum */
		case 47: reset_pulse_sum(); return (int16) 0;
		/* modbus statistics */
		case 48: return (int16) current.modbus_our_packets;
		case 49: return (int16) current.modbus_other_packets;
		case 50: return (int16) current.modbus_last_error;
		/* triggers a modbus statistics reset */
		case 51: reset_modbus_stats(); return (int16) 0;
		
		/* meta */
		case 52: return (int16) current.rda_bytes_received;
		case 53: return (int16) current.rda2_bytes_received;
		case 54: return (int16) current.button_state;
		case 55: return (int16) current.latch_sw_magnet;

		/* configuration */
		case 1000: return (int16) config.serial_prefix;
		case 1001: return (int16) config.serial_number;
		case 1002: return (int16) 'P';
		case 1003: return (int16) 'W';
		case 1004: return (int16) 'X';
		case 1005: return (int16) 1;
		case 1006: return (int16) config.modbus_address;
		case 1007: return (int16) config.adc_sample_ticks;
		case 1008: return (int16) config.allow_bootload_request;
		case 1009: return (int16) config.watchdog_seconds_max;
		case 1010: return (int16) config.pi_offtime_seconds;
		case 1011: return (int16) config.power_startup;
		case 1012: return (int16) config.rs485_port_mode;
		case 1013: 
			if ( RS485_SPEED_1200 == config.rs485_port_speed )  return (int16) 1200;
			if ( RS485_SPEED_2400 == config.rs485_port_speed )  return (int16) 2400;
			if ( RS485_SPEED_4800 == config.rs485_port_speed )  return (int16) 4800;
			if ( RS485_SPEED_19200 == config.rs485_port_speed ) return (int16) 19200;
			if ( RS485_SPEED_38400 == config.rs485_port_speed ) return (int16) 38400;
			if ( RS485_SPEED_57600 == config.rs485_port_speed ) return (int16) 57600;

			return (int16) 9600;
		case 1014: return (int16) config.pic_to_pi_latch_mask;

		/* NMEA sentence age and length */
		case 6500: return (int16) nmea.sentence_age[0];
		case 6501: return (int16) nmea.sentence_length[0];
		case 6502: return (int16) nmea.sentence_age[1];
		case 6503: return (int16) nmea.sentence_length[1];
		case 6504: return (int16) nmea.sentence_age[2];
		case 6505: return (int16) nmea.sentence_length[2];
		case 6506: return (int16) nmea.sentence_age[3];
		case 6507: return (int16) nmea.sentence_length[3];
		case 6508: return (int16) nmea.sentence_age[4];
		case 6509: return (int16) nmea.sentence_length[4];
		case 6510: return (int16) nmea.sentence_age[5];
		case 6511: return (int16) nmea.sentence_length[5];
		case 6512: return (int16) nmea.sentence_age[6];
		case 6513: return (int16) nmea.sentence_length[6];
		case 6514: return (int16) nmea.sentence_age[7];
		case 6515: return (int16) nmea.sentence_length[7];
		case 6516: return (int16) nmea.sentence_age[8];
		case 6517: return (int16) nmea.sentence_length[8];
		case 6518: return (int16) nmea.sentence_age[9];
		case 6519: return (int16) nmea.sentence_length[9];
		case 6520: return (int16) nmea.sentence_age[10];
		case 6521: return (int16) nmea.sentence_length[10];
		case 6522: return (int16) nmea.sentence_age[11];
		case 6523: return (int16) nmea.sentence_length[11];


		/* we should have range checked, and never gotten here */
		default: return (int16) 65535;
	}

}


int8 modbus_valid_read_registers(int16 start, int16 end) {
	if ( 19999==start && 20000==end)
		return 1;

	if ( start >= MIN_NMEA0183_META_REGISTER && end <= MAX_NMEA0183_META_REGISTER ) 
		return 1;

	if ( start >= MIN_NMEA0183_WORD_REGISTER && end <= MAX_NMEA0183_WORD_REGISTER ) 
		return 1;

	if ( start >= MIN_NMEA0183_BYTE_REGISTER && end <= MAX_NMEA0183_BYTE_REGISTER ) 
		return 1;

	if ( start >= MIN_NMEA0183_CONFIG_REGISTER && end <= MAX_NMEA0183_CONFIG_REGISTER )
		return 1;

	if ( start >= MIN_CONFIG_REGISTER && end <= MAX_CONFIG_REGISTER+1 )
		return 1;

	if ( start >= MIN_EE_REGISTER && end <= MAX_EE_REGISTER+1 )
		return 1;
	

	/* end is always start + at least one ... so no need to test for range starting at 0 */
	if ( end <= MAX_STATUS_REGISTER+1)
		return 1;

	return 0;
}

int8 modbus_valid_write_registers(int16 start, int16 end) {
	if ( 19999==start && 20000==end)
		return 1;

	if ( start >= MIN_EE_REGISTER && end <= MAX_EE_REGISTER+1 )
		return 1;

	if ( start >= MIN_NMEA0183_CONFIG_REGISTER && end <= MAX_NMEA0183_CONFIG_REGISTER+1 )
		return 1;

	if ( start >= MIN_CONFIG_REGISTER && end <= MAX_CONFIG_REGISTER+1 )
		return 1;
	
	/* end is always start + at least one ... so no need to test for range starting at 0 */
	if ( end <= MAX_STATUS_REGISTER+1)
		return 1;

	return 0;
}

void modbus_read_register_response(function func, int8 address, int16 start_address, int16 register_count ) {
	int16 i;
	int16 l;

	modbus_serial_send_start(address, func); // FUNC_READ_HOLDING_REGISTERS);
	modbus_serial_putc(register_count*2);


	for( i=0 ; i<register_count ; i++ ) {
		l=map_modbus(start_address+i);
		modbus_serial_putc(make8(l,1));
  		modbus_serial_putc(make8(l,0));
	}

	modbus_serial_send_stop();
}

/* 
try to write the specified register
if successful, return 0, otherwise return a modbus exception
*/
exception modbus_write_register(int16 address, int16 value) {
	int8 n,o;

	if ( address >= MIN_EE_REGISTER && address < MAX_EE_REGISTER ) {
		if ( value > 256 ) return ILLEGAL_DATA_VALUE;
		write_eeprom(address - MIN_EE_REGISTER + EE_FOR_HOST_ADDRESS,(int8) value);
		return 0;
	}

	if ( address >= MIN_NMEA0183_CONFIG_REGISTER && address < MAX_NMEA0183_CONFIG_REGISTER ) {
		if ( value > 256 ) return ILLEGAL_DATA_VALUE;

		/* get rid of our base */
		n = (address-MIN_NMEA0183_CONFIG_REGISTER);

		/* configurable sentence parts are the first 6 characters */
		o = n % 6; /* offset into sentence */
		n = n / 6; /* number of sentence */

		config.nmea0183_sentence[n][o]=(int8) value;
		
		return 0;
	}


	/* if we have been unlocked, then we can modify serial number */
	if ( current.factory_unlocked ) {
		if ( 1000 == address ) {
			config.serial_prefix=value;
			return 0;
		} else if ( 1001 == address ) {
			config.serial_number=value;
			return 0;
		}
	}

	/* publicly writeable addresses */
	switch ( address ) {
		case 55:
			if ( 0 != value ) return ILLEGAL_DATA_VALUE;
			current.latch_sw_magnet=0;
			break;			

		case 1006:
			/* Modbus address {0 to 127 or 128 for respond to any} */
			if ( value > 128 ) return ILLEGAL_DATA_VALUE;
			config.modbus_address=value;
			break;

		case 1007:
			/* ADC sample interval */
			timers.now_adc_reset_count=1;
			config.adc_sample_ticks=value;
			break;

		case 1008:
			/* allow this processor to follow requests of the PIC BOOTLOAD REQUEST line to reset ourselves */
			if ( value > 1 ) return ILLEGAL_DATA_VALUE;
			config.allow_bootload_request=value;
			break;

		case 1009:
			config.watchdog_seconds_max=value;
			break;

		case 1010:
			if ( value < 1 ) return ILLEGAL_DATA_VALUE;
			config.pi_offtime_seconds=value;
			break;
		
		case 1011:
			if ( value > 1 ) return ILLEGAL_DATA_VALUE;
			config.power_startup=value;
			break;
		
		case 1012:
			if ( value > 2 ) return ILLEGAL_DATA_VALUE;
			config.rs485_port_mode=value;
			break;
		
		case 1013:

			n=0;
			if ( 1200 == value )  { n=1; config.rs485_port_speed=RS485_SPEED_1200; }
			if ( 2400 == value )  { n=1; config.rs485_port_speed=RS485_SPEED_2400; }
			if ( 4800 == value )  { n=1; config.rs485_port_speed=RS485_SPEED_4800; }
			if ( 9600 == value )  { n=1; config.rs485_port_speed=RS485_SPEED_9600; }
			if ( 19200 == value ) { n=1; config.rs485_port_speed=RS485_SPEED_19200; }
			if ( 38400 == value ) { n=1; config.rs485_port_speed=RS485_SPEED_38400; }
			if ( 57600 == value ) { n=1; config.rs485_port_speed=RS485_SPEED_57600; }

			if (  1==n ) {
				set_rs485_speed();
			}  else {
				return ILLEGAL_DATA_VALUE;
			}
			break;


		case 1014:
			if ( value > 1 ) return ILLEGAL_DATA_VALUE;
			config.pic_to_pi_latch_mask=value;
			break;

		case 1997:
			/* reset CPU */
			if ( 1 != value ) return ILLEGAL_DATA_VALUE;
			reset_cpu();
		case 1998:
			/* write default config to EEPROM */
			if ( 1 != value ) return ILLEGAL_DATA_VALUE;
			write_default_param_file();
			break;
		case 1999:
			/* write config to EEPROM */
			if ( 1 != value ) return ILLEGAL_DATA_VALUE;
			write_param_file();
			break;
		case 19999:
			/* unlock factory programming registers when we get 1802 in passcode register */
			if ( 1802 != value ) {
				current.factory_unlocked=0;
				return ILLEGAL_DATA_VALUE;
			}
			current.factory_unlocked=1;
			/* green LED for 2 seconds */
			timers.led_on_green=200;
			break;
		default:
			return ILLEGAL_DATA_ADDRESS;

	}

	/* must not have triggered an exception */
	return 0;
}


void modbus_process(void) {
	int16 start_addr;
	int16 num_registers;
	exception result;
	int8 i;


	/* check for message */
	if ( modbus_kbhit() ) {
//		output_high(TP_RED);

		if ( RS485_MODE_MODBUS_BRIDGE==config.rs485_port_mode && modbus_rx.address!=config.modbus_address ) {
			/* rebuld modbus packet and send to RS-485 port */

			/* start transmitting */
			output_high(RS485_DE);
			output_high(RS485_NRE);
			/* 3.5 character delay (3500000/baud) */
			delay_us(365); /* 9600 */

			/* address */
			fputc(modbus_rx.address,STREAM_RS485);
			delay_us(104); //one stop bit @ 9600 baud

			/* function */
			fputc(modbus_rx.func,STREAM_RS485);
			delay_us(104); //one stop bit @ 9600 baud

			/* data and (hopefully) CRC */
			for ( i=0 ; i<modbus_rx.len+2 ; i++ ) {
				fputc(modbus_rx.data[i],STREAM_RS485);
				delay_us(104); //one stop bit @ 9600 baud
			}

			/* wait for transmitter buffer to empty */
			while ( ! TRMT2 )
				;
			/* 3.5 character delay (3500000/baud) */
			delay_us(365); /* 9600 */
			/* shut off transmitter */
			output_low(RS485_DE);
			output_low(RS485_NRE);

		}

		if ( 128==config.modbus_address || modbus_rx.address==config.modbus_address ) {
			/* Modbus statistics */
			if ( current.modbus_our_packets < 65535 )
				current.modbus_our_packets++;
	
			/* green LED for 200 milliseconds */
			timers.led_on_green=20;

			switch(modbus_rx.func) {
				case FUNC_READ_HOLDING_REGISTERS: /* 3 */
				case FUNC_READ_INPUT_REGISTERS:   /* 4 */
					start_addr=make16(modbus_rx.data[0],modbus_rx.data[1]);
					num_registers=make16(modbus_rx.data[2],modbus_rx.data[3]);
	
					/* make sure our address is within range */
					if ( ! modbus_valid_read_registers(start_addr,start_addr+num_registers) ) {
					    modbus_exception_rsp(modbus_rx.address,modbus_rx.func,ILLEGAL_DATA_ADDRESS);
						current.modbus_last_error=ILLEGAL_DATA_ADDRESS;

						/* red LED for 1 second */
						timers.led_on_green=0;
					} else {
						modbus_read_register_response(modbus_rx.func,modbus_rx.address,start_addr,num_registers);
					}
					break;
				case FUNC_WRITE_SINGLE_REGISTER: /* 6 */
					start_addr=make16(modbus_rx.data[0],modbus_rx.data[1]);

					/* try the write */
					result=modbus_write_register(start_addr,make16(modbus_rx.data[2],modbus_rx.data[3]));

					if ( result ) {
						/* exception */
						modbus_exception_rsp(modbus_rx.address,modbus_rx.func,result);
						current.modbus_last_error=result;

						/* red LED for 1 second */
						timers.led_on_green=0;
					}  else {
						/* no exception, send ack */
						modbus_write_single_register_rsp(modbus_rx.address,
							start_addr,
							make16(modbus_rx.data[2],modbus_rx.data[3])
						);
					}
					break;
				case FUNC_WRITE_MULTIPLE_REGISTERS: /* 16 */
					start_addr=make16(modbus_rx.data[0],modbus_rx.data[1]);
					num_registers=make16(modbus_rx.data[2],modbus_rx.data[3]);

					/* attempt to write each register. Stop if exception */
					for ( i=0 ; i<num_registers ; i++ ) {
						result=modbus_write_register(start_addr+i,make16(modbus_rx.data[5+i*2],modbus_rx.data[6+i*2]));

						if ( result ) {
							/* exception */
							modbus_exception_rsp(modbus_rx.address,modbus_rx.func,result);
							current.modbus_last_error=result;
	
							/* red LED for 1 second */
							timers.led_on_green=0;
			
							break;
						}
					}
		
					/* we could have gotten here with an exception already send, so only send if no exception */
					if ( 0 == result ) {
						/* no exception, send ack */
						modbus_write_multiple_registers_rsp(modbus_rx.address,start_addr,num_registers);
					}

					break;  
				default:
					/* we don't support most operations, so return ILLEGAL_FUNCTION exception */
					modbus_exception_rsp(modbus_rx.address,modbus_rx.func,ILLEGAL_FUNCTION);
					current.modbus_last_error=ILLEGAL_FUNCTION;

					/* red led for 1 second */
					timers.led_on_green=0;
			}
			/* reset watchdog seconds now that we are done processing request */
			current.watchdog_seconds=0;

		} else {
			/* MODBUS packet for somebody else */
			if ( current.modbus_other_packets < 65535 )
				current.modbus_other_packets++;

			/* yellow LED 200 milliseconds */
			timers.led_on_green=10;
		}

	}
//	output_low(TP_RED);
}