#define MAX_STATUS_REGISTER  53

#define MIN_CONFIG_REGISTER  1000
#define MAX_CONFIG_REGISTER  1017

#define MIN_COEFF_REGISTER   1100
#define MAX_COEFF_REGISTER   1108


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
		case 44: return (int16) current.uptime_minutes; /* uptime seconds */
		/* 45 triggers a new measurement */
		case 45: reset_counters(); return (int16) 0;
		/* 46 triggers a reset of pulse sum */
		case 46: reset_pulse_sum(); return (int16) 0;
		/* modbus statistics */
		case 47: return (int16) current.modbus_our_packets;
		case 48: return (int16) current.modbus_other_packets;
		case 49: return (int16) current.modbus_last_error;
		/* 50 triggers a modbus statistics reset */
		case 50: reset_modbus_stats(); return (int16) 0;

		/* configuration */
//		case 1000: return (int16) modbus_rx.len;
		case 1000: return (int16) config.serial_prefix;
		case 1001: return (int16) config.serial_number;
		case 1002: return (int16) 'X';
		case 1003: return (int16) 'G';
		case 1004: return (int16) 'X';
		case 1005: return (int16) 1;
		case 1006: return (int16) config.modbus_address;
//		case 1007: return (int16) config.modbus_speed;
//		case 1008: return (int16) config.modbus_mode;
//		case 1009: return (int16) config.worldData_seconds;
//		case 1010: return (int16) config.worldData_trigger_prefix;
//		case 1011: return (int16) config.worldData_trigger_number;
//		case 1012: return (int16) config.io_enable;
//		case 1013: return (int16) config.io_mode[IO_LINE_XBEE_SLEEP];
//		case 1014: return (int16) config.io_mode[IO_LINE_XBEE_RTS];
//		case 1015: return (int16) config.io_mode[IO_LINE_XBEE_CTS];
//		case 1016: return (int16) config.modbus_strobe_address;
//		case 1017: return (int16) config.modbus_strobe_register;

#if 0
		case 1100: return (int16) config.adc_average_mode[0];
		case 1101: return (int16) config.adc_average_mode[1];
		case 1102: return (int16) config.adc_average_mode[2];
		case 1103: return (int16) config.adc_average_mode[3];
		case 1104: return (int16) config.adc_average_mode[4];
		case 1105: return (int16) config.adc_average_mode[5];
		case 1106: return (int16) config.adc_average_mode[6];
		case 1107: return (int16) config.adc_average_mode[7];
		case 1108: return (int16) config.adc_sample_ticks;
#endif

		/* we should have range checked, and never gotten here */
		default: return (int16) 65535;
	}

}


int8 modbus_valid_read_registers(int16 start, int16 end) {
	if ( 19999==start && 20000==end)
		return 1;


	if ( start >= MIN_COEFF_REGISTER && end <= MAX_COEFF_REGISTER+1 )
		return 1;
	
	if ( start >= MIN_CONFIG_REGISTER && end <= MAX_CONFIG_REGISTER+1 )
		return 1;
	

	/* end is always start + at least one ... so no need to test for range starting at 0 */
	if ( end <= MAX_STATUS_REGISTER+1)
		return 1;

	return 0;
}

int8 modbus_valid_write_registers(int16 start, int16 end) {
	if ( 19999==start && 20000==end)
		return 1;


	if ( start >= MIN_COEFF_REGISTER && end <= MAX_COEFF_REGISTER+1 )
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
			

		case 1006:
			/* Modbus address {0 to 127} */
			if ( value > 127 ) return ILLEGAL_DATA_VALUE;
			config.modbus_address=value;
			break;

		case 1108:
			/* ADC sample interval */
			timers.now_adc_reset_count=1;
			config.adc_sample_ticks=value;
			break;
		
		case 1100:
		case 1101:
		case 1102:
		case 1103:
		case 1104:
		case 1105:
		case 1106:
		case 1107:
			/* Analog channels averaging mode */
			address -= 1100;
			config.adc_average_mode[address]=value;
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
		if ( modbus_rx.address==config.modbus_address ) {
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
					    modbus_exception_rsp(config.modbus_address,modbus_rx.func,ILLEGAL_DATA_ADDRESS);
						current.modbus_last_error=ILLEGAL_DATA_ADDRESS;

						/* red LED for 1 second */
						timers.led_on_green=0;
					} else {
						modbus_read_register_response(modbus_rx.func,config.modbus_address,start_addr,num_registers);
					}
					break;
				case FUNC_WRITE_SINGLE_REGISTER: /* 6 */
					start_addr=make16(modbus_rx.data[0],modbus_rx.data[1]);

					/* try the write */
					result=modbus_write_register(start_addr,make16(modbus_rx.data[2],modbus_rx.data[3]));

					if ( result ) {
						/* exception */
						modbus_exception_rsp(config.modbus_address,modbus_rx.func,result);
						current.modbus_last_error=result;

						/* red LED for 1 second */
						timers.led_on_green=0;
					}  else {
						/* no exception, send ack */
						modbus_write_single_register_rsp(config.modbus_address,
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
							modbus_exception_rsp(config.modbus_address,modbus_rx.func,result);
							current.modbus_last_error=result;
	
							/* red LED for 1 second */
							timers.led_on_green=0;
			
							break;
						}
					}
		
					/* we could have gotten here with an exception already send, so only send if no exception */
					if ( 0 == result ) {
						/* no exception, send ack */
						modbus_write_multiple_registers_rsp(config.modbus_address,start_addr,num_registers);
					}

					break;  
				default:
					/* we don't support most operations, so return ILLEGAL_FUNCTION exception */
					modbus_exception_rsp(config.modbus_address,modbus_rx.func,ILLEGAL_FUNCTION);
					current.modbus_last_error=ILLEGAL_FUNCTION;

					/* red led for 1 second */
					timers.led_on_green=0;
			}
		} else {
			/* MODBUS packet for somebody else */
			if ( current.modbus_other_packets < 65535 )
				current.modbus_other_packets++;

			/* yellow LED 200 milliseconds */
			timers.led_on_green=20;
		}
	}
}