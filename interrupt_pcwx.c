/* HIGH priority interrupt will interrupt other interrupts. Compiler will
automatically save all registers ... which seems to incur about a 60 cycle
penalty 

this ISR polls anemometers and counts between falling edges.
*/
#int_timer2 HIGH
void isr_100us(void) {
	static int8 tick=0;

	/* anemometer polling state variables */
	/* anemometer 0 / PIN_B0 */
	short ext0_count;
	short ext0_now;
	static short ext0_last=0;
	static short ext0_state=0;

	/* anemometer 1 / PIN_B1 */
	short ext1_count;
	short ext1_now;
	static short ext1_last=0;
	static short ext1_state=0;

	/* anemometer 2 / PIN_B2 */
	short ext2_count;
	short ext2_now;
	static short ext2_last=0;
	static short ext2_state=0;

//	output_high(TP_RED);
	
	/* count time between falling edges */
	if ( ext0_count && 0xffff != timers.pulse_period[0] )
		timers.pulse_period[0]++;
	if ( ext1_count && 0xffff != timers.pulse_period[1] )
		timers.pulse_period[1]++;
	if ( ext2_count && 0xffff != timers.pulse_period[2] )
		timers.pulse_period[2]++;

	/* anemometer 0 / PIN_B0 trigger on falling edge */
	ext0_now=input(PIN_B0);
	if ( 0 == ext0_now && 1 == ext0_last ) {
		current.pulse_count[0]++;
		current.pulse_sum[0]++;
		if ( 1 == ext0_state ) {
			/* currently counting, time to finish */
			ext0_count=0;
			current.pulse_period[0]=timers.pulse_period[0];
			if ( current.pulse_period[0] < current.pulse_min_period[0] ) {
				current.pulse_min_period[0]=current.pulse_period[0];
			}
			if ( current.pulse_period[0] > current.pulse_max_period[0] && current.pulse_period[0] != 65535  ) {
				current.pulse_max_period[0]=current.pulse_period[0];
			}
			ext0_state=0;
		}
		if ( 0 == ext0_state ) {
			/* not counting, time to start */
			timers.pulse_period[0]=0;
			ext0_count=1;
			ext0_state=1;
		}
	}
	ext0_last = ext0_now;

	/* anemometer 1 / PIN_B1 trigger on falling edge */
	ext1_now=input(PIN_B1);
	if ( 0 == ext1_now && 1 == ext1_last ) {
		current.pulse_count[1]++;
		current.pulse_sum[1]++;
		if ( 1 == ext1_state ) {
			/* currently counting, time to finish */
			ext1_count=0;
			current.pulse_period[1]=timers.pulse_period[1];
			if ( current.pulse_period[1] < current.pulse_min_period[1] ) {
				current.pulse_min_period[1]=current.pulse_period[1];
			}
			if ( current.pulse_period[1] > current.pulse_max_period[1] && current.pulse_period[1] != 65535 ) {
				current.pulse_max_period[1]=current.pulse_period[1];
			}
			ext1_state=0;
		}
		if ( 0 == ext1_state ) {
			/* not counting, time to start */
			timers.pulse_period[1]=0;
			ext1_count=1;
			ext1_state=1;
		}
	}
	ext1_last = ext1_now;

	/* anemometer 2 / PIN_B2 trigger on falling edge */
	ext2_now=input(PIN_B2);
	if ( 0 == ext2_now && 1 == ext2_last ) {
		current.pulse_count[2]++;
		current.pulse_sum[2]++;
		if ( 1 == ext2_state ) {
			/* currently counting, time to finish */
			ext2_count=0;
			current.pulse_period[2]=timers.pulse_period[2];
			if ( current.pulse_period[2] < current.pulse_min_period[2] ) {
				current.pulse_min_period[2]=current.pulse_period[2];
			}
			if ( current.pulse_period[2] > current.pulse_max_period[2] && current.pulse_period[2] != 65535 ) {
				current.pulse_max_period[2]=current.pulse_period[2];
			}
			ext2_state=0;
		}
		if ( 0 == ext2_state ) {
			/* not counting, time to start */
			timers.pulse_period[2]=0;
			ext2_count=1;
			ext2_state=1;
		}
	}
	ext2_last = ext2_now;


	/* every 10 cycles we tell main() loop to do milisecond activities */
	tick++;
	if ( 10 == tick ) {
		tick=0;
		timers.now_millisecond=1;
	}

//	output_low(TP_RED);
}

#int_rda2
void isr_rda2() {
	int8 c;

	c=fgetc(STREAM_RS485);
	current.rda2_bytes_received++;

	if ( RS485_MODE_MODBUS_BRIDGE == config.rs485_port_mode ) {
		/* add to buffer to send to PI */

		timers.rda2_buff_gap=0;

		if ( timers.rda2_buff_pos < sizeof(timers.rda2_buff)-1 ) {
			timers.rda2_buff[timers.rda2_buff_pos]=c;
			timers.rda2_buff_pos++;
		}
	} else if ( RS485_MODE_NMEA0183_RX == config.rs485_port_mode ) {
		/* if we are in NMEA mode, we can also recognize \n or \r as the end of a sentence */
		if ( '\n' == c || '\r' == c  ) {
			if  ( timers.rda2_buff_pos > 0 ) {
				/* mark new packet by a fake gap */
				timers.rda2_buff_gap=20;
			}  else {
				/* do nothing if we are at beginning of next packet */
			}
		} else {
			/* add to buffer */
			timers.rda2_buff_gap=0;

			if ( timers.rda2_buff_pos < sizeof(timers.rda2_buff)-1 ) {
				timers.rda2_buff[timers.rda2_buff_pos]=c;
				timers.rda2_buff_pos++;
			}
		}
	}
}

#if 1
/* transmit buffer empty for Modbus to raspberry pi buffer */
#int_tbe
void isr_uart1_tbe() {
	output_high(_PIC_TO_PI);
	if ( timers.rda_tx_pos >= timers.rda_tx_length ) {
		/* done transmitting */
		timers.now_rda_tx_done=1;
		disable_interrupts(INT_TBE);
	} else {
		/* put another character into TX buffer */
		fputc(timers.rda_tx_buff[timers.rda_tx_pos], STREAM_PI);
		timers.rda_tx_pos++;
	}
	output_low(_PIC_TO_PI);
}
#endif

/*  Raspberry PI connected serial port*/
#int_rda
void isr_uart1_rx() {
	int8 c;

	c=fgetc(STREAM_PI);

	current.rda_bytes_received++;

	/* Modbus */
	if (!modbus_serial_new) {
		if(modbus_serial_state == MODBUS_GETADDY) {
			modbus_serial_crc.d = 0xFFFF;
			modbus_rx.address = c;
			modbus_serial_state++;
			modbus_rx.len = 0;
			modbus_rx.error=0;
		} else if(modbus_serial_state == MODBUS_GETFUNC) {
			modbus_rx.func = c;
			modbus_serial_state++;
		} else if(modbus_serial_state == MODBUS_GETDATA) {
			if (modbus_rx.len>=MODBUS_SERIAL_RX_BUFFER_SIZE) {
				modbus_rx.len=MODBUS_SERIAL_RX_BUFFER_SIZE-1;
			}
			modbus_rx.data[modbus_rx.len]=c;
			modbus_rx.len++;
		}

		modbus_calc_crc(c);
		modbus_enable_timeout(TRUE);
	}
}