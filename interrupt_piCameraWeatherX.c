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
		if ( 1 == ext0_state ) {
			/* currently counting, time to finish */
			ext0_count=0;
			current.pulse_period[0]=timers.pulse_period[0];
			if ( current.pulse_period[0] < current.pulse_min_period[0] ) {
				current.pulse_min_period[0]=current.pulse_period[0];
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
		if ( 1 == ext1_state ) {
			/* currently counting, time to finish */
			ext1_count=0;
			current.pulse_period[1]=timers.pulse_period[1];
			if ( current.pulse_period[1] < current.pulse_min_period[1] ) {
				current.pulse_min_period[1]=current.pulse_period[1];
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
		if ( 1 == ext2_state ) {
			/* currently counting, time to finish */
			ext2_count=0;
			current.pulse_period[2]=timers.pulse_period[2];
			if ( current.pulse_period[2] < current.pulse_min_period[2] ) {
				current.pulse_min_period[2]=current.pulse_period[2];
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
}


void foo_isr_10ms(void) {
	static int16 uptimeticks=0;
	static int16 adcTicks=0;
	static int8 ticks=0;

	clear_interrupt(INT_TIMER1);
	set_timer1(45536);

	output_high(PIN_D5);

	/* anemometers quit moving */
	if ( 0xffff == timers.pulse_period[0] )
				current.pulse_period[0]=0;
	if ( 0xffff == timers.pulse_period[1] )
				current.pulse_period[1]=0;
	if ( 0xffff == timers.pulse_period[2] )
				current.pulse_period[2]=0;

	/* seconds since last query */
	if ( current.interval_milliseconds < 65535 ) {
		current.interval_milliseconds++;
	}

	/* seconds */
	ticks++;
	if ( 100 == ticks ) {
		ticks=0;
	}

	/* uptime counter */
	uptimeTicks++;
	if ( 6000 == uptimeTicks ) {
		uptimeTicks=0;
		if ( current.uptime_minutes < 65535 ) 
			current.uptime_minutes++;
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

	/* LEDs */
	if ( 0==timers.led_on_green ) {
		output_low(LED_GREEN);
	} else {
		output_high(LED_GREEN);
		timers.led_on_green--;
	}

}
