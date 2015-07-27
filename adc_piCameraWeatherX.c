const int8 adcChannelMap[8]={AN_IN_VOLTS, AN_TEMPERATURE, AN_WIND_DIR_0, AN_WIND_DIR_1, AN_USER_USER_0, AN_USER_USER_1, AN_USER_USER_2, AN_USER_USER_3};

int16 adc_get(int8 ch) {
	int16 sum;
	int8 i;

	// Calculate the mean.  This is done by summing up the
	// values and dividing by the number of elements.
	sum = 0;
	for( i = 0; i < 16 ; i++ ) {
		sum += current.adc_buffer[ch][i];;
	}

	/* divide sum by our 16 samples and round by adding 8 */
	return ( (sum+8) >> 4 );
}


void adc_update(void) {
	int8 i;

	/* wrap buffer around */
	current.adc_buffer_index++;
	if ( current.adc_buffer_index >= 16 )
		current.adc_buffer_index=0;


	for ( i=0 ; i<8 ; i++ ) {
		if ( 0==i ) { 
			/* PIC18F4523 can't individually select analogs. And our input voltage measurement got connected to AN11.
			So if we want to turn on A11, we have to turn on AN8, AN9, AN10. And AN8 and AN10 happen to be INT2 and INT1.
			Doing the anlog select causes those interrupts to trigger. So either no input voltage or no INT1 or INT2
			with this revision of hardware */

			current.adc_buffer[i][current.adc_buffer_index] = 0;
			continue;
		}

//			setup_adc_ports(AN0_TO_AN11,VSS_VREF);
			/* this will cause INT_EXT1 and INT_EXT2 to be analog while we do this sample. Potential for timing
			inaccuracy? */
//		} else if ( 1==i ) {
//			setup_adc_ports(AN0_TO_AN7,VSS_VREF);
//		}

		set_adc_channel(adcChannelMap[i]);
		delay_us(3);

		current.adc_buffer[i][current.adc_buffer_index] = read_adc();
		current.adc_std_dev[i]=0;

		delay_us(3);
	}




}