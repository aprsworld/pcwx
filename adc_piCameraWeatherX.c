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

int16 read_adc_channel(int8 channel) {
	ADCON0=(channel&0x0f)<<2 | 0b1;

	ADCON1=0b00010000; /* Vref+ external 5 volt reference, Vref- VSS */

	ADCON2=0b10111101; /* page 360 */

	ANCON0=0b11111111;
	ANCON1=0b00000010;


	bit_set(ADCON0,1);

	/* poll for conversion to be complete */
	while ( bit_test(ADCON0,1) ) 
		;

	return make16(ADRESH,ADRESL);

}


void adc_update(void) {
	int8 i;

	/* wrap buffer around */
	current.adc_buffer_index++;
	if ( current.adc_buffer_index >= 16 )
		current.adc_buffer_index=0;

	for ( i=0 ; i<8 ; i++ ) {
		

//		set_adc_channel(adcChannelMap[i]);

//		ADCON1=0b00010000; /* Vref+ external 5 volt reference, Vref- VSS */

//		current.adc_buffer[i][current.adc_buffer_index] = read_adc();
		current.adc_buffer[i][current.adc_buffer_index] = read_adc_channel(adcChannelMap[i]);

		current.adc_std_dev[i]=0;
	}




}