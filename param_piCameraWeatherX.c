#inline
char xor_crc(char oldcrc, char data) {
	return oldcrc ^ data;
}

char EEPROMDataRead( int16 address, int8 *data, int16 count ) {
	char crc=0;

	while ( count-- != 0 ) {
		*data = read_eeprom( address++ );
		crc = xor_crc(crc,*data);
		data++;
	}
	return crc;
}

char EEPROMDataWrite( int16 address, int8 *data, int16 count ) {
	char crc=0;

	while ( count-- != 0 ) {
		/* restart_wdt() */
		crc = xor_crc(crc,*data);
		write_eeprom( address++, *data++ );
	}

	return crc;
}

void write_param_file() {
	int8 crc;

	/* write the config structure */
	crc = EEPROMDataWrite(PARAM_ADDRESS,(void *)&config,sizeof(config));
	/* write the CRC was calculated on the structure */
	write_eeprom(PARAM_CRC_ADDRESS,crc);
}

void write_default_param_file() {
	/* red LED for 1.5 seconds */
	timers.led_on_green=150;

	config.modbus_address=24;

	config.serial_prefix='P';
	config.serial_number=9876;

#if 0
	config.adc_average_mode[0]=ADC_AVERAGE_NORMAL;
	config.adc_average_mode[1]=ADC_AVERAGE_NORMAL;
	config.adc_average_mode[2]=ADC_AVERAGE_NORMAL;
	config.adc_average_mode[3]=ADC_AVERAGE_NORMAL;
	config.adc_average_mode[4]=ADC_AVERAGE_NORMAL;
	config.adc_average_mode[5]=ADC_AVERAGE_NORMAL;
	config.adc_average_mode[6]=ADC_AVERAGE_NORMAL;
	config.adc_average_mode[7]=ADC_AVERAGE_NORMAL;
#endif
	config.adc_sample_ticks=20;

	/* write them so next time we use from EEPROM */
	write_param_file();

}


void read_param_file() {
	int8 crc;

	crc = EEPROMDataRead(PARAM_ADDRESS, (void *)&config, sizeof(config)); 
		
	if ( crc != read_eeprom(PARAM_CRC_ADDRESS) ) {
		write_default_param_file();
	}
}


