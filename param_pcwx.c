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

	config.modbus_address=38;
	config.rs485_port_mode=RS485_MODE_NMEA0183_RX; // RS485_MODE_MODBUS_BRIDGE;
//	config.modbus_address=128; /* use any address */

	config.serial_prefix='P';
	config.serial_number=9876;

	config.adc_sample_ticks=20;
	config.allow_bootload_request=0;

	config.watchdog_seconds_max=630; /* 10 minutes & 30 seconds */
	config.pi_offtime_seconds=2;
	config.power_startup=0;

	/* clear NMEA0183 sentence character array */
	memset(config.nmea0183_sentence,0,sizeof(config.nmea0183_sentence));
	/* set defaults. Can set the first 11 this way. They are 6 bytes long, not null terminated.
	Can use normal string functions (null terminated) to set the first 11 of them in order.
	12th would go 1 byte past the end of array */
	strcpy(config.nmea0183_sentence[0],"$GPRMC");
	strcpy(config.nmea0183_sentence[1],"$WIMDA");

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


