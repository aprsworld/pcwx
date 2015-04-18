#define MEAN_FILTER_WIDTH    16
#define MEAN_FILTER_CHANNELS 8

int16 mean_filter_nJ(int8 ch, int16 latest_element) {
	static int16 input_buffer[MEAN_FILTER_CHANNELS][MEAN_FILTER_WIDTH];
	static int8 inbuf_index[MEAN_FILTER_CHANNELS];
	static int8 num_elements[MEAN_FILTER_CHANNELS];
	int32 mean;
	int32 sum;
	int8 i;

	// Insert incoming data element into circular input buffer.
	input_buffer[ch][inbuf_index[ch]] = latest_element;
	inbuf_index[ch]++;
	if(inbuf_index[ch] >= MEAN_FILTER_WIDTH)  // If index went past buffer end
	   inbuf_index[ch] = 0;       // then reset it to start of buffer

	if(num_elements[ch] < MEAN_FILTER_WIDTH)
	   num_elements[ch]++;

	// Calculate the mean.  This is done by summing up the
	// values and dividing by the number of elements.
	sum = 0;
	for(i = 0; i < num_elements[ch]; i++)
		sum += input_buffer[ch][i];

	// Round-off the result by adding half the divisor to
	// the numerator.
	mean = (sum + (int32)(num_elements[ch] >> 1)) / num_elements[ch];

	return((int16)mean);
}

/* 
special case of mean filter. For a 16 element filter, max value of 4096, and equal
number of samples for each channel. Filter result doesn't become valid until at least
16 samples have been added
*/
int16 mean_filter16_nJJ(int8 ch, int16 latest_element) {
	static int16 input_buffer[MEAN_FILTER_CHANNELS][MEAN_FILTER_WIDTH];
	static int8 inbuf_index[MEAN_FILTER_CHANNELS];
	int16 sum;
	int16 min, max;
	int8 i;


	// Insert incoming data element into circular input buffer.
	input_buffer[ch][inbuf_index] = latest_element;
	inbuf_index[ch]++;
	if(inbuf_index[ch] >= MEAN_FILTER_WIDTH)  // If index went past buffer end
	   inbuf_index[ch] = 0;       // then reset it to start of buffer

	// Calculate the mean.  This is done by summing up the
	// values and dividing by the number of elements.
	min=65535;
	max=0;
	sum = 0;
	for( i = 0; i < MEAN_FILTER_WIDTH ; i++ ) {
		sum += input_buffer[ch][i];

		if ( input_buffer[ch][i] > max )
			max=input_buffer[ch][i];
		if ( input_buffer[ch][i] < min )
			min=input_buffer[ch][i];
	}

	/* throw out the highest and lowest values */
	sum -= max;
	sum -= min;

	return ( (sum+7) / 14 );

	// Round-off the result by adding half the divisor to
	// the numerator.
//	return ( (sum+8) >> 4 );
}
