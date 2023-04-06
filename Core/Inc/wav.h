#ifndef __WAV_H
#define __WAV_H

#include <stdint.h>

typedef struct WAV_Header {
	uint32_t riff;
	uint32_t file_size;
	uint32_t file_type;
	uint32_t format_chunk_marker;
	uint32_t format_chunk_length;
	uint16_t format_type;
	uint16_t num_channels;
	uint32_t sampling_freq;
	uint32_t bytes_per_second;
	uint16_t block_size;
	uint32_t data_header;
	uint32_t data_length;
} WAV_Header;



#endif
