#ifndef __AUDIO_H
#define __AUDIO_H

#include <stdint.h>
#include "fatfs.h"
#include "ext_dac.h"
#include "stm32f4xx_hal.h"

#define AUDIO_BUF_LEN 1024

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


typedef struct Audio {
	FATFS* fs;
	FIL* fil;
	Ext_DAC_t* ext_dac;
	TIM_HandleTypeDef* htim;

	WAV_Header wav_header;
	uint8_t dac_buf[2][AUDIO_BUF_LEN];
	uint32_t bytes_left;
	uint16_t dac_buf_idx;
	uint8_t dac_buf_bank;
	uint8_t dac_flag;
} Audio;


/**
 * Call to start playing a specified wav file
 */
void play_wav(Audio* audio, const TCHAR* filename);


/*
 * Called periodically in super loop to check dac buffer
 * flags and read data from SD card
 */
void check_and_fill_audio_buf(Audio* audio);


/*
 * Callback to be executed in timer period exceeded ISR
 * to output the next audio sample to the DAC
 */
void audio_callback(Audio* audio, TIM_HandleTypeDef *htim);





#endif
