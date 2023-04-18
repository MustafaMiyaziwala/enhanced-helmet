#include "audio.h"


static inline uint8_t fill_audio_buffer(Audio* audio, uint8_t buf_bank) {
	UINT bytes_to_read = audio->wav_header.file_size < AUDIO_BUF_LEN
							? audio->wav_header.file_size : AUDIO_BUF_LEN;
	UINT bytes_read;
	FRESULT res;

	res = f_read(audio->fil, audio->dac_buf[buf_bank], bytes_to_read, &bytes_read);

	if (res != FR_OK || !bytes_read) {
		return 0;
	}

	audio->wav_header.file_size -= bytes_read;
	audio->dac_flag |= (1 << buf_bank);
	return 1;
}


static inline void stop_audio(Audio* audio) {
	HAL_TIM_Base_Stop_IT(audio->htim);
	HAL_GPIO_WritePin(audio->amp_enable_port, audio->amp_enable_pin, GPIO_PIN_RESET);
	shutdown_dac(audio->ext_dac);
	//f_close(audio->fil);
	audio->dac_flag = 0;
}

static inline void play_next(Audio* audio) {
	const TCHAR* filename = audio->queue[audio->read_pos];

	if (!filename) {
		stop_audio(audio);
		return;
	}

	UINT count;
	f_open(audio->fil, filename, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	f_read(audio->fil, &audio->wav_header, sizeof(WAV_Header), &count);

	audio->queue[audio->read_pos] = NULL;
 	audio->read_pos = (audio->read_pos + 1) % MAX_AUDIO_QUEUE_LEN;

	audio->bytes_left = audio->wav_header.file_size - 2048;
	audio->dac_buf_bank = 0;
	audio->dac_buf_idx = 0;

	fill_audio_buffer(audio, 0);
	fill_audio_buffer(audio, 1);


	if (!is_playing(audio)) {
		HAL_GPIO_WritePin(audio->amp_enable_port, audio->amp_enable_pin, GPIO_PIN_SET);
		audio->dac_flag |= (1 << 2);
	}
}

void audio_init(Audio* audio) {
	clear_queue(audio);
}

void clear_queue(Audio* audio) {
	HAL_TIM_Base_Stop_IT(audio->htim);
	audio->dac_flag = 0;

	for (uint8_t i = 0; i < MAX_AUDIO_QUEUE_LEN; ++i) {
		audio->queue[i] = NULL;
	}

	audio->read_pos = 0;
	audio->write_pos = 0;
	audio->bytes_left = 0;
	f_close(audio->fil);
}

uint8_t is_playing(Audio* audio) {
	return audio->dac_flag & (1 << 2);
}


void play_wav(Audio* audio, const TCHAR* filename) {
	audio->queue[audio->write_pos] = filename;
	audio->write_pos = (audio->write_pos + 1) % MAX_AUDIO_QUEUE_LEN;
	//audio->dac_flag |= (1 << 2);
	HAL_TIM_Base_Start_IT(audio->htim);
}

void check_and_fill_audio_buf(Audio* audio) {
	if (!(is_playing(audio))) {
		return;
	}

	if (!(audio->dac_flag & 1)) {
		fill_audio_buffer(audio, 0);
	}

	if (!(audio->dac_flag & (1 << 1))) {
		fill_audio_buffer(audio, 1);
	}
}

void audio_callback(Audio* audio) {

	if (!audio->bytes_left) {
		shutdown_dac(audio->ext_dac);
		f_close(audio->fil);
		play_next(audio);
		return;
	}

	if (audio->dac_buf_idx >= AUDIO_BUF_LEN) {
		audio->dac_flag &= ~(1 << audio->dac_buf_bank);
		audio->dac_buf_bank = (audio->dac_buf_bank + 1) % 2;
		audio->dac_buf_idx = 0;
	}

	// ERROR CASE: stop and reinitialize
	if (!(audio->dac_flag & 0b11)) {
//		stop_audio(audio);
//		audio_init(audio);
//		printf("DAC error?\r\n");
		audio->dac_flag |= (1 << 5);
		return;
	}

	write_to_dac(audio->ext_dac,
			audio->dac_buf[audio->dac_buf_bank][audio->dac_buf_idx++]);

	--audio->bytes_left;
}
