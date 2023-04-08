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
	shutdown_dac(audio->ext_dac);
	f_close(audio->fil);
	audio->dac_flag &= ~(1 << 2);
}


void play_wav(Audio* audio, const TCHAR* filename) {
	HAL_TIM_Base_Stop_IT(audio->htim);
	shutdown_dac(audio->ext_dac);

	UINT count;
	f_open(audio->fil, filename, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	f_read(audio->fil, &audio->wav_header, sizeof(WAV_Header), &count);

	audio->bytes_left = audio->wav_header.file_size;

	fill_audio_buffer(audio, 0);
	fill_audio_buffer(audio, 1);

	audio->dac_flag |= (1 << 2);

	HAL_TIM_Base_Start_IT(audio->htim);
}

void check_and_fill_audio_buf(Audio* audio) {
	if (!(audio->dac_flag & (1 << 2))) {
		return;
	}

	if (!(audio->dac_flag & 1)) {
		fill_audio_buffer(audio, 0);
	}

	if (!(audio->dac_flag & (1 << 1))) {
		fill_audio_buffer(audio, 1);
	}
}

void audio_callback(Audio* audio, TIM_HandleTypeDef *htim) {
	if (audio->htim != htim) {
		return;
	}

	if (!audio->bytes_left) {
		stop_audio(audio);
		return;
	}

	if (audio->dac_buf_idx >= AUDIO_BUF_LEN) {
		audio->dac_flag &= ~(1 << audio->dac_buf_bank);
		audio->dac_buf_bank = (audio->dac_buf_bank + 1) % 2;
		audio->dac_buf_idx = 0;
	}

	if (!(audio->dac_flag & 0b11)) {
		stop_audio(audio);
		return;
	}

	write_to_dac(audio->ext_dac,
			audio->dac_buf[audio->dac_buf_bank][audio->dac_buf_idx++]);

	--audio->bytes_left;
}
