#ifndef _AUDIO_PLAYER_H
#define _AUDIO_PLAYER_H

typedef struct {
	DAC_HandleTypeDef* hdac;
	TIM_HandleTypeDef* htim;
	uint32_t dac_channel;
} audioController;


// MAX file name is 64 chars
char soundtrackDirectory [][64] = {
		"oiia_AUDIO.txt",
		"meow_AUDIO.txt",
		"low_battery_AUDIO.txt",
		"intruder_detected_AUDIO.txt"
};

uint16_t soundtrackFileSize [] = {
		57208,
		19069,
		7200,
		11040
};

uint16_t* soundtrackAddr [] = {
		&oiia_audio,
		&meow_audio,
		&low_battery_audio,
		&intruder_alert_audio
};

typedef enum {
	OIIA_AUDIO,
	MEOW_AUDIO,
	LOW_BATTERY_AUDIO,
	INTRUDER_ALERT_AUDIO
} soundtrack_index;



void initAudio(audioController* audio_controller) {
    HAL_TIM_Base_Start(audio_controller->htim);
}


// Initialize SD card first before calling this function
void loadAudioFiles() {
	char line[20] = {'\0'};

	// Currently hardcoded to load 4 audio tracks
	for (uint8_t soundtrack = 0; soundtrack < 4; soundtrack++) {
		char name[64] = {'\0'};

		strcpy(name, soundtrackDirectory[soundtrack]);

		FRESULT res = f_open(&SDFile, name, FA_READ);

		for (int i = 0; i < soundtrackFileSize[soundtrack]; i++) {
		  f_gets(&line, sizeof(line), &SDFile);

		  int val = (line[0] - 48) * 1000 + (line[1] - 48) * 100 + (line[2] - 48) * 10 + (line[3] - 48);
		  soundtrackAddr[soundtrack][i] = val;
		}

		f_close(&SDFile);
	}
}


void playAudio(audioController* audio_controller, soundtrack_index sound) {
	HAL_DAC_Start_DMA(audio_controller->hdac, audio_controller->dac_channel, soundtrackAddr[sound], soundtrackFileSize[sound], DAC_ALIGN_12B_R);
}


#endif
