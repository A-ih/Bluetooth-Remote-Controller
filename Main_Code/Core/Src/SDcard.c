#include "SDcard.h"


// Mounts the SD card
SDcard_STATUS SD_Init() {
	FRESULT res = f_mount(&SDFatFS, SDPath, 1);

	if (res == FR_OK) {
		return SDcard_OK;
	}
	return SDcard_FAIL;
}



SDcard_WRITE_STATUS SD_WriteFile(char* FILENAME, char* DATA, uint32_t DATA_LEN) {
	FRESULT res = f_open(&SDFile, FILENAME, FA_WRITE | FA_OPEN_ALWAYS);

	if (res == FR_OK) {
	  UINT no_of_successful_bytes_written = 0;
	  f_write(&SDFile, DATA, DATA_LEN, &no_of_successful_bytes_written);
	  f_close(&SDFile);

	  if (no_of_successful_bytes_written == DATA_LEN) {
		  return SDcard_WRITE_OK;
	  }
	  else {
		  return SDcard_WRITE_COUNT_ERROR;
	  }
	}
	else {
		f_close(&SDFile);
		return SDcard_WRITE_FAIL;
	}
}


SDcard_READ_STATUS SD_ReadFile(char* FILENAME, char* BUF, uint32_t DATA_LEN) {
  FRESULT res = f_open(&SDFile, FILENAME, FA_READ);

  if (res == FR_OK) {
	  UINT no_of_successful_bytes_read = 0;
	  f_read(&SDFile, BUF, DATA_LEN, &no_of_successful_bytes_read);
	  f_close(&SDFile);

	  if (no_of_successful_bytes_read == DATA_LEN) {
		  return SDcard_READ_OK;
	  }
	  else {
		  return SDcard_READ_COUNT_ERROR;
	  }
  }
  else {
	  return SDcard_READ_FAIL;
  }
}


//FRESULT res = f_mount(&SDFatFS, SDPath, 1);
//
//if (res == FR_OK) {
//	  res = f_open(&SDFile, "test.txt", FA_WRITE | FA_OPEN_ALWAYS);
//	  // Writing a file for testing
//	  if (res == FR_OK) {
//		  UINT no_of_successful_bytes_written = 0;
//		  f_write(&SDFile, msg, sizeof(msg), &no_of_successful_bytes_written);
//		  f_close(&SDFile);
//	  }
//
//
//	  res = f_open(&SDFile, "test.txt", FA_READ);
//	  // Reading a file for testing
//	  if (res == FR_OK) {
//		  UINT no_of_successful_bytes_written = 0;
//		  f_read(&SDFile, buf, sizeof(msg), &no_of_successful_bytes_written);
//		  f_close(&SDFile);
//	  }
//}
