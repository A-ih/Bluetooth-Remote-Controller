/*
 * uart_receiver.h
 *
 *  Created on: May 5, 2025
 *      Author: chowiphim
 */

#ifndef INC_UART_RECEIVER_H_
#define INC_UART_RECEIVER_H_


#define PHOTO_WIDTH  320
#define PHOTO_HEIGHT 240
#define PHOTO_SIZE   (PHOTO_WIDTH * PHOTO_HEIGHT)

uint8_t photo_buffer[PHOTO_SIZE];

// receive normal data
HAL_StatusTypeDef receive_normal_data(UART_HandleTypeDef *huart, int8_t *cal_x, int8_t *cal_y, uint8_t *button, uint8_t *joybut)
{
    uint8_t rx_buf[4];
    if (HAL_UART_Receive(huart, rx_buf, 4, 1000) == HAL_OK)
    {
    	*button   = (int8_t)rx_buf[0];
        *cal_x  = (int8_t)rx_buf[1];
        *cal_y  = rx_buf[2];
        *joybut  = rx_buf[3];
        // rx_buf[4] is stop bit (0x5A)
        return HAL_OK;
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef receive_photo(UART_HandleTypeDef *huart)
{
    uint8_t header[2];
    uint8_t footer[2];

    // Receive header
    if (HAL_UART_Receive(huart, header, 2, 1000) != HAL_OK)
        return HAL_ERROR;
    if (header[0] != 0xB5 || header[1] != 0xAA)
        return HAL_ERROR; // Invalid header

    // Receive image data in two chunks
    if (HAL_UART_Receive(huart, photo_buffer, 65535, 10000) != HAL_OK)
        return HAL_ERROR;
    if (HAL_UART_Receive(huart, photo_buffer + 65535, PHOTO_SIZE - 65535, 10000) != HAL_OK)
        return HAL_ERROR;

    // Receive footer
    if (HAL_UART_Receive(huart, footer, 2, 1000) != HAL_OK)
        return HAL_ERROR;
    if (footer[0] != 0xAA || footer[1] != 0xB5)
        return HAL_ERROR; // Invalid footer

    return HAL_OK;
}


//
//HAL_StatusTypeDef receive_photo(UART_HandleTypeDef *huart)
//{
//    uint8_t header[2];
//    uint8_t footer[2];
//
//    // Receive header
//    if (HAL_UART_Receive(huart, header, 2, 1000) != HAL_OK)
//        return HAL_ERROR;
//    if (header[0] != 0xB5 || header[1] != 0xAA)
//        return HAL_ERROR; // Invalid header
//
//    // Receive image data in two chunks
//    if (HAL_UART_Receive(huart, photo_buffer, 65535, 10000) != HAL_OK)
//        return HAL_ERROR;
//    if (HAL_UART_Receive(huart, photo_buffer + 65535, PHOTO_SIZE - 65535, 10000) != HAL_OK)
//        return HAL_ERROR;
//
//    // Receive footer
//    if (HAL_UART_Receive(huart, footer, 2, 1000) != HAL_OK)
//        return HAL_ERROR;
//    if (footer[0] != 0xAA || footer[1] != 0xB5)
//        return HAL_ERROR; // Invalid footer
//
//    return HAL_OK;
//}




#endif /* INC_UART_RECEIVER_H_ */
