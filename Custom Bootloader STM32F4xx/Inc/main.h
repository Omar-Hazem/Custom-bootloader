/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define BL_ACK 		0xA5
#define BL_NACK 		0x7F
#define VERIFY_CRC_SUCCESS	 0u
#define VERIFY_CRC_FAILURE	 1u
#define BL_Version 0x50

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void bootloader_uart_read_data(void);
void bootloader_jump_to_user_app(void);

//Bootloader Handle Commands Function Prototype
void BootLoader_UART_Write_Data(uint8_t *pbuffer, uint32_t length);
void BL_handle_getVersion_cmd(uint8_t* BL_RX_BUFFER);
void BL_handle_getHelp_cmd(uint8_t* BL_RX_BUFFER);
void BL_handle_getCID_cmd(uint8_t* BL_RX_BUFFER);
//void BL_handle_getRDP_cmd(uint8_t* BL_RX_BUFFER);
//void BL_handle_goAddress_cmd(uint8_t* BL_RX_BUFFER);
//void BL_handle_flashErase_cmd(uint8_t* BL_RX_BUFFER);
//void BL_handle_memWrite_cmd(uint8_t* BL_RX_BUFFER);
//void BL_handle_memRead_cmd(uint8_t* BL_RX_BUFFER);
//void BL_handle_ReadSectorStatus_cmd(uint8_t* BL_RX_BUFFER);
void bootloader_send_ack(uint8_t command_code,uint8_t follow_length);
void bootloader_send_nack(void);
uint8_t bootloader_verify_CRC(uint8_t*pData,uint32_t length,uint32_t CRC_host);
uint8_t get_bootloader_version(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define FLASH_SECTOR2_BASE_ADDRESS 0x08008000U

//Bootloader Commands
#define BL_GET_VER							0x51 // BootLoader Get Version of The Micro_controller
#define BL_GET_HELP							0x52 // Bootloader Get The Supported Commands
#define BL_GET_CID							0x53 // Bootloader Get chip ID
#define BL_GET_RDP_STATUS				0x54 // Bootloader Reed Flash Protection Status
#define BL_GO_TO_ADDR						0x55 // Bootloader Go To Specific address
#define BL_FLASH_ERASE					0x56 // Bootloader mass Erase or Sector Erase
#define BL_MEM_WRITE						0x57 // Bootloader memory Write
#define BL_EN_R_W_PROTECT				0x58 // BL Enable R/W Protection on different Sectors on Flash
#define BL_MEM_READ							0x59 // BL Read Data from Different Memories of MC
#define BL_READ_SECTOR_STATUS		0x5A // BL Read all the sector protection status
#define BL_OTP_READ							0x5B // BL Read the OTP Content 
#define BL_DIS_R_W_PROTECT			0x5C // BL Disable R/W Protection on different Sectors on Flash
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
