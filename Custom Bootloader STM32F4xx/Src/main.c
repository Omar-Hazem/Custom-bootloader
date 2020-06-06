/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "stm32f4xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BL_DEBUG_MSG_EN	//enable this macro to enable debug messages over debug uart

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

#define D_UART &huart3	//debug uart
#define C_UART &huart2	//command uart
#define BL_RX_LEN 200
uint8_t BL_RX_BUFFER[BL_RX_LEN]; //to store all the command bytes sent by the bootloader
/* USER CODE BEGIN PV */
uint8_t supported_commands[8] = {
															 BL_GET_VER,
                               BL_GET_HELP,
                               BL_GET_CID,
                               BL_GET_RDP_STATUS,
                               BL_GO_TO_ADDR,
                               BL_FLASH_ERASE,
                               BL_MEM_WRITE,
															 BL_READ_SECTOR_STATUS} ;
	
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
static void print_msg(char* format,...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

	if(HAL_GPIO_ReadPin(B1_GPIO_Port,B1_Pin) == GPIO_PIN_RESET )
	{
		print_msg("BL_DEBUG_MSG : Button is pressed , going to BL mode...\n");
		bootloader_uart_read_data();
	}
	else
	{
		print_msg("BL_DEBUG_MSG : Button is not pressed , going to user application mode...\n");
		bootloader_jump_to_user_app();
	}
  /* USER CODE END 2 */
//	char somedata[] ="hello from bootloader\n"; 
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		//HAL_UART_Transmit(&huart2,(uint8_t *)somedata,sizeof(somedata),HAL_MAX_DELAY);
//		//HAL_UART_Transmit(&huart3,(uint8_t *)somedata,sizeof(somedata),HAL_MAX_DELAY);
////		uint32_t cuurentTick = HAL_GetTick();
////		print_msg("current tick value = %d\n",cuurentTick);
////		while(HAL_GetTick() <= (cuurentTick+500));
//		
//    /* USER CODE END WHILE */

//    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */

void bootloader_uart_read_data(void)
{
uint8_t rcv_len=0;
	while(1)
	{
		/*The C library function void *memset(void *str, int c, size_t n) copies the character c (an unsigned char) 
		to the first n characters of the string pointed to, by the argument str.*/
		memset(BL_RX_BUFFER,0,BL_RX_LEN);
		// we will read the command coming from the host and decode it
		//we will read the first byte which is the length of the command packet
		HAL_UART_Receive(C_UART,BL_RX_BUFFER,1,HAL_MAX_DELAY);
		rcv_len = BL_RX_BUFFER[0];
		HAL_UART_Receive(C_UART,&BL_RX_BUFFER[1],rcv_len,HAL_MAX_DELAY);
		switch(BL_RX_BUFFER[1])
		{
			case BL_GET_VER:
				BL_handle_getVersion_cmd(BL_RX_BUFFER);
				break;
			case BL_GET_HELP:
				BL_handle_getHelp_cmd(BL_RX_BUFFER);
				break;
			case BL_GET_CID:
				BL_handle_getCID_cmd(BL_RX_BUFFER);
				break;
//			case BL_GET_RDP_STATUS:
//				BL_handle_getRDP_cmd(BL_RX_BUFFER);
//				break;
//			case BL_GO_TO_ADDR:
//				BL_handle_goAddress_cmd(BL_RX_BUFFER);
//				break;
//			case BL_FLASH_ERASE:
//				BL_handle_flashErase_cmd(BL_RX_BUFFER);
//				break;
//			case BL_MEM_WRITE:
//				BL_handle_memWrite_cmd(BL_RX_BUFFER);
//				break;
//			case BL_MEM_READ:
//				BL_handle_memRead_cmd(BL_RX_BUFFER);
//				break;
//			case BL_READ_SECTOR_STATUS:
//				BL_handle_ReadSectorStatus_cmd(BL_RX_BUFFER);
//				break;
			default:
				print_msg("BL_DEBUG_MSG: INVALID COMMAND CODE RECEIVED FROM THE HOST\r\n");
		}
		
		
		
	}
}

void bootloader_jump_to_user_app(void)
{
  void (*function_ptr)(void);//to hold the address of the reset handler of the user app
	print_msg("BL_DEBUG_MSG : bootloader jump to user app \n");
	uint32_t msp_value = *(volatile uint32_t *)FLASH_SECTOR2_BASE_ADDRESS;
	print_msg("BL_DEBUG_MSG : MSP value = %x\n",msp_value);
	__set_MSP(msp_value);
	uint32_t reset_handler_value = *(volatile uint32_t *)(FLASH_SECTOR2_BASE_ADDRESS + 4);
	function_ptr = (void *)reset_handler_value;
	print_msg("BL_DEBUG_MSG : reset handler value = %x\n",reset_handler_value);
	while(1)
	{
	function_ptr(); //jump to reset handler of the user application i.e : bootloader handling control to user app
	for(uint16_t i = 0 ; i<= 1023;i++);
	}
	}


void print_msg(char* format,...)
{
	#ifdef BL_DEBUG_MSG_EN
	char str[80];
	va_list args;
	va_start(args,format);
	vsprintf(str,format,args);
	HAL_UART_Transmit(D_UART,(uint8_t *)str,strlen(str),HAL_MAX_DELAY);
	va_end(args);
	
	#endif
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************** Implementation of bootloader command handle functions*************************/

void BootLoader_UART_Write_Data(uint8_t *pbuffer, uint32_t length)
{
	HAL_UART_Transmit(C_UART,pbuffer,length,HAL_MAX_DELAY);
}

void BL_handle_getVersion_cmd(uint8_t* pbuffer)
{
	uint8_t BL_VER;
	uint8_t command_packet_length = pbuffer[0] + 1; // the command packet length size itself is 1 byte
	uint32_t host_crc = *((uint32_t * )(pbuffer + command_packet_length -4));
	print_msg("BL_DEBUG_MSG : BL_handle_getVersion_cmd running...\n");
	//verifying checksum
	if(!bootloader_verify_CRC(&pbuffer[0],command_packet_length - 4,host_crc)) 
	{	
		print_msg("BL_DEBUG_MSG : CheckSum correct...\n");
		//checksum is correct
		bootloader_send_ack(pbuffer[0],1);//2nd argument is the length to follow and it's decided by the func itself
		BL_VER = get_bootloader_version();
		print_msg("BL_DEBUG_MSG : BootLoader Version : %d %x \n ",BL_VER,BL_VER);
		//sending reply to the host
		HAL_UART_Transmit(C_UART,&BL_VER,1,HAL_MAX_DELAY); //Recheck
		
	}
	else
		{
			print_msg("BL_DEBUG_MSG : CheckSum failed, sending NACK...\n");
			bootloader_send_nack();
		}
}


void BL_handle_getHelp_cmd(uint8_t* pbuffer)
{
	uint32_t command_packet_length = pbuffer[0] + 0x01;
	uint32_t host_crc = *( (uint32_t * ) (pbuffer + command_packet_length -4) );
	print_msg("BL_DEBUG_MSG : BL_handle_getHelp_cmd running...\n");
	if(! bootloader_verify_CRC(pbuffer,(command_packet_length - 4),host_crc)) 
	{	
		print_msg("BL_DEBUG_MSG : CheckSum correct...\n");
		//checksum is correct
		bootloader_send_ack(pbuffer[0],sizeof(supported_commands));//2nd argument is the length to follow and it's decided by the func itself
		//sending reply to the host
		BootLoader_UART_Write_Data(supported_commands,sizeof(supported_commands));
	}
	else
		{
			print_msg("BL_DEBUG_MSG : CheckSum failed, sending NACK...\n");
			bootloader_send_nack();
		}
}


void BL_handle_getCID_cmd(uint8_t* pbuffer)
{
	uint32_t command_packet_length = pbuffer[0] + 0x01;
	uint32_t host_crc = *( (uint32_t * ) (pbuffer + command_packet_length -4) );
	print_msg("BL_DEBUG_MSG : BL_handle_getCID_cmd running...\n");
	uint16_t chip_id ;
	chip_id = (uint16_t)(DBGMCU->IDCODE) & 0xFFFF;
	if(! bootloader_verify_CRC(pbuffer,(command_packet_length - 4),host_crc)) 
	{	
		print_msg("BL_DEBUG_MSG : CheckSum correct...\n");
		//checksum is correct
		bootloader_send_ack(pbuffer[0],2);//2nd argument is the length to follow and it's decided by the func itself
		//sending reply to the host
		BootLoader_UART_Write_Data((uint8_t *)&chip_id,sizeof(chip_id));
	}
	else
		{
			print_msg("BL_DEBUG_MSG : CheckSum failed, sending NACK...\n");
			bootloader_send_nack();
		}
}
//void BL_handle_getRDP_cmd(uint8_t* pbuffer)
//{
//	
//}
//void BL_handle_goAddress_cmd(uint8_t* pbuffer)
//{
//	
//}
//void BL_handle_flashErase_cmd(uint8_t* pbuffer)
//{
//	
//}
//void BL_handle_memWrite_cmd(uint8_t* pbuffer)
//{
//	
//}
//void BL_handle_memRead_cmd(uint8_t* pbuffer)
//{	
//	
//}
//void BL_handle_ReadSectorStatus_cmd(uint8_t* pbuffer)
//{
//	
//}
/* this function sends ACK if the CRC matches with the lenght to follow*/
void bootloader_send_ack(uint8_t command_code,uint8_t follow_length)
{
	uint8_t ack_buf[2] ;
	//we send 2 bytes ACK and length value
	ack_buf[0] = BL_ACK;
	ack_buf[1] =  follow_length;
	HAL_UART_Transmit(C_UART,ack_buf,2,HAL_MAX_DELAY);
}

void bootloader_send_nack(void)
{
	uint8_t nack = BL_NACK;
	HAL_UART_Transmit(C_UART,&nack,1,HAL_MAX_DELAY);
}
//verifies the CRC of the given buffer in pData
//checks the CRC of the pData over the length and compares it w ith the CRC_host
uint8_t bootloader_verify_CRC(uint8_t * pData,uint32_t length,uint32_t CRC_host)
{
	uint32_t CRC_value = 0xff;
	uint32_t i;
	for(i=0;i<length;i++)
	{
		uint32_t data = pData[i];
		CRC_value = HAL_CRC_Accumulate(& hcrc,(uint32_t *)data,sizeof(data));
	}
	if(CRC_value == CRC_host)
	{
		return VERIFY_CRC_SUCCESS;
	}
	else
	{
		return VERIFY_CRC_FAILURE;
	}
}

uint8_t get_bootloader_version(void)
{
	return BL_Version;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
