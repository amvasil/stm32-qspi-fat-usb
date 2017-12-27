/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 * This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * Copyright (c) 2017 STMicroelectronics International N.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "fatfs.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "fat_qspi_drv.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
QSPI_HandleTypeDef hqspi;
DMA_HandleTypeDef hdma_quadspi;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
FATFS QSPIFatFs;  /* File system object for SD card logical drive */
char QSPIPath[4]; /* SD card logical drive path */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_QUADSPI_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//extern void initialise_monitor_handles(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

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
	MX_DMA_Init();
	//MX_FATFS_Init();
	MX_USB_DEVICE_Init();
	MX_QUADSPI_Init();

	/* USER CODE BEGIN 2 */
	//enable semihosting
	//initialise_monitor_handles();

	__enable_irq();

	FRESULT res;


	//##-1- Link the micro SD disk I/O driver ##################################
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);

	uint32_t blink_period = 500;
	FILINFO file_info;
	DIR root_dir;
	FIL file;
	char filename[20];
	memset(filename,0,sizeof(filename));
	//printf("Linking FS\n");
	if(FATFS_LinkDriver(&QSPI_Driver, QSPIPath) == 0)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
		//##-2- Register the file system object to the FatFs module ##############
		//printf("Mounting FS\n");
		if((res=f_mount(&QSPIFatFs, (TCHAR const*)QSPIPath, 1)) != FR_OK)
		{
			//FatFs Initialization Error
			//printf("error %d\n",(uint8_t)res);
			Error_Handler();
		}
		else
		{
			//printf("FAT mounted as %s\n",QSPIPath);
			if(!check_qspi_avaliable())
			{
				//printf("QSPI check failed\n");
				Error_Handler();
			}
			else
			{
				//printf("QSPI check OK\n");
			}
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

			//open directory
			//printf("Opening root dir\n");

			if( (res=f_opendir(&root_dir, (const TCHAR*)"/")) != FR_OK)
			{
				//printf("error %d\n",(uint8_t)res);
				Error_Handler();
			}

			//read all items in directory
			//printf("Read files in root dir\n");
			uint8_t file_found = 0;

			while(1)
			{
				if(f_readdir(&root_dir, &file_info) != FR_OK)
				{
					Error_Handler();
				}
				if(strlen(file_info.fname) == 0)
					break;
				//printf("Found file %s\n",file_info.fname);
				const char *config_name = "CONFIG.TXT";
				if(!strncmp(file_info.fname,config_name, strlen(config_name)))
				{
					file_found = 1;
					//set file path
					filename[0] = '/';
					strncpy(&filename[1], file_info.fname, strlen(file_info.fname));
				}

			}
			if((res=f_closedir(&root_dir)) != FR_OK)
			{
				//printf("Dir close error %d\n",(uint8_t)res);
				Error_Handler();
			}
			//printf("Root dir read OK\n");
			if(!file_found)
			{
				//printf("No files to read\n");
			}
			else
			{
				//open last found file
				//printf("Opening file %s\n",filename);
				char line[100];
				if((res = f_open(&file, filename, FA_READ)) != FR_OK)
				{
					//printf("File open failed, error %d\n",res);
					Error_Handler();
				}
				//read all file
				while (f_gets(line, sizeof(line), &file)) {
					//printf(line);
					int32_t num = atoi(line);
					if(num > 0 && num < 10000)
					{
						//printf("Setting blink period to %d\n", num);
						blink_period = num;
					}
				}
				//close file
				f_close(&file);
				//printf("\n");
				//printf("File closed\n");
			}


		}
	}
	else
	{
		Error_Handler();
	}

	/*##-11- Unlink the RAM disk I/O driver ####################################*/

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_8);
		HAL_Delay(blink_period);
	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 40;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
	PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
	PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
	PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
	PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
	PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* QUADSPI init function */
static void MX_QUADSPI_Init(void)
{

	hqspi.Instance = QUADSPI;
	hqspi.Init.ClockPrescaler = 0;
	hqspi.Init.FifoThreshold = 4;
	hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
	hqspi.Init.FlashSize = 23;
	hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
	hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
	if (HAL_QSPI_Init(&hqspi) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) 
{
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/** Configure pins
     PE2   ------> SAI1_MCLK_A
     PE4   ------> SAI1_FS_A
     PE5   ------> SAI1_SCK_A
     PE6   ------> SAI1_SD_A
     PC3   ------> LCD_VLCD
     PA6   ------> LCD_SEG3
     PA7   ------> LCD_SEG4
     PC4   ------> LCD_SEG22
     PC5   ------> LCD_SEG23
     PB0   ------> LCD_SEG5
     PB1   ------> LCD_SEG6
     PE7   ------> SAI1_SD_B
     PE9   ------> SAI1_FS_B
     PB10   ------> I2C2_SCL
     PB11   ------> I2C2_SDA
     PB12   ------> LCD_SEG12
     PB13   ------> LCD_SEG13
     PB14   ------> LCD_SEG14
     PB15   ------> LCD_SEG15
     PD8   ------> LCD_SEG28
     PD9   ------> LCD_SEG29
     PD10   ------> LCD_SEG30
     PD11   ------> LCD_SEG31
     PD12   ------> LCD_SEG32
     PD13   ------> LCD_SEG33
     PD14   ------> LCD_SEG34
     PD15   ------> LCD_SEG35
     PC6   ------> LCD_SEG24
     PC7   ------> LCD_SEG25
     PC8   ------> LCD_SEG26
     PA8   ------> LCD_COM0
     PA9   ------> LCD_COM1
     PA10   ------> LCD_COM2
     PA15 (JTDI)   ------> LCD_SEG17
     PD1   ------> SPI2_SCK
     PD3   ------> SPI2_MISO
     PD4   ------> SPI2_MOSI
     PD5   ------> USART2_TX
     PD6   ------> USART2_RX
     PB4 (NJTRST)   ------> LCD_SEG8
     PB5   ------> LCD_SEG9
     PB6   ------> I2C1_SCL
     PB7   ------> I2C1_SDA
     PB9   ------> LCD_COM3
 */
static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, AUDIO_RST_Pin|LD_G_Pin|XL_CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD_R_GPIO_Port, LD_R_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(OTG_FS_VBUS_GPIO_Port, OTG_FS_VBUS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : SAI1_MCK_Pin SAI1_FS_Pin SAI1_SCK_Pin SAI1_SD_Pin
                           AUDIO_DIN_Pin */
	GPIO_InitStruct.Pin = SAI1_MCK_Pin|SAI1_FS_Pin|SAI1_SCK_Pin|SAI1_SD_Pin
			|AUDIO_DIN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : AUDIO_RST_Pin */
	GPIO_InitStruct.Pin = AUDIO_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(AUDIO_RST_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : MFX_IRQ_OUT_Pin OTG_FS_OverCurrent_Pin */
	GPIO_InitStruct.Pin = MFX_IRQ_OUT_Pin|OTG_FS_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PC0 MAG_INT_Pin MAG_DRDY_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_0|MAG_INT_Pin|MAG_DRDY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : VLCD_Pin SEG22_Pin SEG1_Pin SEG14_Pin
                           SEG9_Pin SEG13_Pin */
	GPIO_InitStruct.Pin = VLCD_Pin|SEG22_Pin|SEG1_Pin|SEG14_Pin
			|SEG9_Pin|SEG13_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : JOY_CENTER_Pin JOY_LEFT_Pin JOY_RIGHT_Pin JOY_UP_Pin
                           JOY_DOWN_Pin */
	GPIO_InitStruct.Pin = JOY_CENTER_Pin|JOY_LEFT_Pin|JOY_RIGHT_Pin|JOY_UP_Pin
			|JOY_DOWN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : MFX_WAKEUP_Pin */
	GPIO_InitStruct.Pin = MFX_WAKEUP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(MFX_WAKEUP_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SEG23_Pin SEG0_Pin COM0_Pin COM1_Pin
                           COM2_Pin SEG10_Pin */
	GPIO_InitStruct.Pin = SEG23_Pin|SEG0_Pin|COM0_Pin|COM1_Pin
			|COM2_Pin|SEG10_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : SEG21_Pin SEG2_Pin SEG20_Pin SEG3_Pin
                           SEG19_Pin SEG4_Pin SEG11_Pin SEG12_Pin 
                           COM3_Pin */
	GPIO_InitStruct.Pin = SEG21_Pin|SEG2_Pin|SEG20_Pin|SEG3_Pin
			|SEG19_Pin|SEG4_Pin|SEG11_Pin|SEG12_Pin
			|COM3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : LD_R_Pin */
	GPIO_InitStruct.Pin = LD_R_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(LD_R_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD_G_Pin */
	GPIO_InitStruct.Pin = LD_G_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(LD_G_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : AUDIO_CLK_Pin */
	GPIO_InitStruct.Pin = AUDIO_CLK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
	HAL_GPIO_Init(AUDIO_CLK_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : MFX_I2C_SLC_Pin MFX_I2C_SDA_Pin */
	GPIO_InitStruct.Pin = MFX_I2C_SLC_Pin|MFX_I2C_SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : SEG18_Pin SEG5_Pin SEG17_Pin SEG6_Pin
                           SEG16_Pin SEG7_Pin SEG15_Pin SEG8_Pin */
	GPIO_InitStruct.Pin = SEG18_Pin|SEG5_Pin|SEG17_Pin|SEG6_Pin
			|SEG16_Pin|SEG7_Pin|SEG15_Pin|SEG8_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : OTG_FS_PowerSwitchOn_Pin OTG_FS_VBUS_Pin */
	GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin|OTG_FS_VBUS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : EXT_RST_Pin GYRO_INT1_Pin */
	GPIO_InitStruct.Pin = EXT_RST_Pin|GYRO_INT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : MEMS_SCK_Pin MEMS_MISO_Pin MEMS_MOSI_Pin */
	GPIO_InitStruct.Pin = MEMS_SCK_Pin|MEMS_MISO_Pin|MEMS_MOSI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
	GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : GYRO_CS_Pin */
	GPIO_InitStruct.Pin = GYRO_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GYRO_CS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : I2C1_SCL_Pin I2C1_SDA_Pin */
	GPIO_InitStruct.Pin = I2C1_SCL_Pin|I2C1_SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : GYRO_INT2_Pin */
	GPIO_InitStruct.Pin = GYRO_INT2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GYRO_INT2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : XL_CS_Pin */
	GPIO_InitStruct.Pin = XL_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(XL_CS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : XL_INT_Pin */
	GPIO_InitStruct.Pin = XL_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(XL_INT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */



/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	__disable_irq();
	/* User can add his own implementation to report the HAL error return state */
	while(1)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
		for(volatile long int i=0;i<100000;i++);
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
