/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define SAI1_MCK_Pin GPIO_PIN_2
#define SAI1_MCK_GPIO_Port GPIOE
#define AUDIO_RST_Pin GPIO_PIN_3
#define AUDIO_RST_GPIO_Port GPIOE
#define SAI1_FS_Pin GPIO_PIN_4
#define SAI1_FS_GPIO_Port GPIOE
#define SAI1_SCK_Pin GPIO_PIN_5
#define SAI1_SCK_GPIO_Port GPIOE
#define SAI1_SD_Pin GPIO_PIN_6
#define SAI1_SD_GPIO_Port GPIOE
#define MFX_IRQ_OUT_Pin GPIO_PIN_13
#define MFX_IRQ_OUT_GPIO_Port GPIOC
#define MAG_INT_Pin GPIO_PIN_1
#define MAG_INT_GPIO_Port GPIOC
#define MAG_DRDY_Pin GPIO_PIN_2
#define MAG_DRDY_GPIO_Port GPIOC
#define VLCD_Pin GPIO_PIN_3
#define VLCD_GPIO_Port GPIOC
#define JOY_CENTER_Pin GPIO_PIN_0
#define JOY_CENTER_GPIO_Port GPIOA
#define JOY_LEFT_Pin GPIO_PIN_1
#define JOY_LEFT_GPIO_Port GPIOA
#define JOY_RIGHT_Pin GPIO_PIN_2
#define JOY_RIGHT_GPIO_Port GPIOA
#define JOY_UP_Pin GPIO_PIN_3
#define JOY_UP_GPIO_Port GPIOA
#define MFX_WAKEUP_Pin GPIO_PIN_4
#define MFX_WAKEUP_GPIO_Port GPIOA
#define JOY_DOWN_Pin GPIO_PIN_5
#define JOY_DOWN_GPIO_Port GPIOA
#define SEG23_Pin GPIO_PIN_6
#define SEG23_GPIO_Port GPIOA
#define SEG0_Pin GPIO_PIN_7
#define SEG0_GPIO_Port GPIOA
#define SEG22_Pin GPIO_PIN_4
#define SEG22_GPIO_Port GPIOC
#define SEG1_Pin GPIO_PIN_5
#define SEG1_GPIO_Port GPIOC
#define SEG21_Pin GPIO_PIN_0
#define SEG21_GPIO_Port GPIOB
#define SEG2_Pin GPIO_PIN_1
#define SEG2_GPIO_Port GPIOB
#define LD_R_Pin GPIO_PIN_2
#define LD_R_GPIO_Port GPIOB
#define AUDIO_DIN_Pin GPIO_PIN_7
#define AUDIO_DIN_GPIO_Port GPIOE
#define LD_G_Pin GPIO_PIN_8
#define LD_G_GPIO_Port GPIOE
#define AUDIO_CLK_Pin GPIO_PIN_9
#define AUDIO_CLK_GPIO_Port GPIOE
#define QSPI_CLK_Pin GPIO_PIN_10
#define QSPI_CLK_GPIO_Port GPIOE
#define QSPI_CS_Pin GPIO_PIN_11
#define QSPI_CS_GPIO_Port GPIOE
#define QSPI_D0_Pin GPIO_PIN_12
#define QSPI_D0_GPIO_Port GPIOE
#define QSPI_D1_Pin GPIO_PIN_13
#define QSPI_D1_GPIO_Port GPIOE
#define QSPI_D2_Pin GPIO_PIN_14
#define QSPI_D2_GPIO_Port GPIOE
#define QSPI_D3_Pin GPIO_PIN_15
#define QSPI_D3_GPIO_Port GPIOE
#define MFX_I2C_SLC_Pin GPIO_PIN_10
#define MFX_I2C_SLC_GPIO_Port GPIOB
#define MFX_I2C_SDA_Pin GPIO_PIN_11
#define MFX_I2C_SDA_GPIO_Port GPIOB
#define SEG20_Pin GPIO_PIN_12
#define SEG20_GPIO_Port GPIOB
#define SEG3_Pin GPIO_PIN_13
#define SEG3_GPIO_Port GPIOB
#define SEG19_Pin GPIO_PIN_14
#define SEG19_GPIO_Port GPIOB
#define SEG4_Pin GPIO_PIN_15
#define SEG4_GPIO_Port GPIOB
#define SEG18_Pin GPIO_PIN_8
#define SEG18_GPIO_Port GPIOD
#define SEG5_Pin GPIO_PIN_9
#define SEG5_GPIO_Port GPIOD
#define SEG17_Pin GPIO_PIN_10
#define SEG17_GPIO_Port GPIOD
#define SEG6_Pin GPIO_PIN_11
#define SEG6_GPIO_Port GPIOD
#define SEG16_Pin GPIO_PIN_12
#define SEG16_GPIO_Port GPIOD
#define SEG7_Pin GPIO_PIN_13
#define SEG7_GPIO_Port GPIOD
#define SEG15_Pin GPIO_PIN_14
#define SEG15_GPIO_Port GPIOD
#define SEG8_Pin GPIO_PIN_15
#define SEG8_GPIO_Port GPIOD
#define SEG14_Pin GPIO_PIN_6
#define SEG14_GPIO_Port GPIOC
#define SEG9_Pin GPIO_PIN_7
#define SEG9_GPIO_Port GPIOC
#define SEG13_Pin GPIO_PIN_8
#define SEG13_GPIO_Port GPIOC
#define OTG_FS_PowerSwitchOn_Pin GPIO_PIN_9
#define OTG_FS_PowerSwitchOn_GPIO_Port GPIOC
#define COM0_Pin GPIO_PIN_8
#define COM0_GPIO_Port GPIOA
#define COM1_Pin GPIO_PIN_9
#define COM1_GPIO_Port GPIOA
#define COM2_Pin GPIO_PIN_10
#define COM2_GPIO_Port GPIOA
#define OTG_FS_DM_Pin GPIO_PIN_11
#define OTG_FS_DM_GPIO_Port GPIOA
#define OTG_FS_DP_Pin GPIO_PIN_12
#define OTG_FS_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SEG10_Pin GPIO_PIN_15
#define SEG10_GPIO_Port GPIOA
#define OTG_FS_OverCurrent_Pin GPIO_PIN_10
#define OTG_FS_OverCurrent_GPIO_Port GPIOC
#define OTG_FS_VBUS_Pin GPIO_PIN_11
#define OTG_FS_VBUS_GPIO_Port GPIOC
#define EXT_RST_Pin GPIO_PIN_0
#define EXT_RST_GPIO_Port GPIOD
#define MEMS_SCK_Pin GPIO_PIN_1
#define MEMS_SCK_GPIO_Port GPIOD
#define GYRO_INT1_Pin GPIO_PIN_2
#define GYRO_INT1_GPIO_Port GPIOD
#define MEMS_MISO_Pin GPIO_PIN_3
#define MEMS_MISO_GPIO_Port GPIOD
#define MEMS_MOSI_Pin GPIO_PIN_4
#define MEMS_MOSI_GPIO_Port GPIOD
#define USART_TX_Pin GPIO_PIN_5
#define USART_TX_GPIO_Port GPIOD
#define USART_RX_Pin GPIO_PIN_6
#define USART_RX_GPIO_Port GPIOD
#define GYRO_CS_Pin GPIO_PIN_7
#define GYRO_CS_GPIO_Port GPIOD
#define SEG11_Pin GPIO_PIN_4
#define SEG11_GPIO_Port GPIOB
#define SEG12_Pin GPIO_PIN_5
#define SEG12_GPIO_Port GPIOB
#define I2C1_SCL_Pin GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB
#define GYRO_INT2_Pin GPIO_PIN_8
#define GYRO_INT2_GPIO_Port GPIOB
#define COM3_Pin GPIO_PIN_9
#define COM3_GPIO_Port GPIOB
#define XL_CS_Pin GPIO_PIN_0
#define XL_CS_GPIO_Port GPIOE
#define XL_INT_Pin GPIO_PIN_1
#define XL_INT_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
