/**
  ******************************************************************************
  * @file    SPI/SPI_FullDuplex_ComPolling/Inc/main.h
  * @author  MCD Application Team
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_nucleo_32.h"
#define MASTER_BOARD

#define pwr_en_Pin GPIO_PIN_3
#define pwr_en_GPIO_Port GPIOA

#define CS_Pin GPIO_PIN_4
#define CS_GPIO_Port GPIOA

#define blank_Pin GPIO_PIN_0
#define blank_GPIO_Port GPIOB

#define user_btn_Pin GPIO_PIN_12
#define user_btn_GPIO_Port GPIOA


/* Definition for TIMx clock resources */
#define TIMx                             TIM2

/* Definition for TIMx clock resources */
#define TIMx_CLK_ENABLE                  __HAL_RCC_TIM2_CLK_ENABLE
#define DMAx_CLK_ENABLE                  __HAL_RCC_DMA1_CLK_ENABLE

/* Definition for TIMx Pins */

/* Definition for TIMx Pins */
#define TIMx_CHANNEL_GPIO_PORT()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define TIMx_GPIO_PORT                 GPIOB
#define TIMx_GPIO_PIN_CHANNEL1         GPIO_PIN_5
#define TIMx_GPIO_AF_TIMx              GPIO_AF5_TIM21

#define TIMx_CHANNEL3_GPIO_CLK_ENABLE    __HAL_RCC_GPIOB_CLK_ENABLE
#define TIMx_GPIO_CHANNEL3_PORT          GPIOB
#define GPIO_PIN_CHANNEL3                GPIO_PIN_7

#define GPIO_AF_TIMx                     GPIO_AF5_TIM2

/* Definition for TIMx's DMA */
#define TIMx_CC3_DMA_REQUEST             DMA_REQUEST_8
#define TIMx_CC3_DMA_INST                DMA1_Channel4

/* Definition for ADCx's NVIC */
#define TIMx_DMA_IRQn                    DMA1_Channel2_3_IRQn
#define TIMx_DMA_IRQHandler              DMA1_Channel2_3_IRQnHandler
#define TIMx_IRQn                      TIM2_IRQn
#define TIMx_IRQHandler                TIM2_IRQHandler

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor SPIx instance used and associated
   resources */
/* Definition for SPIx clock resources */
#define SPIx                             SPI1
#define SPIx_CLK_ENABLE()                __HAL_RCC_SPI1_CLK_ENABLE()
#define SPIx_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPIx_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPIx_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define SPIx_FORCE_RESET()               __HAL_RCC_SPI1_FORCE_RESET()
#define SPIx_RELEASE_RESET()             __HAL_RCC_SPI1_RELEASE_RESET()

/* Definition for SPIx Pins */
#define SPIx_SCK_PIN                     GPIO_PIN_5
#define SPIx_SCK_GPIO_PORT               GPIOA
#define SPIx_SCK_AF                      GPIO_AF0_SPI1
#define SPIx_MISO_PIN                    GPIO_PIN_6
#define SPIx_MISO_GPIO_PORT              GPIOA
#define SPIx_MISO_AF                     GPIO_AF0_SPI1
#define SPIx_MOSI_PIN                    GPIO_PIN_7
#define SPIx_MOSI_GPIO_PORT              GPIOA
#define SPIx_MOSI_AF                     GPIO_AF0_SPI1

/* Size of buffer */
#define BUFFERSIZE                       (COUNTOF(aTxBuffer) - 1)

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Exported functions ------------------------------------------------------- */

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
