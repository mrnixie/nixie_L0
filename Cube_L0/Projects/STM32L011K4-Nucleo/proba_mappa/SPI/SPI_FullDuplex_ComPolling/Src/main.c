/**
  ******************************************************************************
  * @file    SPI/SPI_FullDuplex_ComPolling/Src/main.c
  * @author  MCD Application Team
  * @brief   This sample code shows how to use STM32L0xx SPI HAL API to transmit
  *          and receive a data buffer with a communication process based on
  *          Polling transfer.
  *          The communication is done using 2 Boards.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* SPI handler declaration */
SPI_HandleTypeDef SpiHandle;
TIM_HandleTypeDef  TimHandle;
TIM_HandleTypeDef  TimHandle21;
TIM_OC_InitTypeDef sConfig;
TIM_IC_InitTypeDef sICConfig;

uint32_t               uwIC2Value1 = 0;
uint32_t               uwIC2Value2 = 0;
uint32_t               uwDiffCapture = 0;

/* Capture index */
uint16_t               uhCaptureIndex = 0;

/* Frequency Value */
uint32_t               uwFrequency = 0;


uint32_t nixie_bytes[] = {
		0,//0b00010001000100010001000100010010,
		0,//0b00010001000100010001000100010010,
};

/* Buffer used for reception */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Error_Handler(void);

void loading(uint8_t loading_var){
	nixie_bytes[0] = 0;
	nixie_bytes[1] = 0;
	  switch(loading_var % 4){
		  case 0: nixie_bytes[0] |= (1 << 22); break;
		  case 1: nixie_bytes[0] |= (1 << 14); break;
		  case 2: nixie_bytes[0] |= (1 << 10); break;
		  case 3: nixie_bytes[0] |= (1 << 0); break;
		  //default: loading_var = 0; break;
	  }
}
/* Private functions ---------------------------------------------------------*/
void nixie2(uint8_t hour, uint8_t min, uint8_t sec){
	nixie_bytes[1] = 0;
	nixie_bytes[0] = 0;

	 if (hour < 100){
		switch(hour / 10){
			case 7: nixie_bytes[1] |= (1 << 22); break;
			case 8: nixie_bytes[1] |= (1 << 23); break;
			case 9: nixie_bytes[1] |= (1 << 24); break;
			case 0: nixie_bytes[1] |= (1 << 25); break;
			case 6: nixie_bytes[1] |= (1 << 26); break;
			case 5: nixie_bytes[1] |= (1 << 27); break;
			case 4: nixie_bytes[1] |= (1 << 28); break;
			case 3: nixie_bytes[1] |= (1 << 29); break;
			case 2: nixie_bytes[1] |= (1 << 30); break;
			case 1: nixie_bytes[1] |= (1 << 31); break;
		}

		switch(hour % 10){
			case 6: nixie_bytes[1] |= (1 << 12); break;
			case 7: nixie_bytes[1] |= (1 << 13); break;
			case 8: nixie_bytes[1] |= (1 << 14); break;
			case 9: nixie_bytes[1] |= (1 << 15); break;
			case 0: nixie_bytes[1] |= (1 << 16); break;
			case 5: nixie_bytes[1] |= (1 << 17); break;
			case 4: nixie_bytes[1] |= (1 << 18); break;
			case 3: nixie_bytes[1] |= (1 << 19); break;
			case 2: nixie_bytes[1] |= (1 << 20); break;
			case 1: nixie_bytes[1] |= (1 << 21); break;
		}
	 }

	 if (min < 100){
		switch(min / 10){
			case 0: nixie_bytes[1] |= (1 << 2); break;
			case 9: nixie_bytes[1] |= (1 << 3); break;
			case 8: nixie_bytes[1] |= (1 << 4); break;
			case 7: nixie_bytes[1] |= (1 << 5); break;
			case 6: nixie_bytes[1] |= (1 << 6); break;
			case 1: nixie_bytes[1] |= (1 << 7); break;
			case 5: nixie_bytes[1] |= (1 << 8); break;
			case 2: nixie_bytes[1] |= (1 << 9); break;
			case 3: nixie_bytes[1] |= (1 << 10); break;
			case 4: nixie_bytes[1] |= (1 << 11); break;
		}

		switch(min % 10){
			case 3: nixie_bytes[1] |= (1 << 0); break;
			case 4: nixie_bytes[1] |= (1 << 1); break;

			case 9: nixie_bytes[0] |= (1 << 24); break;
			case 0: nixie_bytes[0] |= (1 << 25); break;
			case 8: nixie_bytes[0] |= (1 << 26); break;
			case 7: nixie_bytes[0] |= (1 << 27); break;
			case 6: nixie_bytes[0] |= (1 << 28); break;
			case 5: nixie_bytes[0] |= (1 << 29); break;
			case 2: nixie_bytes[0] |= (1 << 30); break;
			case 1: nixie_bytes[0] |= (1 << 31); break;
		}
	 }


	if(sec <100){
		switch(sec / 10){
			case 5: nixie_bytes[0] |= (1 << 12); break;
			case 3: nixie_bytes[0] |= (1 << 13); break;
			//jobb N5: nixie_bytes[0] |= (1 << 14); break;
			case 6: nixie_bytes[0] |= (1 << 15); break;
			case 4: nixie_bytes[0] |= (1 << 16); break;
			case 0: nixie_bytes[0] |= (1 << 17); break;
			case 9: nixie_bytes[0] |= (1 << 18); break;
			case 1: nixie_bytes[0] |= (1 << 19); break;
			case 7: nixie_bytes[0] |= (1 << 20); break;
			case 2: nixie_bytes[0] |= (1 << 21); break;
			//bal pont N5: nixie_bytes[0] |= (1 << 22); break;
			case 8: nixie_bytes[0] |= (1 << 23); break;
		}
		switch(sec % 10){
			//jobb N6: nixie_bytes[0] |= (1 << 0); break;
			case 5: nixie_bytes[0] |= (1 << 1); break;
			case 6: nixie_bytes[0] |= (1 << 2); break;
			case 3: nixie_bytes[0] |= (1 << 3); break;
			case 4: nixie_bytes[0] |= (1 << 4); break;
			case 0: nixie_bytes[0] |= (1 << 5); break;
			case 8: nixie_bytes[0] |= (1 << 6); break;
			case 1: nixie_bytes[0] |= (1 << 7); break;
			case 9: nixie_bytes[0] |= (1 << 8); break;
			case 2: nixie_bytes[0] |= (1 << 9); break;
			//bal pont N6: nixie_bytes[0] |= (1 << 10); break;
			case 7: nixie_bytes[0] |= (1 << 11); break;
		}
	}else{
		loading(sec);
	}

}
void nixie(uint32_t num){
	nixie_bytes[1] = 0;
	nixie_bytes[0] = 0;
//256000
		switch(num / 100000){
			case 7: nixie_bytes[1] |= (1 << 22); num -= 700000;break;
			case 8: nixie_bytes[1] |= (1 << 23); num -= 800000;break;
			case 9: nixie_bytes[1] |= (1 << 24); num -= 900000;break;
			case 0: nixie_bytes[1] |= (1 << 25); break;
			case 6: nixie_bytes[1] |= (1 << 26); num -= 600000;break;
			case 5: nixie_bytes[1] |= (1 << 27); num -= 500000;break;
			case 4: nixie_bytes[1] |= (1 << 28); num -= 400000;break;
			case 3: nixie_bytes[1] |= (1 << 29); num -= 300000;break;
			case 2: nixie_bytes[1] |= (1 << 30); num -= 200000;break;
			case 1: nixie_bytes[1] |= (1 << 31); num -= 100000;break;
			default:  break;
		}

		switch(num / 10000){
			case 6: nixie_bytes[1] |= (1 << 12); num -= 6000-0;break;
			case 7: nixie_bytes[1] |= (1 << 13); num -= 70000;break;
			case 8: nixie_bytes[1] |= (1 << 14); num -= 80000;break;
			case 9: nixie_bytes[1] |= (1 << 15); num -= 90000; break;
			case 0: nixie_bytes[1] |= (1 << 16); break;
			case 5: nixie_bytes[1] |= (1 << 17); num -= 50000;break;
			case 4: nixie_bytes[1] |= (1 << 18); num -= 40000;break;
			case 3: nixie_bytes[1] |= (1 << 19); num -= 30000;break;
			case 2: nixie_bytes[1] |= (1 << 20); num -= 20000;break;
			case 1: nixie_bytes[1] |= (1 << 21); num -= 10000;break;
			default:  break;
		}

		switch(num / 1000){
			case 0: nixie_bytes[1] |= (1 << 2); break;
			case 9: nixie_bytes[1] |= (1 << 3); num -= 9000;break;
			case 8: nixie_bytes[1] |= (1 << 4); num -= 8000;break;
			case 7: nixie_bytes[1] |= (1 << 5); num -= 7000;break;
			case 6: nixie_bytes[1] |= (1 << 6); num -= 6000;break;
			case 1: nixie_bytes[1] |= (1 << 7); num -= 1000;break;
			case 5: nixie_bytes[1] |= (1 << 8); num -= 5000;break;
			case 2: nixie_bytes[1] |= (1 << 9); num -= 2000;break;
			case 3: nixie_bytes[1] |= (1 << 10); num -= 3000;break;
			case 4: nixie_bytes[1] |= (1 << 11); num -= 4000;break;
			default:   break;
		}

		switch(num / 100){
			case 3: nixie_bytes[1] |= (1 << 0); num -= 300;break;
			case 4: nixie_bytes[1] |= (1 << 1); num -= 400;break;

			case 9: nixie_bytes[0] |= (1 << 24); num -= 900;break;
			case 0: nixie_bytes[0] |= (1 << 25); break;
			case 8: nixie_bytes[0] |= (1 << 26); num -= 800;break;
			case 7: nixie_bytes[0] |= (1 << 27); num -= 700;break;
			case 6: nixie_bytes[0] |= (1 << 28); num -= 600;break;
			case 5: nixie_bytes[0] |= (1 << 29); num -= 500;break;
			case 2: nixie_bytes[0] |= (1 << 30); num -= 200;break;
			case 1: nixie_bytes[0] |= (1 << 31); num -= 100;break;
			default:    break;
		}

		switch(num / 10){
			case 5: nixie_bytes[0] |= (1 << 12); break;
			case 3: nixie_bytes[0] |= (1 << 13); break;
			//jobb N5: nixie_bytes[0] |= (1 << 14); break;
			case 6: nixie_bytes[0] |= (1 << 15); break;
			case 4: nixie_bytes[0] |= (1 << 16); break;
			case 0: nixie_bytes[0] |= (1 << 17); break;
			case 9: nixie_bytes[0] |= (1 << 18); break;
			case 1: nixie_bytes[0] |= (1 << 19); break;
			case 7: nixie_bytes[0] |= (1 << 20); break;
			case 2: nixie_bytes[0] |= (1 << 21); break;
			//bal pont N5: nixie_bytes[0] |= (1 << 22); break;
			case 8: nixie_bytes[0] |= (1 << 23); break;
			default:     break;
		}
		switch(num % 10){
			//jobb N6: nixie_bytes[0] |= (1 << 0); break;
			case 5: nixie_bytes[0] |= (1 << 1); break;
			case 6: nixie_bytes[0] |= (1 << 2); break;
			case 3: nixie_bytes[0] |= (1 << 3); break;
			case 4: nixie_bytes[0] |= (1 << 4); break;
			case 0: nixie_bytes[0] |= (1 << 5); break;
			case 8: nixie_bytes[0] |= (1 << 6); break;
			case 1: nixie_bytes[0] |= (1 << 7); break;
			case 9: nixie_bytes[0] |= (1 << 8); break;
			case 2: nixie_bytes[0] |= (1 << 9); break;
			//bal pont N6: nixie_bytes[0] |= (1 << 10); break;
			case 7: nixie_bytes[0] |= (1 << 11); break;
			default:   loading(num);  break;
		}


}
void cs_1(){
	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin,GPIO_PIN_SET);//CS/LE pi low
	  HAL_Delay(1);
	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin,GPIO_PIN_RESET);//CS/LE pi high
}
void cs_2(){
	  HAL_GPIO_WritePin(blank_GPIO_Port, blank_Pin,GPIO_PIN_SET);//CS/LE pi low
	  HAL_Delay(1);
	  HAL_GPIO_WritePin(blank_GPIO_Port, blank_Pin,GPIO_PIN_RESET);//CS/LE pi low
}

void InitTIM2(void)
{
	TimHandle.Instance = TIM2;
	TimHandle.Init.Period            = 18;
	TimHandle.Init.Prescaler         = 0;
	TimHandle.Init.ClockDivision     = 0;
	TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK)
	{

		Error_Handler();
	}

    sConfig.OCMode       = TIM_OCMODE_PWM1;
	sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
	sConfig.Pulse        = 0;
	sConfig.OCFastMode = TIM_OCFAST_ENABLE;
	if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_4) != HAL_OK)
	{
	  Error_Handler();
	}
}

/* Capture Compare buffer 6 = bit0, 12 = bit1*/
	const uint16_t rgb_buf1[145] = {12, 6, 6, 6, 6, 6, 6, 6,//GREEN//144
							12, 6, 6, 6, 6, 6, 6, 6,//RED
							12, 6, 6, 6, 6, 6, 6, 6,//BLUE

							12, 6, 6, 6, 6, 6, 6, 6,//GREEN
							12, 6, 6, 6, 6, 6, 6, 6,//RED
							12, 6, 6, 6, 6, 6, 6, 6,//BLUE

							12, 6, 6, 6, 6, 6, 6, 6,//GREEN
							12, 6, 6, 6, 6, 6, 6, 6,//RED
							12, 6, 6, 6, 6, 6, 6, 6,//BLUE

							12, 6, 6, 6, 6, 6, 6, 6,//GREEN
							12, 6, 6, 6, 6, 6, 6, 6,//RED
							12, 6, 6, 6, 6, 6, 6, 6,//BLUE

							12, 6, 6, 6, 6, 6, 6, 6,//GREEN
							12, 6, 6, 6, 6, 6, 6, 6,//RED
							12, 6, 6, 6, 6, 6, 6, 6,//BLUE

							12, 6, 6, 6, 6, 6, 6, 6,//GREEN
							12, 6, 6, 6, 6, 6, 6, 6,//RED
							12, 6, 6, 6, 6, 6, 6, 6,//BLUE

	};

void rgb_sett(uint8_t red, uint8_t blue, uint8_t green){



	//HAL_TIM_PWM_Stop_DMA(&TimHandle, TIM_CHANNEL_4);
	//InitTIM2();
	HAL_TIM_PWM_Start_DMA(&TimHandle, TIM_CHANNEL_4, rgb_buf1, 24);

	/*
	DMA_SetCurrDataCounter(DMA1_Channel4, 144);     // load number of bytes to be transferred

	    // PAP: Clear the timer's counter and set the compare value to 0. This
	    // sets the output low on start and gives us a full cycle to set up DMA.
	    TIM_SetCounter(TIM2, 0);
	    TIM_SetCompare4(TIM2, 0);
	    TIM_Cmd(TIM2, ENABLE);                         // enable Timer 3

	    // PAP: Start DMA transfer after starting the timer. This prevents the
	    // DMA/PWM from dropping the first bit.
	    DMA_Cmd(DMA1_Channel4, ENABLE);             // enable DMA channel 6
	    while(!DMA_GetFlagStatus( DMA_FLAG_TC4));     // wait until transfer complete
	    TIM_Cmd(TIM2, DISABLE);                     // disable Timer 3
	    DMA_Cmd(DMA1_Channel4, DISABLE);             // disable DMA channel 6
	    DMA_ClearFlag( DMA_FLAG_TC4);                 // clear DMA1 Channel 6 transfer complete flag
	    */
	//}
		//while(HAL_TIM_PWM_Start_DMA(&TimHandle, TIM_CHANNEL_4, rgb_buf1, 24) != HAL_OK)
	HAL_Delay(1);
	HAL_TIM_PWM_Stop_DMA(&TimHandle, TIM_CHANNEL_4);
}

uint32_t ir_signal;
uint8_t belepesek = 0;
uint32_t tmp_arr[20];
int main(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  HAL_Init();
  SystemClock_Config();

  BSP_LED_Init(LED3);

  InitTIM2();

  /* Set TIMx instance */
    TimHandle21.Instance = TIM21;
    TimHandle21.Init.Period            = 0xFFFF;
    TimHandle21.Init.Prescaler         = 100;
    TimHandle21.Init.ClockDivision     = 256;
    TimHandle21.Init.CounterMode       = TIM_COUNTERMODE_UP;
    if(HAL_TIM_IC_Init(&TimHandle21) != HAL_OK)
    {
      /* Initialization Error */
      Error_Handler();
    }

    /* Configure the Input Capture of channel 1 */
    sICConfig.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
    sICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sICConfig.ICPrescaler = TIM_ICPSC_DIV1;
    sICConfig.ICFilter    = 0;
    if(HAL_TIM_IC_ConfigChannel(&TimHandle21, &sICConfig, TIM_CHANNEL_1) != HAL_OK)
    {
      /* Configuration Error */
      Error_Handler();
    }

    /*##-3- Start the Input Capture in interrupt mode ##########################*/
    if(HAL_TIM_IC_Start_IT(&TimHandle21, TIM_CHANNEL_1) != HAL_OK)
    {
      /* Starting Error */
      Error_Handler();
    }

  /*##-1- Configure the SPI peripheral #######################################*/
  /* Set the SPI parameters */
  SpiHandle.Instance               = SPIx;
  SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
  SpiHandle.Init.CLKPhase          = SPI_PHASE_2EDGE;
  SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
  SpiHandle.Init.DataSize          = SPI_DATASIZE_16BIT;
  SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_LSB;
  SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
  SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  SpiHandle.Init.CRCPolynomial     = 7;
  SpiHandle.Init.NSS               = SPI_NSS_SOFT;

  SpiHandle.Init.Mode = SPI_MODE_MASTER;
  if(HAL_SPI_Init(&SpiHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Configure PA.12 (Arduino D2) button */
  GPIO_InitStruct.Pin = user_btn_Pin;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = pwr_en_Pin;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  //BLANK
  GPIO_InitStruct.Pin = blank_Pin;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Enable GPIOA clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  //HAL_GPIO_WritePin(pwr_en_GPIO_Port, pwr_en_Pin,GPIO_PIN_SET);//DO HV for peropherals

  uint8_t pwr = 1;
  uint32_t i = 0;
  uint8_t ii = 0;
  //nixie_bytes[0] = 1;

	while (1)
	{
		rgb_sett(12,0,0);
		HAL_Delay(1000);
/*		  BSP_LED_Toggle(LED_GREEN);

		if(ir_signal > 0){
			nixie(ir_signal);
			ir_signal = 0;
		}else loading(i++);

	  HAL_SPI_Transmit(&SpiHandle, (uint8_t*) &nixie_bytes[1], 2, 1000);
	  cs_1();
	  HAL_SPI_Transmit(&SpiHandle, (uint8_t*) &nixie_bytes[0], 2, 1000);
	  cs_2();
/*
	  if(belepesek == 12){
		  belepesek = 0;
			if((tmp_arr[i] - tmp_arr[i - 1]) > 30){
				ir_signal |= (1 << i);
				BSP_LED_Toggle(LED3);
			}
	  }
*/
	  if((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == GPIO_PIN_RESET))
		{
		  ir_signal = 0;
		  belepesek = 0;
		  i++;
			if((tmp_arr[i] - tmp_arr[i - 1]) > 30){
				ir_signal |= (1 << i);
				BSP_LED_Toggle(LED3);

			}
		}
  }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	//BSP_LED_Toggle(LED_GREEN);
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	tmp_arr[belepesek] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
	if(belepesek > 0){
		if((tmp_arr[belepesek] - tmp_arr[belepesek - 1]) > 30){
			ir_signal |= (1 << belepesek);
			BSP_LED_Toggle(LED3);

		}
	}
	if(belepesek < 12){
		belepesek++;
	}else{
		belepesek = 0;
		for(int j = 0; j < 20; j++){
			tmp_arr[j] = 0;
			BSP_LED_Toggle(LED_GREEN);
		}
	}
}

void XferCpltCallback(){
	BSP_LED_Toggle(LED_GREEN);
	HAL_TIM_PWM_Stop_DMA(&TimHandle, TIM_CHANNEL_4);
	HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_4);

}

static void Error_Handler(void)
{
  while(1)
  {
    /* Toggle LED3 for error */
    BSP_LED_Toggle(LED3);
    HAL_Delay(1000);
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = HSI
  *            SYSCLK(Hz)                     = 32000000
  *            HCLK(Hz)                       = 32000000
  *            AHB Prescaler                  = 2
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 16
  *            Flash Latency(WS)              = 0
  *            Main regulator output voltage  = Scale3 mode
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }
  
  /* Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  
  /* Disable Power Control clock */
  __HAL_RCC_PWR_CLK_DISABLE();
  /**Configure the Systick interrupt time
  */
HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  /**Configure the Systick
  */
HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

/* SysTick_IRQn interrupt configuration */
HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
  
#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
