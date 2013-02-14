/*

		*******************************************************************************
		* @file    temp_adc.c																													*
		* @author  Jakub Zymelka - zymelkajakub@gmail.com															*
		* @version V1.0																																*
		* @date    14-February-2013																										*
		* @brief   Obsluga ADC'ow oraz pomiar i konwersja temperatury z NTC						*
		*******************************************************************************
*/


#include "temp_adc.h"

#define ADC_CCR_ADDRESS    ((uint32_t)0x40012308)

ADC_InitTypeDef ADC_InitStructure;
ADC_CommonInitTypeDef ADC_CommonInitStructure;
DMA_InitTypeDef DMA_InitStructure;
__IO uint16_t ADC3ConvertedValue[4];
__IO double ADC3ConvertedVoltage = 0;



/**
  * @brief  ADC3 channel07 with DMA configuration
  * @param  None
  * @retval None
  */
void ADC3_DMA_Config(void)
{
  
  /* Enable peripheral clocks *************************************************/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_ADC2, ENABLE);

  /* DMA2 Stream0 channel0 configuration **************************************/
  DMA_InitStructure.DMA_Channel = DMA_Channel_0; 
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC3ConvertedValue;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC_CCR_ADDRESS;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 4;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);

  /* DMA2_Stream0 enable */
  DMA_Cmd(DMA2_Stream0, ENABLE);
  
  /* ADCs configuration ------------------------------------------------------*/
  /* Configure ADC Channel10, 11, 12 pin as analog input */
  /* ADC Channel 10 -> PC0
     ADC Channel 11 -> PC1
     ADC Channel 12 -> PC2
  */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* ADC Common Init */
  ADC_CommonInitStructure.ADC_Mode = ADC_DualMode_RegSimult;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);


  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 2;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channels 10, 11 configuration */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_3Cycles);

  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 2;
  ADC_Init(ADC2, &ADC_InitStructure);

  /* ADC2 regular channels 11, 12 configuration */ 
  ADC_RegularChannelConfig(ADC2, ADC_Channel_12, 1, ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC2, ADC_Channel_13, 2, ADC_SampleTime_3Cycles);


  /* Enable DMA request after last transfer (Multi-ADC mode)  */
  ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);

  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC2 */
  ADC_Cmd(ADC2, ENABLE);

  /* Start ADC1 Software Conversion */
  ADC_SoftwareStartConv(ADC1);
	
}

/**
  * @brief  Pobiera wartosc starej i usrednia z 10 pomiarami nowej temperatury
  * @param1 stara_temp - aktualna wartosc temp wyswietlana na LCD
	* @param2 adc				 - wybor wartosci z tablicy zczytanej z DMA - kanalu ADC
	* 											dla temperatury zewn adc=2 dla wewn adc=3
  * @retval liczba1 	 - zwraca usredniona temperature w double
  */
void czytaj_temp_adc(double (uint16_t)(stara_temp*10), uint8_t adc)	// zwraca wartosc usredniona z poprzednia wartoscia temp
{
		uint8_t i=0;
		uint16_t liczba=0, temp_buf[10];
		double liczba1=0;
	
			/*	
					Zczytywanie temperatury czujnika z zewnatrz
					temperatura = (3380/(log(4095/wartoscADC-1)+log(9400)-log(0.1193)))-273.2;
			*/
		
		for(i=0;i<10;i++) 															// Usrednianie 10 pomiarow na biezaco
		{
			liczba = ADC3ConvertedValue[adc];								// zczytanie wartosci z ADC'a
			liczba1 = log((0xFFF/(liczba))-1);
			liczba1 = (3380/(liczba1+11.27457892))-273.2;	// przekonwertowanie liczby na temp
			tempbuf[i] = (uint16_t)(liczba1*10);					// zapis temp*100 w formacie uint16 w buforze usredniania
			stara_temp += tempbuf[i];
		}
		liczba1 = (double)stara_temp/11;
		return liczba1;
}
			
/**
  * @brief  Display ADC converted value on LCD
  * @param  None
  * @retval None
  */
/*void Display(void)
{
  double v=0;
	uint32_t mv=0;
  char text[50];

  v=(ADC3ConvertedVoltage/1000);
  mv = (ADC3ConvertedVoltage%1000)/100;
	sprintf((char *)text," ADC = %d V", mv);
	UART_SendLine(text);
	UART_SendLine(" \n\r ");
  pisz_liczbe(v, 150,0);
}
*/