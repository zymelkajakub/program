/*

		*******************************************************************************
		* @file    temp_adc.h																													*
		* @author  Jakub Zymelka - zymelkajakub@gmail.com															*
		* @version V1.0																																*
		* @date    14-February-2013																										*
		* @brief   Header dla temp_adc.c																							*
		*******************************************************************************
*/	


void ADC3_DMA_Config(void);
void czytaj_temp_adc(double (uint16_t)(stara_temp*10), uint8_t adc);
