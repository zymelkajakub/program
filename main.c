#include "stm32f4xx.h"
#include <stdio.h>
#include <math.h>
#include "stm32f4_discovery.h"
#include "ssd1963.c"
#include "inc\UARTLib.h"
#include "inc\SDIO.h"
#include "cyfry_duze.c"

#define MESSAGE1   "abcdefghijklmnopqrstuvwxyz" 
#define MESSAGE0   "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
#define MESSAGE2   "0123456789" 
#define MESSAGE3   ":'<>/[]{} _=+-" 
#define MESSAGE4   "Jakub Zymelka" 
#define MESSAGE5   "(C)2012" 
#define MESSAGE6   "Jakub i Marcia sie bardzo kochaja!" 

#define ADC_CCR_ADDRESS    ((uint32_t)0x40012308)

ADC_CommonInitTypeDef ADC_CommonInitStructure;
DMA_InitTypeDef DMA_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
ADC_InitTypeDef ADC_InitStructure;


/* Private variables ---------------------------------------------------------*/
__IO uint16_t ADC3ConvertedValue[4];
__IO double ADC3ConvertedVoltage = 0;


uint8_t aa,bb,cc,dd,aa1,bb1,cc1,dd1;
uint16_t z,x,w16bit;
uint32_t sektor=0,temp=0;
uint8_t outbuf[512];
uint8_t inbuf[512];
uint16_t tempbuf[512];
double liczba=0,liczba1=0;

/* Private functions ---------------------------------------------------------*/
void ADC3_DMA_Config(void);
void pisz_liczbe(double liczba, uint16_t pozx, uint16_t pozy);
void pisz_liczbe1(double liczba, uint16_t pozx, uint16_t pozy);
void Delay(__IO uint32_t nCount);
void Display(void);

int main(void)
{
	volatile uint32_t i=0,j=0;	
	
	/* Enable GPIOA, GPIOD, GPIOE, GPIOF and GPIOG interface clock */
  RCC->AHB1ENR   = 0x0000001D;
  
	/* Connect PAx pins to FSMC Alternate function */
/*GPIOA->AFR[0]  = 0x00cc00cc;
  GPIOA->AFR[1]  = 0xcc0ccccc;*/
  /* Configure PAx pins in General purpose output mode */  
  GPIOA->MODER   = 0xA8040000;
  /* Configure PAx pins speed to 2 MHz */  
  GPIOA->OSPEEDR = 0x00000000;
  /* Configure PAx pins Output type to push-pull */  
  GPIOA->OTYPER  = 0x00000000;
  /* Pull-up for PAx pins */ 
  GPIOA->PUPDR   = 0x64000000;
	GPIOA->BSRRH	 = (1 << 9);

  /* Connect PDx pins to FSMC Alternate function */
/*GPIOD->AFR[0]  = 0x00cc00cc;
  GPIOD->AFR[1]  = 0xcc0ccccc;*/
  /* Configure PDx pins in General purpose output mode */  
  GPIOD->MODER   = 0x55400400;
  /* Configure PDx pins speed to 2 MHz */  
  GPIOD->OSPEEDR = 0x00000000;
  /* Configure PDx pins Output type to push-pull */  
  GPIOD->OTYPER  = 0x00000000;
  /* Pull-up for PDx pins */ 
  GPIOD->PUPDR   = 0x55400400;
	
  /* Connect PEx pins to FSMC Alternate function */
/*GPIOE->AFR[0]  = 0xc00cc0cc;
  GPIOE->AFR[1]  = 0xcccccccc;*/
  /* Configure PEx pins in General purpose output mode */ 
  GPIOE->MODER   = 0x55555555;
  /* Configure PEx pins speed to 2 MHz */ 
  GPIOE->OSPEEDR = 0x00000000;
  /* Configure PEx pins Output type to push-pull */  
  GPIOE->OTYPER  = 0x00000000;
  /* Pull-up for PEx pins */ 
  GPIOE->PUPDR   = 0x55555555;
	
	GPIOD->BSRRH 	 = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIOD->BSRRL 	 = GPIO_Pin_5;
			
	SSD1963_Init();
	SetBacklight(0x9F);
	
	UART_LowLevel_Init();
	
  SD_LowLevel_Init();   //Initialize PINS, vector table and SDIO interface
	
  UART_SendLine("LCD, LED, UART initialization OK.\r\n");
	UART_SendLine("Initialize SD Card...\r\n");
	
	if ( GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_0) == (uint8_t)Bit_RESET )
	{
		UART_SendLine("SD Card detected...\r\n");
		SD_Init();  //After return from this function sd is in transfer mode.
		UART_SendLine("SD Card initialization OK\r\n");	
	} else
		{
			UART_SendLine("SD Card NOT detected.\r\n");
			while(1)
			{};
		}

	
	SD_ReadSingleBlock(inbuf, 2998);
	SD_WriteSingleBlock(inbuf, 3004);
	UART_SendLine("Block 2998:\r\n");
  UART_Send(inbuf,512);
  SD_ReadSingleBlock(inbuf, 2999);
	SD_WriteSingleBlock(inbuf, 3005);
  UART_SendLine("\r\n");
	UART_SendLine("Block 2999:\r\n");
  UART_Send(inbuf,512);
	
	
	UART_SendLine("\r\n");
	UART_SendLine("Czyszczenie LCD...\r\n");
	SSD1963_ClearScreen(bialy);
		
	UART_SendLine("Wyswietlenie obrazka (sektory 1500..2999)...\r\n");
	SSD1963_SetArea(0, TFT_WIDTH-1 , 0, TFT_HEIGHT-1);
	SSD1963_WriteCommand(0x2c);
	sektor=1500;
	while(sektor<3000)
	{
		// Read block of 512 bytes from sektor
		SD_ReadSingleBlock(inbuf, sektor);
		z = 0x100;
		while(z>0)
		{
			x = 256 - z;
			w16bit = (((((inbuf[x*2]) << 8 ) & 0xFF00)) + (inbuf[(x*2)+1]));
			SSD1963_WriteData(w16bit);
			z--;
		}
		sektor++;
	}
	
	//GPIOD->BSRRL = GPIO_Pin_5;
	
  //GPIOC->BSRRL = GPIO_Pin_1;         //Makes TP_UP to 0V
  //GPIOC->BSRRH = GPIO_Pin_;         //Makes TP_DOWN to 3,3V
	
	ADC3_DMA_Config();
	
	SSD1963_ClearScreen(czarny);
	
  /* Start ADC3 Software Conversion */ 

	LCD_pisz_tekst(0,3,MESSAGE6);
	LCD_pisz_tekst(0,30,MESSAGE6);
	LCD_pisz_tekst(0,57,MESSAGE6);
	LCD_pisz_tekst(0,84,MESSAGE1);
	LCD_pisz_tekst(0,111,MESSAGE0);
	LCD_pisz_tekst(0,138,MESSAGE2);
	LCD_pisz_tekst(0,165,MESSAGE3);
	SSD1963_ClearScreen(czarny);
	
	LCD_pisz_tekst(142,100,"WYBOR CZUJNIKOW TEMPERATURY");
	LCD_pisz_tekst(180,175,"USTAWIENIA REGULATORA PID");
	LCD_pisz_tekst(244,214,"JASNOSC I KONTRAST");
	LCD_pisz_tekst(284,253,"DATA I GODZINA");
	LCD_pisz_tekst(219,292,"RESETOWANIE WSKAZAN");
//	SSD1963_ClearScreen(czarny);
//	LCD_pisz_tekst(0,350,"temperatura czujnika");
	
	while(1)
	{
	/*	if (i>=128) i=0;
		ADC3ConvertedVoltage = (ADC3ConvertedValue[3]*294)/0xFFF;
		//Display();
		pisz_liczbe(ADC3ConvertedVoltage, 0,0);
		
		liczba = ADC3ConvertedValue[2];
		liczba1 = log((0xFFF/(liczba))-1);
		liczba1 = (3380/(liczba1+11.27457892))-273.2;
		tempbuf[i] = (uint16_t)(liczba1*10);

		//liczba = (3380/(log(4095/ADC3ConvertedValue[2]-1)+log(9400)-log(0.1193)))-273.2;
		for(j=0;j<128;j++)
		{
			temp += tempbuf[j];
		}
		temp/=128;
		liczba1 = (double)temp/10;
		pisz_liczbe1(liczba1, 200,300);
		GPIOD->ODR ^= GPIO_Pin_5;
		
		i++;
		*/
		//Delay(20000);
		/*
	UART_SendLine("Wyswietlenie pierwszego obrazka (sektory 0..1499)...\r\n");
	SSD1963_SetArea(0, TFT_WIDTH-1 , 0, TFT_HEIGHT-1);
	SSD1963_WriteCommand(0x2c);
	sektor=0;
	while(sektor<1500)
	{
		// Read block of 512 bytes from sektor
		SD_ReadSingleBlock(inbuf, sektor);
		a = 0x100;
		while(a>0)
		{
			b = 256 - a;
			w16bit = (((((inbuf[b*2]) << 8 ) & 0xFF00)) + (inbuf[(b*2)+1]));
			SSD1963_WriteData(w16bit);
			a--;
		}
		sektor++;
	}
		
	GPIOD->BSRRH = (1 << 5);
	Delay(200000);
	GPIOD->BSRRL = (1 << 5);
	
	UART_SendLine("Wyswietlenie drugiego obrazka (sektory 1500..2999)...\r\n");
	SSD1963_SetArea(0, TFT_WIDTH-1 , 0, TFT_HEIGHT-1);
	SSD1963_WriteCommand(0x2c);
	sektor=1500;
	while(sektor<3000)
	{
		// Read block of 512 bytes from sektor
		SD_ReadSingleBlock(inbuf, sektor);
		a = 0x100;
		while(a>0)
		{
			b = 256 - a;
			w16bit = (((((inbuf[b*2]) << 8 ) & 0xFF00)) + (inbuf[(b*2)+1]));
			SSD1963_WriteData(w16bit);
			a--;
		}
		sektor++;
	}
		
	GPIOD->BSRRH = (1 << 5);
	Delay(200000);
	GPIOD->BSRRL = (1 << 5);
	*/}

}
		
/**
  * @brief  Delay Function.
  * @param  nCount:specifies the Delay time length.
  * @retval None
  */
void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}

void pisz_liczbe(double liczba, uint16_t pozx, uint16_t pozy)
{
	uint8_t a,b,c,d,e;
	e = liczba/1000;
	a = (liczba/100)-(e*1000);
	b = (liczba-(e*1000)-(a*100))/10;
	c = (liczba-(e*1000)-(a*100))-(b*10);
	d = ((((liczba-(e*1000)-(a*100))-(b*10))-c))*10;

	if (aa == a) 				{}
											 else if (a == 0) {SSD1963_FillArea(pozx,	  pozx+138,  pozy, pozy+166, czarny); aa = a;}
											 else {						
																						SSD1963_FillArea(pozx, 	  pozx+138,  pozy, pozy+166, czarny); pisz_cyfre(a, pozx, 	pozy); aa = a;
														}
														
	if (bb == b) 			{}
											 else if (( b == 0) & (a == 0)) {SSD1963_FillArea(pozx+138,pozx+276,  pozy, pozy+166, czarny);}
											 else {						
																					SSD1963_FillArea(pozx+138,pozx+276,  pozy, pozy+166, czarny); pisz_cyfre(b, pozx+138, pozy); bb = b;
														}
														
	if (cc==c) {} else { 										SSD1963_FillArea(pozx+276,pozx+414,  pozy, pozy+166, czarny); pisz_cyfre(c, pozx+276,	pozy); cc = c; 
																							pisz_przecinek(pozx+414,pozy+140);}
	if (dd==d) {} else { 										SSD1963_FillArea(pozx+439,pozx+577,  pozy, pozy+166, czarny); pisz_cyfre(d, pozx+439, pozy); dd = d;}

}

void pisz_liczbe1(double liczba, uint16_t pozx, uint16_t pozy)
{
	uint8_t a,b,c,d,e;
	e = liczba/1000;
	a = (liczba/100)-(e*1000);
	b = (liczba-(e*1000)-(a*100))/10;
	c = (liczba-(e*1000)-(a*100))-(b*10);
	d = ((((liczba-(e*1000)-(a*100))-(b*10))-c))*10;

	if (aa1 == a) 				{}
											 else if (a == 0) {SSD1963_FillArea(pozx,	  pozx+138,  pozy, pozy+166, czarny); aa1 = a;}
											 else {						
																						SSD1963_FillArea(pozx, 	  pozx+138,  pozy, pozy+166, czarny); pisz_cyfre(a, pozx, 	pozy); aa1 = a;
														}
														
	if (bb1 == b) 			{}
											 else if (( b == 0) & (a == 0)) {SSD1963_FillArea(pozx+138,pozx+276,  pozy, pozy+166, czarny);}
											 else {						
																					SSD1963_FillArea(pozx+138,pozx+276,  pozy, pozy+166, czarny); pisz_cyfre(b, pozx+138, pozy); bb1 = b;
														}
	if (cc1==c) {} else { 										SSD1963_FillArea(pozx+276,pozx+414,  pozy, pozy+166, czarny); pisz_cyfre(c, pozx+276,	pozy); cc1 = c; 
																							pisz_przecinek(pozx+414,pozy+140);}
	if (dd1==d) {} else { 										SSD1963_FillArea(pozx+439,pozx+577,  pozy, pozy+166, czarny); pisz_cyfre(d, pozx+439, pozy); dd1 = d;}

}

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
/*
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint32_t BufferLength)
{
  while (BufferLength--)
  {
    if (*pBuffer1 != *pBuffer2)
    {
      return FAILED;
    }

    pBuffer1++;
    pBuffer2++;
  }

  return PASSED;
}
*/
/**
  * @brief  Fills buffer with user predefined data.
  * @param  pBuffer: pointer on the Buffer to fill
  * @param  BufferLength: size of the buffer to fill
  * @param  Offset: first value to fill on the Buffer
  * @retval None
  *
void Fill_Buffer(uint8_t *pBuffer, uint32_t BufferLength, uint32_t Offset)
{
  uint16_t index = 0;

  // Put in global buffer same values 
  for (index = 0; index < BufferLength; index++)
  {
    pBuffer[index] = index + Offset;
  }
}
*/

/**
  * @brief  Checks if a buffer has all its values are equal to zero.
  * @param  pBuffer: buffer to be compared.
  * @param  BufferLength: buffer's length
  * @retval PASSED: pBuffer values are zero
  *         FAILED: At least one value from pBuffer buffer is different from zero.
  *//*
TestStatus eBuffercmp(uint8_t* pBuffer, uint32_t BufferLength)
{
  while (BufferLength--)
  {
    // In some SD Cards the erased state is 0xFF, in others it's 0x00
    if ((*pBuffer != 0xFF) && (*pBuffer != 0x00))
    {
      return FAILED;
    }

    pBuffer++;
  }

  return PASSED;
}
*/
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

