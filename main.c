/*

		*******************************************************************************
		* @file    main.h																															*
		* @author  Jakub Zymelka - zymelkajakub@gmail.com															*
		* @version V1.0																																*
		* @date    14-February-2013																										*
		* @brief   Plik main aplikacji sterowania ogrzewaczem przeplywowym						*
		*******************************************************************************
	
		Zrobione
		
		- dodac obsluge SSD1963 poprzez FSMC
		- dodac obsluge touch panelu (TSC2046, komun. SPI) lub (STMPE811 SPI/I2C)
				oraz dodanie i obsluga przerwania od touch panelu
		- ...
		
		
		
		TODO
		
		- w obsludze LCD'ka sprawdzic czy piny podl. sa odpowiednie  <- !!!!@@@@!!!!
		
		- stworzenie obrazu karty SD wraz z opisem adresowym poszczegolnych danych
		- po uruchomieniu urzadzenia pobieranie ustawien z karty SD z bloku 1:
		
				- sprawdzenie i w razie istnienia pobranie wartosci kalibracyjnych touch panelu
				- ustawienia wybranych czujnikow
				- wartosc ustawionego podswietlenia
				- wartosci min i max temperatury zewnetrznej
			
		- komunikcja z czescia mocy RS485
		- dodac przerwanie co 0,5s i w obsludze pomiar 2 temperatur zewn i wewn z ADC'ka z usrednianiem
				funkcja czytaj_temp_adc(double (uint16_t)(stara_temp*10), uint8_t adc)
				
				LUB
				
				zaleznie od zastosowanego ukladu do obslugi Touch Panelu
				czytac temperatury za pomoca pozostalych 4 ADC'ow w ukladzie STMPE811
		- ...
		
*/


#include "stm32f4xx.h"
#include <stdio.h>
#include <math.h>

void delay(__IO uint32_t nCount);

#include "stm32f4_discovery.h"
#include "stm32f4_ssd1963_fsmc.h"
//#include "cyfry_duze.c"
//#include "temp_adc.h"
#include "fonts.c"
#include "fonts.h"
#include "inc\STMPE811QTR.h"
#include "inc\LCD_Touch_Calibration.h"

/* Zmienne ---------------------------------------------------------*/

uint8_t aa,bb,cc,dd,aa1,bb1,cc1,dd1;
uint16_t temp_wody=0;
double temp_wewn=0,temp_zewn=0;

/* Prototypy funkcji -----------------------------------------------*/
/*void pisz_liczbe(double liczba, uint16_t pozx, uint16_t pozy);
void pisz_liczbe1(double liczba, uint16_t pozx, uint16_t pozy);*/


int main(void)
{

	Touch_init();	  			// Inicjalizacja kontrolera Touch panel'u STMPE811
	LCD_SSD1963_Init();		// Inicjalizacja kontrolera wyswietlacza SSD1963
	
	Lcd_Touch_Calibration();  //dodac warunek sprawdzania kalibracji z SD <---@@@!!!@@@
	
	
//	inicjalizacja();	// Inicjalizacja peryferiow i wyswietlenie interfejsu bez temperatur


	
	while(1)
	{
			/*
					W to miejsce wstawic kod wyswietlania temperatury wody z RS'a,
					wyswietlania temperatur zewnetrznej i wewnetrznej oraz
					aktualizacji ich wartosci maksymalnych i minimalnych
			*/
		
		
	}
}

/**
  * @brief  Inserts a delay time.
  * @param  nCount: specifies the delay time length.
  * @retval None
  */
void delay(__IO uint32_t nCount)
{
  __IO uint32_t index = 0; 
  for (index = (100000 * nCount); index != 0; index--);
}

/*
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


/*-------------------------------------------------------
		Czytanie z karty poszczegolnych blokow 
		i wyswietlanie zawartosci tych blokow
		w oknie terminala RS'a
  -------------------------------------------------------
		
	SD_ReadSingleBlock(inbuf, 2998);
	SD_WriteSingleBlock(inbuf, 3004);
	UART_SendLine("Block 2998:\r\n");
  UART_Send(inbuf,512);
  SD_ReadSingleBlock(inbuf, 2999);
	SD_WriteSingleBlock(inbuf, 3005);
  UART_SendLine("\r\n");
	UART_SendLine("Block 2999:\r\n");
  UART_Send(inbuf,512);
	
		*/

/*-------------------------------------------------------
		Wyswietlenie okna MENU
  -------------------------------------------------------

		SSD1963_ClearScreen(czarny);
	
	LCD_pisz_tekst(142,100,"WYBOR CZUJNIKOW TEMPERATURY");
	LCD_pisz_tekst(180,175,"USTAWIENIA REGULATORA PID");
	LCD_pisz_tekst(244,214,"JASNOSC I KONTRAST");
	LCD_pisz_tekst(284,253,"DATA I GODZINA");
	LCD_pisz_tekst(219,292,"RESETOWANIE WSKAZAN");
	
		*/

/*-------------------------------------------------------
		Wzor definicji wyswietlania tekstu
  -------------------------------------------------------
		
#define MESSAGE1   "abcdefghijklmnopqrstuvwxyz" 
#define MESSAGE0   "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
#define MESSAGE2   "0123456789" 
#define MESSAGE3   ":'<>/[]{} _=+-" 
#define MESSAGE4   "Jakub Zymelka" 
#define MESSAGE5   "(C)2012" 

	nastepnie:
								 x,liniaY,tekst
	LCD_pisz_tekst(0,3,MESSAGE6);
	LCD_pisz_tekst(0,30,MESSAGE6);
	LCD_pisz_tekst(0,57,MESSAGE6);
	LCD_pisz_tekst(0,84,MESSAGE1);
	LCD_pisz_tekst(0,111,MESSAGE0);
	LCD_pisz_tekst(0,138,MESSAGE2);
	LCD_pisz_tekst(0,165,MESSAGE3);
	
	lub:
	
	LCD_pisz_tekst(0,350,"temperatura czujnika");

		*/
		
/*-------------------------------------------------------
		Zczytywanie wartosci ADC i wyswietlenie w formie U [V]
  -------------------------------------------------------
			
ADC3ConvertedVoltage = (ADC3ConvertedValue[3]*294)/0xFFF;

		*/
		
/*-------------------------------------------------------
		wyswietlenie wartosci ADC na LCD
  -------------------------------------------------------

void Display(void)
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
		
		
/*-------------------------------------------------------
	wyswietlenie DUZEJ liczby na LCD w pozycji szer x wys y
  -------------------------------------------------------
pisz_liczbe1(liczba, y, x);

		*/

/*-------------------------------------------------------
	wyswietlenie obrazka z karty SD i mrugniecie dioda
  -------------------------------------------------------

	UART_SendLine("Wyswietlenie obrazka (sektory 0..1499)...\r\n");
	SSD1963_SetArea(0, TFT_WIDTH-1 , 0, TFT_HEIGHT-1);
	SSD1963_WriteCommand(0x2c);
	sektor=0;
	while(sektor<1500)
	{
		// Czytaj blok 512 bajtow do "inbuf" z "sektor"
		SD_ReadSingleBlock(inbuf, sektor);
		a = 0x100;
		while(a>0)
		{
			b = 256 - a;
			w16bit = (((((inbuf[b*2]) << 8 ) & 0xFF00)) + (inbuf[(b*2)+1])); 
			SSD1963_WriteData(w16bit); // przekonwertowanie danych na wartosc koloru i wyswietlenie pikselu
			a--;
		}
		sektor++;
	}
		
	GPIOD->BSRRH = (1 << 5);	// mrugniecie dioda czerwona
	Delay(200000);						
	GPIOD->BSRRL = (1 << 5);
		*/
		
//		GPIOD->ODR ^= GPIO_Pin_5; 		// zmiana stanu na porcie		

/*-------------------------------------------------------
	  czyszczenie ekranu na czarno
  -------------------------------------------------------
	LCD_WriteReg(SSD2119_RAM_DATA_REG, 0x0000);
  for(ulCount = 0; ulCount < (LCD_PIXEL_WIDTH * LCD_PIXEL_HEIGHT); ulCount++)
  {
    LCD_WriteRAM(0x0000);
  }
  LCD_SetFont(&LCD_DEFAULT_FONT);
	
		*/
		
		
/*

uint8_t podsw_max=0;												// wartosc zczytywana z karty podczas pierwszego uruchomienia
uint32_t sektor=0;
uint8_t outbuf[512];
uint8_t inbuf[512];
uint16_t z,x,w16bit;

void inicjalizacja(void)
{
	volatile uint32_t i=0,j=0;	
	
	*/// Enable GPIOA, GPIOD, GPIOE, GPIOF and GPIOG interface clock
//  RCC->AHB1ENR   = 0x0000001D;
  
	/* Connect PAx pins to FSMC Alternate function */
/*GPIOA->AFR[0]  = 0x00cc00cc;
  GPIOA->AFR[1]  = 0xcc0ccccc;*/
  /* Configure PAx pins in General purpose output mode */  
//  GPIOA->MODER   = 0xA8040000;
  /* Configure PAx pins speed to 2 MHz */  
//  GPIOA->OSPEEDR = 0x00000000;
  /* Configure PAx pins Output type to push-pull */  
//  GPIOA->OTYPER  = 0x00000000;
  /* Pull-up for PAx pins */ 
//  GPIOA->PUPDR   = 0x64000000;
//	GPIOA->BSRRH	 = (1 << 9);

  /* Connect PDx pins to FSMC Alternate function */
/*GPIOD->AFR[0]  = 0x00cc00cc;
  GPIOD->AFR[1]  = 0xcc0ccccc;*/
  /* Configure PDx pins in General purpose output mode */  
//  GPIOD->MODER   = 0x55400400;
  /* Configure PDx pins speed to 2 MHz */  
//  GPIOD->OSPEEDR = 0x00000000;
  /* Configure PDx pins Output type to push-pull */  
//  GPIOD->OTYPER  = 0x00000000;
  /* Pull-up for PDx pins */ 
//  GPIOD->PUPDR   = 0x55400400;
	
  /* Connect PEx pins to FSMC Alternate function */
/*GPIOE->AFR[0]  = 0xc00cc0cc;
  GPIOE->AFR[1]  = 0xcccccccc;*/
  /* Configure PEx pins in General purpose output mode */ 
//  GPIOE->MODER   = 0x55555555;
  /* Configure PEx pins speed to 2 MHz */ 
//  GPIOE->OSPEEDR = 0x00000000;
  /* Configure PEx pins Output type to push-pull */  
//  GPIOE->OTYPER  = 0x00000000;
  /* Pull-up for PEx pins */ 
//  GPIOE->PUPDR   = 0x55555555;
	
/*	GPIOD->BSRRH 	 = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIOD->BSRRL 	 = GPIO_Pin_5;
			
//	SSD1963_Init();				// Inicjalizacja wyswietlacza i wygaszenie podswietlenia
//	SetBacklight(0x00);
	
	UART_LowLevel_Init(); // Inicjalizacja interfejsu RS
	ADC3_DMA_Config();		// Inicjalizacja przetwornikow ADC	
  SD_LowLevel_Init();   // Inicjalizacja pinow, tablicy wektorow oraz interfejsu SDIO
	
  UART_SendLine("LCD, LED, UART initialization OK.\r\n");
	UART_SendLine("Initialize SD Card...\r\n");
	
	if ( GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_0) == (uint8_t)Bit_RESET )
	{
		UART_SendLine("SD Card detected...\r\n");
		SD_Init();  //Po powrocie z tej funkcji karta sd jest w trybie "transfer mode".
		UART_SendLine("SD Card initialization OK\r\n");	
	} else
		{
			UART_SendLine("SD Card NOT detected.\r\n");
			while(1)
			{};
		}

	UART_SendLine("\r\n");
	UART_SendLine("Czyszczenie LCD...\r\n");
	SSD1963_ClearScreen(czarny);
	
	UART_SendLine("Wyswietlenie interfejsu bez temperatur (sektory 1500..2999)...\r\n");
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
	
	
	for(i=0x00;i<0xpodsw_max;i++)		// Stopniowe zwiekszenie podswietlenia LCD do wartosci podsw_max
	{
		Delay(50);
		SetBacklight(i);
	}
	
}

		*/
