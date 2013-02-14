/**
  ******************************************************************************
  * @file    stm32f4_discovery.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    28-October-2011
  * @brief   This file provides set of firmware functions to manage Leds and
  *          push-button available on STM32F4-Discovery Kit from STMicroelectronics.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 
  
/* Includes ------------------------------------------------------------------*/
#include "stm32f4_discovery.h"
#include "ssd1963.c"
#include "inc\UARTLib.h"
#include "inc\SDIO.h"


GPIO_InitTypeDef GPIO_InitStructure;

/** @addtogroup Utilities
  * @{
  */ 

/** @addtogroup STM32F4_DISCOVERY
  * @{
  */   
    
/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL 
  * @brief This file provides set of firmware functions to manage Leds and push-button
  *        available on STM32F4-Discovery Kit from STMicroelectronics.
  * @{
  */ 

/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Private_Defines
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Private_Variables
  * @{
  */ 

uint8_t podsw_max=0;												// wartosc zczytywana z karty podczas pierwszego uruchomienia
uint32_t sektor=0;
uint8_t outbuf[512];
uint8_t inbuf[512];
uint16_t z,x,w16bit;

GPIO_TypeDef* GPIO_PORT[LEDn] = {LED4_GPIO_PORT, LED3_GPIO_PORT, LED5_GPIO_PORT,
                                 LED6_GPIO_PORT};
const uint16_t GPIO_PIN[LEDn] = {LED4_PIN, LED3_PIN, LED5_PIN,
                                 LED6_PIN};
const uint32_t GPIO_CLK[LEDn] = {LED4_GPIO_CLK, LED3_GPIO_CLK, LED5_GPIO_CLK,
                                 LED6_GPIO_CLK};

GPIO_TypeDef* BUTTON_PORT[BUTTONn] = {USER_BUTTON_GPIO_PORT }; 

const uint16_t BUTTON_PIN[BUTTONn] = {USER_BUTTON_PIN }; 

const uint32_t BUTTON_CLK[BUTTONn] = {USER_BUTTON_GPIO_CLK };

const uint16_t BUTTON_EXTI_LINE[BUTTONn] = {USER_BUTTON_EXTI_LINE };

const uint8_t BUTTON_PORT_SOURCE[BUTTONn] = {USER_BUTTON_EXTI_PORT_SOURCE};
								 
const uint8_t BUTTON_PIN_SOURCE[BUTTONn] = {USER_BUTTON_EXTI_PIN_SOURCE }; 
const uint8_t BUTTON_IRQn[BUTTONn] = {USER_BUTTON_EXTI_IRQn };

USART_TypeDef* COM_USART[COMn] = {EVAL_COM1,EVAL_COM2}; 

GPIO_TypeDef* COM_TX_PORT[COMn] = {EVAL_COM1_TX_GPIO_PORT, EVAL_COM2_TX_GPIO_PORT};
 
GPIO_TypeDef* COM_RX_PORT[COMn] = {EVAL_COM1_RX_GPIO_PORT, EVAL_COM2_RX_GPIO_PORT};

const uint32_t COM_USART_CLK[COMn] = {EVAL_COM1_CLK, EVAL_COM2_CLK};

const uint32_t COM_TX_PORT_CLK[COMn] = {EVAL_COM1_TX_GPIO_CLK, EVAL_COM2_TX_GPIO_CLK};
 
const uint32_t COM_RX_PORT_CLK[COMn] = {EVAL_COM1_RX_GPIO_CLK, EVAL_COM2_RX_GPIO_CLK};

const uint16_t COM_TX_PIN[COMn] = {EVAL_COM1_TX_PIN, EVAL_COM2_TX_PIN};

const uint16_t COM_RX_PIN[COMn] = {EVAL_COM1_RX_PIN, EVAL_COM2_RX_PIN};
 
const uint16_t COM_TX_PIN_SOURCE[COMn] = {EVAL_COM1_TX_SOURCE, EVAL_COM2_TX_SOURCE};

const uint16_t COM_RX_PIN_SOURCE[COMn] = {EVAL_COM1_RX_SOURCE, EVAL_COM2_RX_SOURCE};
 
const uint16_t COM_TX_AF[COMn] = {EVAL_COM1_TX_AF, EVAL_COM2_TX_AF};
 
const uint16_t COM_RX_AF[COMn] = {EVAL_COM1_RX_AF, EVAL_COM2_RX_AF};

DMA_InitTypeDef    sEEDMA_InitStructure; 
NVIC_InitTypeDef   NVIC_InitStructure;

/**
  * @}
  */ 


/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Private_FunctionPrototypes
  * @{
  */ 

/**
  * @}
  */ 

/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Private_Functions
  * @{
  */ 

void inicjalizacja(void)
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
			
	SSD1963_Init();				// Inicjalizacja wyswietlacza i wygaszenie podswietlenia
	SetBacklight(0x00);
	
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

/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured. 
  *   This parameter can be one of following parameters:
  *     @arg LED4
  *     @arg LED3
  *     @arg LED5
  *     @arg LED6
  * @retval None
  */
void STM_EVAL_LEDInit(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Enable the GPIO_LED Clock */
  RCC_AHB1PeriphClockCmd(GPIO_CLK[Led], ENABLE);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_PIN[Led];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIO_PORT[Led], &GPIO_InitStructure);
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on. 
  *   This parameter can be one of following parameters:
  *     @arg LED4
  *     @arg LED3
  *     @arg LED5
  *     @arg LED6  
  * @retval None
  */
void STM_EVAL_LEDOn(Led_TypeDef Led)
{
  GPIO_PORT[Led]->BSRRL = GPIO_PIN[Led];
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off. 
  *   This parameter can be one of following parameters:
  *     @arg LED4
  *     @arg LED3
  *     @arg LED5
  *     @arg LED6 
  * @retval None
  */
void STM_EVAL_LEDOff(Led_TypeDef Led)
{
  GPIO_PORT[Led]->BSRRH = GPIO_PIN[Led];  
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled. 
  *   This parameter can be one of following parameters:
  *     @arg LED4
  *     @arg LED3
  *     @arg LED5
  *     @arg LED6  
  * @retval None
  */
void STM_EVAL_LEDToggle(Led_TypeDef Led)
{
  GPIO_PORT[Led]->ODR ^= GPIO_PIN[Led];
}

/**
  * @brief  Configures Button GPIO and EXTI Line.
  * @param  Button: Specifies the Button to be configured.
  *   This parameter should be: BUTTON_USER
  * @param  Button_Mode: Specifies Button mode.
  *   This parameter can be one of following parameters:   
  *     @arg BUTTON_MODE_GPIO: Button will be used as simple IO 
  *     @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
  *                            generation capability  
  * @retval None
  */
void STM_EVAL_PBInit(Button_TypeDef Button, ButtonMode_TypeDef Button_Mode)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the BUTTON Clock */
  RCC_AHB1PeriphClockCmd(BUTTON_CLK[Button], ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Configure Button pin as input */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = BUTTON_PIN[Button];
  GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStructure);

  if (Button_Mode == BUTTON_MODE_EXTI)
  {
    /* Connect Button EXTI Line to Button GPIO Pin */
    SYSCFG_EXTILineConfig(BUTTON_PORT_SOURCE[Button], BUTTON_PIN_SOURCE[Button]);

    /* Configure Button EXTI line */
    EXTI_InitStructure.EXTI_Line = BUTTON_EXTI_LINE[Button];
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = BUTTON_IRQn[Button];
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure); 
  }
}

/**
  * @brief  Returns the selected Button state.
  * @param  Button: Specifies the Button to be checked.
  *   This parameter should be: BUTTON_USER  
  * @retval The Button GPIO pin value.
  */
uint32_t STM_EVAL_PBGetState(Button_TypeDef Button)
{
  return GPIO_ReadInputDataBit(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}

/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */   

/**
  * @}
  */ 
    
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
