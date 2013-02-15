/**
  ******************************************************************************
  * @file    stm324xg_discovery_lcd.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    30-September-2011
  * @brief   This file contains all the functions prototypes for the 
  *          stm324xg_discovery_lcd.c driver.
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
  * <h2><center>&copy; Portions COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************  
  */ 
/**
  ******************************************************************************
  * <h2><center>&copy; Portions COPYRIGHT 2012 Embest Tech. Co., Ltd.</center></h2>
  * @file    stm32f4_discovery_lcd.h
  * @author  CMP Team
  * @version V1.0.0
  * @date    28-December-2012
  * @brief   This file contains all the functions prototypes for the 
  *          stm324xg_discovery_lcd.c driver.
  *          Modified to support the STM32F4DISCOVERY, STM32F4DIS-BB, STM32F4DIS-CAM
  *          and STM32F4DIS-LCD modules.      
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, Embest SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT
  * OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
  * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
  * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4_DISCOVERY_LCD_H
#define __STM32F4_DISCOVERY_LCD_H

#ifdef __cplusplus
 extern "C" {
#endif 
	 
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "fonts.h"

/** @addtogroup Utilities
  * @{
  */

/** @addtogroup STM32F4_DISCOVERY
  * @{
  */ 

/** @addtogroup STM32F4_DISCOVERY
  * @{
  */
    
/** @addtogroup STM32F4_DISCOVERY_LCD
  * @{
  */ 


/** @defgroup STM32F4_DISCOVERY_LCD_Exported_Types
  * @{
  */
typedef struct 
{
  int16_t X;
  int16_t Y;
} Point, * pPoint;   
/**
  * @}
  */ 


//=============================================================================
// SSD1963 commands
//=============================================================================
#define SSD1963_NOP                       0x00
#define SSD1963_SOFT_RESET                0x01
#define SSD1963_GET_POWER_MODE            0x0A
#define SSD1963_GET_ADDRESS_MODE          0x0B
#define SSD1963_GET_DISPLAY_MODE          0x0D
#define SSD1963_GET_TEAR_EFFECT_STATUS    0x0E
#define SSD1963_ENTER_SLEEP_MODE          0x10
#define SSD1963_EXIT_SLEEP_MODE           0x11
#define SSD1963_ENTER_PARTIAL_MODE        0x12
#define SSD1963_ENTER_NORMAL_MODE         0x13
#define SSD1963_EXIT_INVERT_MODE          0x20
#define SSD1963_ENTER_INVERT_MODE         0x21
#define SSD1963_SET_GAMMA_CURVE           0x26
#define SSD1963_SET_DISPLAY_OFF           0x28
#define SSD1963_SET_DISPLAY_ON            0x29
#define SSD1963_SET_COLUMN_ADDRESS        0x2A
#define SSD1963_SET_PAGE_ADDRESS          0x2B
#define SSD1963_WRITE_MEMORY_START        0x2C
#define SSD1963_READ_MEMORY_START         0x2E
#define SSD1963_SET_PARTIAL_AREA          0x30
#define SSD1963_SET_SCROLL_AREA           0x33
#define SSD1963_SET_TEAR_OFF              0x34
#define SSD1963_SET_TEAR_ON               0x35
#define SSD1963_SET_ADDRESS_MODE          0x36
#define SSD1963_SET_SCROLL_START          0x37
#define SSD1963_EXIT_IDLE_MODE            0x38
#define SSD1963_ENTER_IDLE_MODE           0x39
#define SSD1963_WRITE_MEMORY_CONTINUE     0x3C
#define SSD1963_READ_MEMORY_CONTINUE      0x3E
#define SSD1963_SET_TEAR_SCANLINE         0x44
#define SSD1963_GET_SCANLINE              0x45
#define SSD1963_READ_DDB                  0xA1
#define SSD1963_SET_LCD_MODE              0xB0
#define SSD1963_GET_LCD_MODE              0xB1
#define SSD1963_SET_HORI_PERIOD           0xB4
#define SSD1963_GET_HORI_PERIOD           0xB5
#define SSD1963_SET_VERT_PERIOD           0xB6
#define SSD1963_GET_VERT_PERIOD           0xB7
#define SSD1963_SET_GPIO_CONF             0xB8
#define SSD1963_GET_GPIO_CONF             0xB9
#define SSD1963_SET_GPIO_VALUE            0xBA
#define SSD1963_GET_GPIO_STATUS           0xBB
#define SSD1963_SET_POST_PROC             0xBC
#define SSD1963_GET_POST_PROC             0xBD
#define SSD1963_SET_PWM_CONF              0xBE
#define SSD1963_GET_PWM_CONF              0xBF
#define SSD1963_GET_LCD_GEN0              0xC0
#define SSD1963_SET_LCD_GEN0              0xC1
#define SSD1963_GET_LCD_GEN1              0xC2
#define SSD1963_SET_LCD_GEN1              0xC3
#define SSD1963_GET_LCD_GEN2              0xC4
#define SSD1963_SET_LCD_GEN2              0xC5
#define SSD1963_GET_LCD_GEN3              0xC6
#define SSD1963_SET_LCD_GEN3              0xC7
#define SSD1963_SET_GPIO0_ROP             0xC8
#define SSD1963_GET_GPIO0_ROP             0xC9
#define SSD1963_SET_GPIO1_ROP             0xCA
#define SSD1963_GET_GPIO1_ROP             0xCB
#define SSD1963_SET_GPIO2_ROP             0xCC
#define SSD1963_GET_GPIO2_ROP             0xCD
#define SSD1963_SET_GPIO3_ROP             0xCE
#define SSD1963_GET_GPIO3_ROP             0xCF
#define SSD1963_SET_DBC_CONF              0xD0
#define SSD1963_GET_DBC_CONF              0xD1
#define SSD1963_SET_DBC_TH                0xD4
#define SSD1963_GET_DBC_TH                0xD5
#define SSD1963_SET_PLL                   0xE0
#define SSD1963_SET_PLL_MN                0xE2
#define SSD1963_GET_PLL_MN                0xE3
#define SSD1963_GET_PLL_STATUS            0xE4
#define SSD1963_SET_DEEP_SLEEP            0xE5
#define SSD1963_SET_LSHIFT_FREQ           0xE6
#define SSD1963_GET_LSHIFT_FREQ           0xE7
#define SSD1963_SET_PIXEL_DATA_INTERFACE  0xF0
#define SSD1963_PDI_8BIT                  0
#define SSD1963_PDI_12BIT                 1
#define SSD1963_PDI_16BIT                 2
#define SSD1963_PDI_16BIT565              3
#define SSD1963_PDI_18BIT                 4
#define SSD1963_PDI_24BIT                 5
#define SSD1963_PDI_9BIT                  6
#define SSD1963_GET_PIXEL_DATA_INTERFACE  0xF1

#define czarny                            0x0000
#define bialy 							  						0xFFFF
#define zielony                           0x7E0
#define czerwony                          0xF800
#define niebieski                         0x1F
#define fioletowy                         0xF81F
#define szary                             0xC618
#define zolty                             0xFFE0
#define szarszy							  						0x8410
#define blekitny						  						0x0C58

#define ENTRY_MODE_DEFAULT 0x6830
#define ENTRY_MODE_BMP 	   0x6810
#define MAKE_ENTRY_MODE(x) ((ENTRY_MODE_DEFAULT & 0xFF00) | (x))

/** 
  * @brief The dimensions of the LCD panel.
  */
#define LCD_VERTICAL_MAX   480
#define LCD_HORIZONTAL_MAX 800

/** 
  * @brief Various definitions controlling coordinate space mapping and drawing
           direction in the four supported orientations.   
  */
#define PORTRAIT

#ifdef PORTRAIT
#define HORIZ_DIRECTION 0x28
#define VERT_DIRECTION 0x20
#define MAPPED_X(x, y) (799 - (y))
#define MAPPED_Y(x, y) (x)
#endif
#ifdef LANDSCAPE
#define HORIZ_DIRECTION 0x00
#define VERT_DIRECTION  0x08
#define MAPPED_X(x, y) (799 - (x))
#define MAPPED_Y(x, y) (439 - (y))
#endif
#ifdef PORTRAIT_FLIP
#define HORIZ_DIRECTION 0x18
#define VERT_DIRECTION 0x10
#define MAPPED_X(x, y) (y)
#define MAPPED_Y(x, y) (439 - (x))
#endif
#ifdef LANDSCAPE_FLIP
#define HORIZ_DIRECTION 0x30
#define VERT_DIRECTION  0x38
#define MAPPED_X(x, y) (x)
#define MAPPED_Y(x, y) (y)
#endif

/** @defgroup STM32F4_DISCOVERY_LCD_Exported_Constants
  * @{
  */ 

/**
 * @brief Uncomment the line below if you want to use user defined Delay function
 *        (for precise timing), otherwise default _delay_ function defined within
 *         this driver is used (less precise timing).  
 */
/* #define USE_Delay */

#ifdef USE_Delay
#include "main.h" 
  #define _delay_     Delay  /* !< User can provide more timing precise _delay_ function
                                   (with 10ms time base), using SysTick for example */
#else
  #define _delay_     delay      /* !< Default _delay_ function with less precise timing */
#endif

/** 
  * @brief  LCD color  
  */ 
#define LCD_COLOR_WHITE          0xFFFF
#define LCD_COLOR_BLACK          0x0000
#define LCD_COLOR_GREY           0xF7DE
#define LCD_COLOR_BLUE           0x001F
#define LCD_COLOR_BLUE2          0x051F
#define LCD_COLOR_RED            0xF800
#define LCD_COLOR_MAGENTA        0xF81F
#define LCD_COLOR_GREEN          0x07E0
#define LCD_COLOR_CYAN           0x7FFF
#define LCD_COLOR_YELLOW         0xFFE0

#define White		         LCD_COLOR_WHITE
#define Black		         LCD_COLOR_BLACK
#define Red		         LCD_COLOR_RED
#define Blue		         LCD_COLOR_BLUE
#define Green		         LCD_COLOR_GREEN
#define Cyan                     LCD_COLOR_CYAN

/** 
  * @brief  LCD Lines depending on the chosen fonts.  
  */
#define LCD_LINE_0               LINE(0)
#define LCD_LINE_1               LINE(1)
#define LCD_LINE_2               LINE(2)
#define LCD_LINE_3               LINE(3)
#define LCD_LINE_4               LINE(4)
#define LCD_LINE_5               LINE(5)
#define LCD_LINE_6               LINE(6)
#define LCD_LINE_7               LINE(7)
#define LCD_LINE_8               LINE(8)
#define LCD_LINE_9               LINE(9)
#define LCD_LINE_10              LINE(10)
#define LCD_LINE_11              LINE(11)
#define LCD_LINE_12              LINE(12)
#define LCD_LINE_13              LINE(13)
#define LCD_LINE_14              LINE(14)
#define LCD_LINE_15              LINE(15)
#define LCD_LINE_16              LINE(16)
#define LCD_LINE_17              LINE(17)
#define LCD_LINE_18              LINE(18)
#define LCD_LINE_19              LINE(19)
#define LCD_LINE_20              LINE(20)
#define LCD_LINE_21              LINE(21)
#define LCD_LINE_22              LINE(22)
#define LCD_LINE_23              LINE(23)
#define LCD_LINE_24              LINE(24)
#define LCD_LINE_25              LINE(25)
#define LCD_LINE_26              LINE(26)
#define LCD_LINE_27              LINE(27)
#define LCD_LINE_28              LINE(28)
#define LCD_LINE_29              LINE(29)

#define Line0                    LCD_LINE_0
#define Line1                    LCD_LINE_1
#define Line2                    LCD_LINE_2
#define Line3                    LCD_LINE_3
#define Line4                    LCD_LINE_4
#define Line5                    LCD_LINE_5
#define Line6                    LCD_LINE_6
#define Line7                    LCD_LINE_7
#define Line8                    LCD_LINE_8
#define Line9                    LCD_LINE_9

#define TFT_FPS 60ULL

#define TFT800480

#ifdef TFT800480
  #define TFT_WIDTH                   800ULL
  #define TFT_HSYNC_BACK_PORCH        40ULL
  #define TFT_HSYNC_FRONT_PORCH       40ULL
  #define TFT_HSYNC_PULSE             48ULL
        
  #define TFT_HEIGHT                  480ULL
  #define TFT_VSYNC_BACK_PORCH        29ULL
  #define TFT_VSYNC_FRONT_PORCH       13ULL
  #define TFT_VSYNC_PULSE             3ULL
#endif


#define TFT_HSYNC_PERIOD                    (TFT_HSYNC_PULSE + TFT_HSYNC_BACK_PORCH + TFT_WIDTH  + TFT_HSYNC_FRONT_PORCH)
#define TFT_VSYNC_PERIOD                    (TFT_VSYNC_PULSE + TFT_VSYNC_BACK_PORCH + TFT_HEIGHT + TFT_VSYNC_FRONT_PORCH)

#define TFT_PCLK                            (TFT_HSYNC_PERIOD * TFT_VSYNC_PERIOD * TFT_FPS)
//#define TFT_PCLK (928 * 525 * 60)
#define LCD_FPR                             ((TFT_PCLK * 1048576)/100000000)

#define mHIGH(x) (x >> 8)
#define mLOW(x) (x & 0xFF)

/** 
  * @brief LCD default font 
  */ 
#define LCD_DEFAULT_FONT         Font16x24

/** 
  * @brief  LCD Direction  
  */ 
#define LCD_DIR_HORIZONTAL       0x0000
#define LCD_DIR_VERTICAL         0x0001

/** 
  * @brief  LCD Size (Width and Height)  
  */ 
#define LCD_PIXEL_WIDTH          800
#define LCD_PIXEL_HEIGHT         480

/**
  * @}
  */ 

/** @defgroup STM32F4_DISCOVERY_LCD_Exported_Macros
  * @{
  */
#define ASSEMBLE_RGB(R ,G, B)    ((((R)& 0xF8) << 8) | (((G) & 0xFC) << 3) | (((B) & 0xF8) >> 3)) 
/**
  * @}
  */ 

/** @defgroup STM32F4_DISCOVERY_LCD_Exported_Functions
  * @{
  */ 
/** @defgroup  
  * @{
  */
void LCD_DeInit(void);   
void LCD_SSD1963_Init(void);
void LCD_RGB_Test(void);
void LCD_SetColors(__IO uint16_t _TextColor, __IO uint16_t _BackColor); 
void LCD_GetColors(__IO uint16_t *_TextColor, __IO uint16_t *_BackColor);
void LCD_SetTextColor(__IO uint16_t Color);
void LCD_SetBackColor(__IO uint16_t Color);
void LCD_ClearLine(uint16_t Line);
void LCD_Clear(uint16_t Color);
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos);
void SSD1963_SetArea(uint16_t sx, uint16_t ex, uint16_t sy, uint16_t ey);
void LCD_DrawChar(uint16_t Xpos, uint16_t Ypos, const uint16_t *c);
void LCD_DisplayChar(uint16_t Line, uint16_t Column, uint8_t Ascii);
void LCD_SetFont(sFONT *fonts);
sFONT *LCD_GetFont(void);
void LCD_DisplayStringLine(uint16_t Line, uint16_t Column, uint8_t *ptr);
void LCD_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Height, uint16_t Width);
void LCD_WindowModeDisable(void);
void LCD_DrawLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length, uint8_t Direction);
void LCD_DrawRect(uint16_t Xpos, uint16_t Ypos, uint8_t Height, uint16_t Width);
void LCD_DrawCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius);
void LCD_DrawMonoPict(const uint32_t *Pict);
void LCD_WriteBMP(uint32_t BmpAddress);
void LCD_DrawUniLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_DrawFullRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);
void LCD_DrawFullCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius);
void LCD_PolyLine(pPoint Points, uint16_t PointCount);
void LCD_PolyLineRelative(pPoint Points, uint16_t PointCount);
void LCD_ClosedPolyLine(pPoint Points, uint16_t PointCount);
void LCD_ClosedPolyLineRelative(pPoint Points, uint16_t PointCount);
void LCD_FillPolyLine(pPoint Points, uint16_t PointCount);
/**
  * @}
  */ 

/** @defgroup  
  * @{
  */ 
void LCD_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue);
uint16_t LCD_ReadReg(uint8_t LCD_Reg);
void LCD_WriteRAM_Prepare(void);
void LCD_WriteRAM(uint16_t RGB_Code);
uint16_t LCD_ReadRAM(void);
void LCD_PowerOn(void);
void LCD_DisplayOn(void);
void LCD_DisplayOff(void);
/**
  * @}
  */ 

/** @defgroup
  * @{
  */ 
void LCD_CtrlLinesConfig(void);
void LCD_FSMCConfig(void);
/**
  * @}
  */
/**
  * @}
  */    
#ifdef __cplusplus
}
#endif

#endif /* __STM32F4_DISCOVERY_LCD_H */
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

/*********** Portions COPYRIGHT 2012 Embest Tech. Co., Ltd.*****END OF FILE****/
