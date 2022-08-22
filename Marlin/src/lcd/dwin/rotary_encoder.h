#ifndef __ROTARY_ENCODER_H
#define __ROTARY_ENCODER_H

#include "../../inc/MarlinConfig.h"
#include "../../Marlin.h"

/*********************** Encoder Set ***********************/
#define ENCODER_PHASE_0  0
#define ENCODER_PHASE_1  2
#define ENCODER_PHASE_2  3
#define ENCODER_PHASE_3  1

#define ENCODER_PULSES_PER_STEP  4

#define BUTTON_PRESSED(BN) !READ(BTN_## BN)

typedef struct
{
  bool encoderRateEnabled = 0;
  int encoderMoveValue = 0;
  millis_t lastEncoderTime = 0;
}ENCODER_Rate;

extern ENCODER_Rate EncoderRate;

typedef enum
{ 
  ENCODER_DIFF_NO    = 0,
	ENCODER_DIFF_CW    = 1,
  ENCODER_DIFF_CCW   = 2,
	ENCODER_DIFF_ENTER = 3
}ENCODER_DiffState;

/*编码器初始化 PB12:Encoder_A PB13:Encoder_B PB14:Encoder_C*/
void Encoder_Configuration(void);

/*接收数据解析 返回值:ENCODER_DIFF_NO,无状态; ENCODER_DIFF_CW,顺时针旋转; ENCODER_DIFF_CCW,逆时针旋转; ENCODER_DIFF_ENTER,按下*/
ENCODER_DiffState Encoder_ReceiveAnalyze(void);


/*********************** Encoder LED ***********************/
#ifdef LCD_LED_PIN

  #define LED_NUM  4
  #define LED_DATA_HIGH  WRITE(LCD_LED_PIN, 1)
  #define LED_DATA_LOW   WRITE(LCD_LED_PIN, 0)

  #define RGB_SCAlE_R10_G7_B5  1     
  #define RGB_SCAlE_R10_G7_B4  2
  #define RGB_SCAlE_R10_G8_B7  3
  #define RGB_SCAlE_NEUTRAL_WHITE  RGB_SCAlE_R10_G7_B5 //正白
  #define RGB_SCAlE_WARM_WHITE  RGB_SCAlE_R10_G7_B4 //暖白
  #define RGB_SCAlE_COOL_WHITE  RGB_SCAlE_R10_G8_B7 //冷白

  extern unsigned int LED_DataArray[LED_NUM];

  /*状态LED初始化*/
  void STATE_LED_Configuration(void);

	/*LED灯操作*/
  void LED_Action(void);

  /*LED初始化*/
  void LED_Configuration(void);

  /*LED写数据*/
  void LED_WriteData(void);

  /*LED控制 RGB_Scale:RGB色彩配比 luminance:亮度(0~0xFF)*/
  void LED_Control(unsigned char RGB_Scale, unsigned char luminance);

  /*LED渐变控制 RGB_Scale:RGB色彩配比 luminance:亮度(0~0xFF) change_Time:渐变时间(ms)*/
  void LED_GraduallyControl(unsigned char RGB_Scale, unsigned char luminance, unsigned int change_Interval);

#endif

#endif
