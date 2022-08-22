#include "dwin_lcd.h"
#include "rotary_encoder.h"
#include "i2c_eeprom.h"

#ifdef DWIN_LCDDISPLAY

  /*********************************/
  #define TEXTBYTELEN     16
  #define MaxFileNumber   30

  /*print size*/
  #define X_MIN_PRINT_POS 0
  #define X_MAX_PRINT_POS 220
  #define Y_MIN_PRINT_POS 0
  #define Y_MAX_PRINT_POS 220

  /*fan speed limit*/
  #define	FanOn           255
  #define	FanOff          0

  /*print speed limit*/
  #define max_print_speed		999		
  #define min_print_speed 	10		

  /*Temp limit*/
  #define max_E_Temp  	(HEATER_0_MAXTEMP - 15)
  #define min_E_Temp 	  HEATER_0_MINTEMP		
  #define max_Bed_Temp	(BED_MAXTEMP	- 10)
  #define min_Bed_Temp 	BED_MINTEMP		

  /*Feedspeed limit*/  // max feedspeed = DEFAULT_MAX_FEEDRATE * 2
  #define min_MaxFeedspeed      1
  #define min_MaxAcceleration   1
  #define min_MaxCorner         0.1
  #define min_Step              1
  
	#define FEEDRATE_E      (60)

  // mininum unit (0.1) : multiple (10)
  #define MinUnitMult   10

  #define Encoder_wait    20
  #define	DWIN_UPDATE_INTERVAL 2000
  #define DWIN_REMAIN_TIME_UPDATE_INTERVAL 20000

  #define	FileNameLen	TEXTBYTELEN

  enum processID {
    /*Process ID*/
    ProcessFrame,
    PrintFile,
    Perpare,
    Control,		
    Leveing,				
    PrintProcess,
    AxisMove,
    Temperation,
    Motion,
    Info,
    Tune,
    PLAPreheat,
    ABSPreheat,
    MaxSpeed,
    MaxSpeed_value,
    MaxAcceleration,
    MaxAcceleration_value,
    MaxCorner,
    MaxCorner_value,
    Step,
    Step_value,

    /*Last Process ID*/
    Last_Perpare,

    /*Back Process ID*/
    Back_Main,
    Back_Print,

    /*Date variable ID*/
    Move_X,
    Move_Y,
    Move_Z,
    Exturder,
    Homeoffset,
    ETemp,
    BedTemp,
    FanSpeed,
    PrintSpeed,
    
    /*Window ID*/
    Print_window,
    Filament_window,
    Popup_Window
  };

  /*Picture ID*/
  #define Start_Process       0
  #define Language_English    1
  #define Language_Chinese    2

  /*ICON ID*/
  #define ICON                      0x09
  #define ICON_LOGO                 0
  #define ICON_Print_0              1
  #define ICON_Print_1              2
  #define ICON_Prepare_0            3
  #define ICON_Prepare_1            4
  #define ICON_Control_0            5
  #define ICON_Control_1            6
  #define ICON_Leveing_0            7
  #define ICON_Leveing_1            8
  #define ICON_HotendTemp           9
  #define ICON_BedTemp              10
  #define ICON_Speed                11
  #define ICON_Zoffest              12
  #define ICON_Back                 13
  #define ICON_File                 14
  #define ICON_PrintTime            15
  #define ICON_RemainTime           16
  #define ICON_Setup_0              17
  #define ICON_Setup_1              18
  #define ICON_Pause_0              19
  #define ICON_Pause_1              20
  #define ICON_Continue_0           21
  #define ICON_Continue_1           22
  #define ICON_Stop_0               23
  #define ICON_Stop_1               24
  #define ICON_Bar                  25
  #define ICON_More                 26
  #define ICON_Axis                 27
  #define ICON_CloseMotor           28
  #define ICON_Homeing              29
  #define ICON_SetHome              30
  #define ICON_PLAPreheat           31
  #define ICON_ABSPreheat           32
  #define ICON_Cool                 33
  #define ICON_Language             34
  #define ICON_MoveX                35
  #define ICON_MoveY                36
  #define ICON_MoveZ                37
  #define ICON_Extruder             38
  #define ICON_Temperation          40
  #define ICON_Motion               41
  #define ICON_WriteEEPROM          42
  #define ICON_ReadEEPROM           43
  #define ICON_ResumeEEPROM         44
  #define ICON_Info                 45
  #define ICON_SetEndTemp           46
  #define ICON_SetBedTemp           47
  #define ICON_FanSpeed             48
  #define ICON_SetPLAPreheat        49
  #define ICON_SetABSPreheat        50
  #define ICON_MaxSpeed             51
  #define ICON_MaxAccelerated       52
  #define ICON_MaxCorner            53
  #define ICON_Step                 54
  #define ICON_PrintSize            55
  #define ICON_Version              56
  #define ICON_Contact              57
  #define ICON_StockConfiguraton    58
  #define ICON_MaxSpeedX            59
  #define ICON_MaxSpeedY            60
  #define ICON_MaxSpeedZ            61
  #define ICON_MaxSpeedE            62
  #define ICON_MaxAccX              63
  #define ICON_MaxAccY              64
  #define ICON_MaxAccZ              65
  #define ICON_MaxAccE              66  
  #define ICON_MaxSpeedCornerX      67
  #define ICON_MaxSpeedCornerY      68
  #define ICON_MaxSpeedCornerZ      69
  #define ICON_MaxSpeedCornerE      70
  #define ICON_StepX                71
  #define ICON_StepY                72
  #define ICON_StepZ                73
  #define ICON_StepE                74
  #define ICON_Setspeed             75
  #define ICON_SetZOffest           76
  #define ICON_Rectangle            77
  #define ICON_BLTouch              78
  #define ICON_TempTooLow           79
  #define ICON_AutoLeveing          80
  #define ICON_TempTooHigh          81
  #define ICON_NoTips_C             82
  #define ICON_NoTips_E             83
  #define ICON_Continue_C           84
  #define ICON_Continue_E           85
  #define ICON_Cancel_C             86
  #define ICON_Cancel_E             87
  #define ICON_Confirm_C            88
  #define ICON_Confirm_E            89
  #define ICON_Info_0               90
  #define ICON_Info_1               91
  #define ICON_Celsius              92

  /*
  * 3-.0：字号大小，0x00-0x09，对应字体大小于下：
  * 0x00=6*12 	0x01=8*16 	0x02=10*20 	0x03=12*24 	0x04=14*28
  * 0x05=16*32 	0x06=20*40 	0x07=24*48 	0x08=28*56 	0x09=32*64
  */
  #define font6x12	0x00
  #define font8x16	0x01
  #define font10x20	0x02
  #define font12x24	0x03
  #define font14x28	0x04
  #define font16x32	0x05
  #define font20x40	0x06
  #define font24x48	0x07
  #define font28x56	0x08
  #define font32x64	0x09

  /* Colour */
  #define White             0xFFFF
  #define Background_window 0x31E8  // 弹窗背景色
  #define Background_blue   0x1125  // 暗蓝背景色
  #define Background_black  0x0841	// 黑色背景色
  #define Font_window       0xD6BA  // 弹窗字体背景色
  #define Line_Color        0x3A6A  // 分割线颜色
  #define Rectangle_Color   0x10E4  // 蓝色方块光标颜色
  #define Percent_Color     0xFE29  // 百分比颜色
  #define BarFill_Color     0x10E4  // 进度条填充色
  #define Select_Color      0x33BB  // 选中色

  extern int checkkey, last_checkkey;
  extern float zprobe_zoffset;
  extern char print_filename[16];

  extern bool yes_PLR_flag;

  extern millis_t heat_time;

  typedef struct
  {
    short int E_Temp          = 0;
    short int Bed_Temp        = 0;
    short int Fan_speed       = 0;
    short int print_speed     = 100;
    float Max_Feedspeed       = 0;
    float Max_Acceleration    = 0;
    float Max_Corner          = 0;
    float Max_Steip           = 0;
    float Move_X_scale        = 0;
    float Move_Y_scale        = 0;
    float Move_Z_scale        = 0;
    float Move_E_scale        = 0;
    float offest_value        = 0;
    char show_mode            = 0;    // -1: Temperation control    0: Printing temperation
    int16_t preheat_hotend_temp[2];
    int16_t preheat_bed_temp[2];  
    uint8_t preheat_fan_speed[2];
  }HMI_ValueTypeDef;

  typedef struct CardRecord
  {
    bool recovery_flag = 0;
    int  Filesum;
    char Cardshowfilename[MaxFileNumber][FileNameLen];
    char Cardfilename[MaxFileNumber][FileNameLen];
  }CRec;

  typedef struct
  {
    bool pause_flag       = 0;
    bool print_finish     = 0;
    bool confirm_flag     = 0;
    bool language_flag    = 0;  // 0: EN, 1: CN
    bool select_flag      = 0;
    bool home_flag        = 0;
    bool heat_flag        = 0;  // 0: heating done  1: during heating
    bool leveing_offest_flag  = 0;
    bool ETempTooLow_flag = 0;
    char feedspeed_flag   = 0;
    char acc_flag         = 0;
    char corner_flag      = 0;
    char step_flag        = 0;
  }HMI_Flag;

  extern HMI_ValueTypeDef HMI_ValueStruct;
  extern HMI_Flag         HMI_flag;

  /* Language */
  void lcd_select_language(void);
  void set_english_to_eeprom(void);
  void set_chinese_to_eeprom(void);

  /* Show ICON*/
  void ICON_Print(bool show);
  void ICON_Prepare(bool show);
  void ICON_Control(bool show);
  void ICON_Leveing(bool show);
  void ICON_StartInfo(bool show);

  void ICON_Setting(bool show);
  void ICON_Pause(bool show);
  void ICON_Continue(bool show);
  void ICON_Stop(bool show);

  /* Popup window tips */
  void Popup_Window_Temperation(char temp);
  void Popup_Window_ETempTooLow(void);
  void Popup_Window_Resume(void);
  void Popup_Window_Home(void);
  void Popup_Window_Leveing(void);
	void Popup_window_Filament(void);

  void Goto_PrintProcess(void);
  void Goto_ProcessFrame(void);

  /* Variable control */
  void HMI_Move_X(void);
  void HMI_Move_Y(void);
  void HMI_Move_Z(void);
  void HMI_Extruder(void);

  void HMI_Zoffset(void);
  void HMI_ETemp(void);
  void HMI_BedTemp(void);
  void HMI_FanSpeed(void);
  void HMI_PrintSpeed(void);

  void HMI_MaxFeedspeedXYZE(void);
  void HMI_MaxAccelerationXYZE(void);
  void HMI_MaxCornerXYZE(void);
  void HMI_StepXYZE(void);
  
  void update_variable(void);
  void show_plus_or_minus(unsigned char size, unsigned short int bColor, unsigned char iNum, unsigned char fNum, unsigned short int x, unsigned short int y, long variate);

  /* SD Card */
  void HMI_SDCardInit(void);
  void HMI_SDCardUpdate(void);

  /* Main Process */
  void Icon_print(bool value);
  void Icon_control(bool value);
  void Icon_temperation(bool value);
  void Icon_leveing(bool value);

  /* Other */
  bool Pause_HeatStatus();
  
  /*开机画面*/
  void HMI_StartFrame(void);

  /*主进程画面*/
  void HMI_ProcessFrame(void);

  /*文件页*/
  void HMI_Printfile(void);
  
  /*打印页*/
  void HMI_Printing(void);

  /*准备页*/
  void HMI_Perpare(void);

  /*控制页*/
  void HMI_Control(void);

  /*调平页*/
  void HMI_Leveing(void);

  /*轴移动菜单*/
  void HMI_AxisMove(void);

  /*温度菜单*/
  void HMI_Temperation(void);

  /*运动菜单*/
  void HMI_Motion(void);

  /*信息菜单*/
  void HMI_Info(void);

  /*调整菜单*/
  void HMI_Tune(void);

  /*PLA预热设置*/
  void HMI_PLAPreheatSetting(void);

  /*ABS预热设置*/
  void HMI_ABSPreheatSetting(void);

  /*最大速度子菜单*/
  void HMI_MaxSpeed(void);

  /*最大加速度子菜单*/
  void HMI_MaxAcceleration(void);

  /*最大拐角速度子菜单*/
  void HMI_MaxCorner(void);

  /*传动比*/
  void HMI_Step(void);


  void HMI_Init(void);
  void DWIN_Update(void);
  void Check_Filament_Update(void);
  void EachMomentUpdate(void);
  void DWIN_HandleDate(void);


#endif

