#ifndef RTS_H
#define RTS_H

#include "string.h"
#include <arduino.h>

#include "i2c_eeprom.h"

#define	MACVERSION		       STRING_CONFIG_H_AUTHOR
#define	SOFTVERSION		       SHORT_BUILD_VERSION
#define MAC_LENGTH           220
#define MAC_WIDTH            220
#define MAC_HEIGHT           250

/*********************************/
#define FHONE   (0x5A)
#define FHTWO   (0xA5)
#define FHLENG  (0x06)
#define TEXTBYTELEN     18
#define MaxFileNumber   20

#define AUTO_BED_LEVEL_PREHEAT  120

#define FileNum             MaxFileNumber
#define FileNameLen         TEXTBYTELEN
#define RTS_UPDATE_INTERVAL 2000
#define RTS_UPDATE_VALUE    RTS_UPDATE_INTERVAL

#define SizeofDatabuf       26

#define	FONT_EEPROM      0
#define Z_VALUE_EEPROM   500

#define Retry_num   2

/*************Register and Variable addr*****************/
#define RegAddr_W   0x80
#define RegAddr_R   0x81
#define VarAddr_W   0x82
#define VarAddr_R   0x83
#define ExchangePageBase    ((unsigned long)0x5A010000)     // the first page ID. other page = first page ID + relevant num;
#define StartSoundSet       ((unsigned long)0x060480A0)     // 06,start-music; 04, 4 musics; 80, the volume value; 04, return value about music number.

/*Error value*/
#define Error_201   "201 (Command Timeout)"   // The command too much inactive time
#define Error_202   "202 (Homing Failed)"     // Homing Failed
#define Error_203   "203 (Probing Failed)"    // Probing Failed
#define Error_204   "204 (Click Reboot)"     // SD Read Error

/*variable addr*/
#define ExchangepageAddr                0x0084
#define SoundAddr                       0x00A0
#define START_PROCESS_ICON_VP           0x1000
#define PRINT_SPEED_RATE_VP             0x1006
#define PRINT_PROCESS_TITLE_VP          0x100E
#define PRINT_TIME_HOUR_VP              0x1010
#define PRINT_TIME_MIN_VP               0x1012
#define PRINT_PROCESS_VP                0x1016
#define PRINTER_FANOPEN_TITLE_VP        0x101E
// #define PRINTER_LEDOPEN_TITLE_VP        0x101F
#define AUTO_BED_LEVEL_ZOFFSET_VP       0x1026
#define HEAD_SET_TEMP_VP                0x1034
#define HEAD_CURRENT_TEMP_VP            0x1036
#define BED_SET_TEMP_VP                 0x103A
#define BED_CURRENT_TEMP_VP             0x103C
#define AXIS_X_COORD_VP                 0x1048
#define AXIS_Y_COORD_VP                 0x104A
#define AXIS_Z_COORD_VP                 0x104C
#define HEAD_FILAMENT_LOAD_DATA_VP      0x1054
#define PRINTER_MACHINE_TEXT_VP         0x1060
#define PRINTER_VERSION_TEXT_VP         0x106A
#define PRINTER_PRINTSIZE_TEXT_VP       0x1074
#define PRINTER_WEBSITE_TEXT_VP         0x107E
#define AUTO_BED_PREHEAT_HEAD_DATA_VP   0x108A
#define AUTO_BED_LEVEL_TITLE_VP         0x108D
#define FILAMENT_LOAD_ICON_VP           0x108E
#define FAN_SPEED_CONTROL_DATA_VP       0x1100
#define PLA_HEAD_SET_DATA_VP            0x1102
#define PLA_BED_SET_DATA_VP             0x1104
#define PLA_FAN_SET_DATA_VP             0x1106
#define ABS_HEAD_SET_DATA_VP            0x1108
#define ABS_BED_SET_DATA_VP             0x110A
#define ABS_FAN_SET_DATA_VP             0x110C
#define ABNORMAL_TEXT_VP                0X1110
#define MOTOR_FREE_ICON_VP              0x1200
#define CONTINUE_PRINT_FILE_TEXT_VP     0x2000

/*file addr*/
#define FILE1_SELECT_ICON_VP            0x1221
#define FILE2_SELECT_ICON_VP            0x1222
#define FILE3_SELECT_ICON_VP            0x1223
#define FILE4_SELECT_ICON_VP            0x1224
#define FILE5_SELECT_ICON_VP            0x1225
#define FILE6_SELECT_ICON_VP            0x1226
#define FILE7_SELECT_ICON_VP            0x1227
#define FILE8_SELECT_ICON_VP            0x1228
#define FILE9_SELECT_ICON_VP            0x1229
#define FILE10_SELECT_ICON_VP           0x122A
#define FILE11_SELECT_ICON_VP           0x122B
#define FILE12_SELECT_ICON_VP           0x122C
#define FILE13_SELECT_ICON_VP           0x122D
#define FILE14_SELECT_ICON_VP           0x122E
#define FILE15_SELECT_ICON_VP           0x122F
#define FILE16_SELECT_ICON_VP           0x1230
#define FILE17_SELECT_ICON_VP           0x1231
#define FILE18_SELECT_ICON_VP           0x1232
#define FILE19_SELECT_ICON_VP           0x1233
#define FILE20_SELECT_ICON_VP           0x1234
#define FILE1_TEXT_VP                   0x200A
#define FILE2_TEXT_VP                   0x2014
#define FILE3_TEXT_VP                   0x201E
#define FILE4_TEXT_VP                   0x2028
#define FILE5_TEXT_VP                   0x2032
#define FILE6_TEXT_VP                   0x203C
#define FILE7_TEXT_VP                   0x2046
#define FILE8_TEXT_VP                   0x2050
#define FILE9_TEXT_VP                   0x205A
#define FILE10_TEXT_VP                  0x2064
#define FILE11_TEXT_VP                  0x206E
#define FILE12_TEXT_VP                  0x2078
#define FILE13_TEXT_VP                  0x2082
#define FILE14_TEXT_VP                  0x208C
#define FILE15_TEXT_VP                  0x2096
#define FILE16_TEXT_VP                  0x20A0
#define FILE17_TEXT_VP                  0x20AA
#define FILE18_TEXT_VP                  0x20B4
#define FILE19_TEXT_VP                  0x20BE
#define FILE20_TEXT_VP                  0x20C8
#define FilenameNature                  0x6003

/*Eight language addr*/
#define Home_VP                         0x1300
#define Home_print_VP                   0x1301
#define Home_prepare_VP                 0x1302
#define Home_control_VP                 0x1303
#define Home_level_VP                   0x1304
#define File_select_VP                  0x1305
// #define Back_VP                         0x1306
#define Printing_VP                     0x1307
#define Print_time_VP                   0x1308
#define Print_finish_VP                 0x1309
#define Print_setup_VP                  0x130A
#define Print_pause_VP                  0x130B
#define Print_stop_VP                   0x130C
#define Setup_VP                        0x130D
#define Setup_speed_VP                  0x130E
#define Setup_hotend_VP                 0x130F
#define Setup_bed_VP                    0x1310
#define Setup_Zoffset_VP                0x1311
#define Setup_fan_VP                    0x1312
// #define Setup_light_VP                  0x1313
#define Perpare_VP                      0x1314
#define Perpare_move_VP                 0x1315
#define Perpare_E_VP                    0x1316
#define Perpare_motor_VP                0x1317
#define Perpare_PLA_VP                  0x1318
#define Perpare_ABS_VP                  0x1319
#define Perpare_cool_VP                 0x131A
#define Move_VP                         0x131B
#define Move_unit_VP                    0x131C
#define Move_X_VP                       0x131D
#define Move_Y_VP                       0x131E
#define Move_Z_VP                       0x131F
#define Feed_Retrun_VP                  0x1320
#define Feed_VP                         0x1321
#define Return_VP                       0x1322
#define Control_VP                      0x1323
#define Control_temp_VP                 0x1324
#define Control_light_VP                0x1325
#define Control_language_VP             0x1326
#define Control_recover_VP              0x1327
#define Control_info_VP                 0x1328
#define Temp_VP                         0x1329
#define Temp_hotend_VP                  0x132A
#define Temp_bed_VP                     0x132B
#define Temp_fan_VP                     0x132C
#define Temp_PLA_VP                     0x132D
#define Temp_ABS_VP                     0x132E
#define PLA_VP                          0x132F
#define PLA_hotend_VP                   0x1330
#define PLA_bed_VP                      0x1331
#define PLA_save_VP                     0x1332
#define ABS_VP                          0x1333
#define ABS_hotend_VP                   0x1334
#define ABS_bed_VP                      0x1335
#define ABS_save_VP                     0x1336
#define Info_VP                         0x1337
#define Info_size_VP                    0x1338
#define Info_version_VP                 0x1339
#define Info_call_VP                    0x133A
#define Level_VP                        0x133B
#define Level_Zoffset_VP                0x133C
#define Level_mode_VP                   0x133D
#define Level_auto_VP                   0x133E
#define Window_Filament_UseUp_VP        0x133F
#define Window_HeatUp_VP                0x1340
#define Window_StopPrint_VP             0x1341
#define Window_Filament_load_VP         0x1342
#define Window_Print_VP                 0x1343
#define Window_pause_VP                 0x1344
#define Window_confirm_VP               0x1345
#define Window_cancel_VP                0x1346
#define Window_stop_VP                  0x1347
#define Window_recover_VP               0x1348
#define Window_home_VP                  0x1349
#define Eight_language_VP               0x134A
#define Leve_heat_VP                    0x134B
#define No_leveling_VP                  0x134C

/************struct**************/
typedef struct DataBuf
{
    unsigned char len;  
    unsigned char head[2];
    unsigned char command;
    unsigned long addr;
    unsigned long bytelen;
    unsigned short data[32];
    unsigned char reserv[4];
} DB;

typedef struct CardRecord
{
    int recordcount;
    int Filesum;
    unsigned long addr[FileNum];
    char Cardshowfilename[FileNum][FileNameLen];
    char Cardfilename[FileNum][FileNameLen];
}CRec;

extern CRec CardRecbuf;

class RTSSHOW {
  public:
    RTSSHOW();
    int RTS_RecData();
    void RTS_SDCardInit(void);
    void RTS_SDCardUpate(void);
    void RTS_SndData(void);
    void RTS_SndData(const String &, unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(const char[], unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(char, unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(unsigned char*, unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(int, unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(float, unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(unsigned int,unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(long,unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(unsigned long,unsigned long, unsigned char = VarAddr_W);
    void RTS_SDcard_Stop();
    void RTS_HandleData();
    void RTS_Init();
    
    DB recdat;
    DB snddat;
  private:
    unsigned char databuf[SizeofDatabuf];
  };

extern RTSSHOW rtscheck;

enum PROC_COM 
{
  MainEnterKey          = 0,
  AdjustEnterKey        = 1,
  PrintSpeedEnterKey    = 2,
  StopPrintKey          = 3,
  PausePrintKey         = 4,
  ResumePrintKey        = 5,
  ZoffsetEnterKey       = 6,
  TempControlKey        = 7,
  CoolDownKey           = 8,
  HeaterTempEnterKey    = 9,
  HotBedTempEnterKey    = 10,
  PrepareEnterKey       = 11,
  BedLevelKey           = 12,
  AutoHomeKey           = 13,
  XaxismoveKey          = 14,
  YaxismoveKey          = 15,
  ZaxismoveKey          = 16,
  HeaterLoadEnterKey    = 17,
  HeaterLoadStartKey    = 18,
  SelectLanguageKey     = 19,
  PowerContinuePrintKey = 20,
  FanSpeedEnterKey      = 21,
  PLAHeadSetEnterKey    = 22,
  PLABedSetEnterKey     = 23,
  PLAFanSetEnterKey     = 24,
  ABSHeadSetEnterKey    = 25,
  ABSBedSetEnterKey     = 26,
  ABSFanSetEnterKey     = 27,
  ChangePageKey         = 28,
  StartFileKey          = 29,
  SelectFileKey         = 30,
  ErrorKey              = 31,
  EightLanguageKey      = 32,
};

const unsigned long Addrbuf[] = 
{
  0x1002,  0x1004,  0x1006,  0x1008,  0x100A,
  0x100C,  0x1026,  0x1030,  0x1032,  0x1034,
  0x103A,  0x103E,  0x1044,  0x1046,  0x1048,
  0x104A,  0x104C,  0x1054,  0x1056,  0x105C,
  0x105F,  0x1100,  0x1102,  0x1104,  0x1106,
  0x1108,  0x110A,  0x110C,  0x110E,  0x20D2,  
  0x20D3,  0x111A,  0x111C
};

typedef struct
{
  int16_t preheat_hotend_temp[2];
  int16_t preheat_bed_temp[2];  
  uint8_t preheat_fan_speed[2];
}HMI_ValueTypeDef;

extern HMI_ValueTypeDef HMI_ValueStruct;

void ErrorHanding();

extern void RTSUpdate();

extern unsigned int language_change_font;

extern float zprobe_zoffset;

extern char waitway;
extern int change_page_font;
extern int Update_Time_Value;
extern unsigned char AxisUnitMode;
extern bool home_flag;
extern bool G29_flag;
extern bool heat_flag;
extern bool print_finish;
extern bool finish_home;
extern bool AutohomeZflag;
extern char commandbuf[30];

extern bool StartPrint_flag;
extern bool pause_action_flag;

extern char errorway;
extern char errornum;
extern char error_sd_num;

#endif// RTS_H
