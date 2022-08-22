#include "dwin.h"

#include <wstring.h>
#include <stdio.h>
#include <string.h>
#include <Arduino.h>

#include "../../Marlin.h"
#include "../../inc/MarlinConfig.h"
#include "../../module/configuration_store.h"
#include "../../core/serial.h"
#include "../../core/macros.h"

#include "../fontutils.h"
#include "../../sd/cardreader.h"
#include "../../feature/power_loss_recovery.h"
#include "../../feature/babystep.h"
#include "../../module/temperature.h"
#include "../../module/printcounter.h"
#include "../../module/motion.h"
#include "../../module/planner.h"
#include "../../gcode/queue.h"
#include "../../gcode/gcode.h"

#include "../../../Version.h"

#if HAS_LEVELING
	#include "../../feature/bedlevel/bedlevel.h"
#endif

#include "../../libs/buzzer.h"

#ifdef DWIN_LCDDISPLAY

	#define PAUSE_HEAT 		true
  #define CHECKFILAMENT flase

	CRec Cardbuf;

	/* Value Init */
	HMI_ValueTypeDef HMI_ValueStruct;
	HMI_Flag HMI_flag;

	millis_t Encoder_ms = 0;
	millis_t Wait_ms = 0;
	millis_t next_rts_update_ms = 0;
	millis_t next_remain_time_update = 0;

	int checkkey = 0, last_checkkey = 0;

	millis_t heat_time = 0;

	unsigned char select_page = 0, last_select_page = 0;
	unsigned char select_file = 0, last_select_file = 0, index_file = 5;
	unsigned char select_print = 0, last_select_print = 0;
	unsigned char select_perpare = 0, last_select_perpare = 0, index_perpare = 5;
	unsigned char select_control = 0, last_select_control = 0, index_control = 5;
	unsigned char select_leveing = 0, last_select_leveing = 0, index_leveing = 5;

	unsigned char select_axis = 0, last_select_axis = 0;
	unsigned char select_temp = 0, last_select_temp = 0;
	unsigned char select_motion = 0, last_select_motion = 0;
	unsigned char select_tune = 0, last_select_tune = 0, index_tune = 5;

	unsigned char select_PLA = 0, last_select_PLA = 0;
	unsigned char select_ABS = 0, last_select_ABS = 0;
	unsigned char select_speed = 0, last_select_speed = 0;
	unsigned char select_acc = 0, last_select_acc = 0;
	unsigned char select_corner = 0, last_select_corner = 0;
	unsigned char select_step = 0, last_select_step = 0;

	uint16_t fileCnt = 0;
	uint8_t __fileCnt = 0;
	char filebuf[50];

	uint8_t countbuf = 0;

	bool abort_flag = 0;
	bool yes_PLR_flag = 0;

	constexpr float default_max_feedrate[] 				= DEFAULT_MAX_FEEDRATE;
	constexpr float default_max_acceleration[] 		= DEFAULT_MAX_ACCELERATION;
	constexpr float default_max_jerk[] 						= {DEFAULT_XJERK, DEFAULT_YJERK, DEFAULT_ZJERK, DEFAULT_EJERK};
	constexpr float default_axis_steps_per_unit[] = DEFAULT_AXIS_STEPS_PER_UNIT;

	unsigned char Percentrecord = 0;
	unsigned int last_Printtime = 0, remain_time = 0;
	float last_temp_hotend_target = 0, last_temp_bed_target = 0;
	float last_temp_hotend_current = 0, last_temp_bed_current = 0;
	uint8_t last_fan_speed = 0;
	unsigned short last_speed = 0;

	float last_E_scale = 0;

	bool DWIN_lcd_sd_status = 0;

	bool pause_action_flag = 0;

	int temphot = 0;
	int tempbed = 0;

	float zprobe_zoffset = 0.00;
	float last_zoffset = 0.00, last_probe_zoffset = 0.00;

	#define FONT_EEPROM_OFFSET 0
	#define DEFAULT_LANGUAGE 0	// 0: EN, 1: CN
	bool first_load_language = 0;
	void lcd_select_language(void)
	{
		BL24CXX_Read(FONT_EEPROM_OFFSET+2, (uint8_t*)&first_load_language, sizeof(first_load_language));
		delay(10);
		if(first_load_language == 0)
		{
			BL24CXX_Read(FONT_EEPROM_OFFSET, (uint8_t*)&HMI_flag.language_flag, sizeof(HMI_flag.language_flag));
			if(HMI_flag.language_flag) DWIN_JPG_CacheTo1(Language_Chinese);
			else DWIN_JPG_CacheTo1(Language_English);
		}
		else
		{
			first_load_language = 0;
			BL24CXX_Write(FONT_EEPROM_OFFSET+2, (uint8_t*)&first_load_language, sizeof(first_load_language));
			#if ENABLED(DEFAULT_LANGUAGE)
				set_chinese_to_eeprom();
			#else
				set_english_to_eeprom();
			#endif
		}
	}

	void set_english_to_eeprom(void)
	{
		HMI_flag.language_flag = 0;
		DWIN_JPG_CacheTo1(Language_English);
		BL24CXX_Write(FONT_EEPROM_OFFSET, (uint8_t*)&HMI_flag.language_flag, sizeof(HMI_flag.language_flag));
	}
	void set_chinese_to_eeprom(void)
	{
		HMI_flag.language_flag = 1;
		DWIN_JPG_CacheTo1(Language_Chinese);
		BL24CXX_Write(FONT_EEPROM_OFFSET, (uint8_t*)&HMI_flag.language_flag, sizeof(HMI_flag.language_flag));
	}

	void show_plus_or_minus(unsigned char size, unsigned short int bColor, unsigned char iNum, unsigned char fNum, unsigned short int x, unsigned short int y, long variate)
	{
		if(variate < 0) 
		{
			DWIN_Draw_String(false,true,size,White,bColor, x-6, y, (char*)"-");
			DWIN_Draw_FloatVariate(true,true,0,size,White,bColor, iNum, fNum, x, y, -variate);
		}
		else
		{
			DWIN_Draw_String(false,true,size,White,bColor, x-6, y, (char*)" ");
			DWIN_Draw_FloatVariate(true,true,0,size,White,bColor, iNum, fNum, x, y, variate);
		}
	}

	void ICON_Print(bool show)
	{
		if(show)
		{
			DWIN_ICON_Show(ICON,ICON_Print_1, 17, 130);	
			if(HMI_flag.language_flag) 
				DWIN_Frame_AreaCopy(1, 1, 447, 271-243, 479-19, 58, 201);
			else 
				DWIN_Frame_AreaCopy(1, 1,	451, 271-240,	479-16, 72-15, 201);
		}
		else
		{
			DWIN_ICON_Show(ICON,ICON_Print_0, 17, 130);
			if(HMI_flag.language_flag) 
				DWIN_Frame_AreaCopy(1, 1, 405, 271-243, 420, 58, 201);
			else 
				DWIN_Frame_AreaCopy(1, 1,	424, 271-240,	479-45, 72-15, 201);
		}
	}

	void ICON_Prepare(bool show)
	{
		if(show)
		{
			DWIN_ICON_Show(ICON,ICON_Prepare_1, 145, 130);
			if(HMI_flag.language_flag) 
				DWIN_Frame_AreaCopy(1, 31, 447, 271-213, 479-19, 186, 201);
			else 
				DWIN_Frame_AreaCopy(1, 33, 451, 271-189,	479-13, 200-25, 201);
		}
		else
		{
			DWIN_ICON_Show(ICON,ICON_Prepare_0, 145, 130);
			if(HMI_flag.language_flag) 
				DWIN_Frame_AreaCopy(1, 31, 405, 271-213, 420, 186, 201);
			else 
				DWIN_Frame_AreaCopy(1, 33, 424, 271-189, 479-42, 200-25, 201);
		}
	}

	void ICON_Control(bool show)
	{
		if(show)
		{
			DWIN_ICON_Show(ICON,ICON_Control_1, 17, 246);
			if(HMI_flag.language_flag) 
				DWIN_Frame_AreaCopy(1, 61, 447, 271-183, 479-19, 58, 318);
			else 
				DWIN_Frame_AreaCopy(1, 85, 451, 271-139,	479-16, 72-24, 318);
		}
		else
		{
			DWIN_ICON_Show(ICON,ICON_Control_0, 17, 246);
			if(HMI_flag.language_flag) 
				DWIN_Frame_AreaCopy(1, 61, 405, 271-183, 420, 58, 318);
			else 
				DWIN_Frame_AreaCopy(1, 85, 424, 271-139, 479-45, 72-24, 318);
		}
	}

	void ICON_Leveing(bool show)
	{
		if(show)
		{
			DWIN_ICON_Show(ICON,ICON_Leveing_1, 145, 246);
			if(HMI_flag.language_flag) 
				DWIN_Frame_AreaCopy(1, 211, 447, 238, 479-19, 186, 318);
			else 
				DWIN_Frame_AreaCopy(1, 84, 437, 120,	449, 200-18, 318);
		}
		else
		{
			DWIN_ICON_Show(ICON,ICON_Leveing_0, 145, 246);
			if(HMI_flag.language_flag) 
				DWIN_Frame_AreaCopy(1, 211, 405, 238, 420, 186, 318);
			else 
				DWIN_Frame_AreaCopy(1, 84, 465, 120, 478, 200-18, 318);
		}
	}

	void ICON_StartInfo(bool show)
	{
		if(show)
		{
			DWIN_ICON_Show(ICON,ICON_Info_1, 145, 246);
			if(HMI_flag.language_flag) 
				DWIN_Frame_AreaCopy(1, 91, 447, 271-153, 479-19, 186, 318);
			else 
				DWIN_Frame_AreaCopy(1, 132, 451, 159, 479-13, 186, 318);
		}
		else
		{
			DWIN_ICON_Show(ICON,ICON_Info_0, 145, 246);
			if(HMI_flag.language_flag) 
				DWIN_Frame_AreaCopy(1, 91, 405, 271-153, 420, 186, 318);
			else 
				DWIN_Frame_AreaCopy(1, 132, 424, 159, 479-42, 186, 318);
		}
	}

  void ICON_Setting(bool show)
	{
		if(show)
		{
			DWIN_ICON_Show(ICON,ICON_Setup_1, 8, 252);
			if(HMI_flag.language_flag) 
				DWIN_Frame_AreaCopy(1, 121, 447, 271-123, 479-21, 34, 325);
			else 
				DWIN_Frame_AreaCopy(1, 2, 465, 271-238,	479-2, 48-17, 325);
		}
		else
		{
			DWIN_ICON_Show(ICON,ICON_Setup_0, 8, 252);
			if(HMI_flag.language_flag) 
				DWIN_Frame_AreaCopy(1, 121, 405, 271-123, 420, 34, 325);
			else 
				DWIN_Frame_AreaCopy(1, 1, 438, 271-239, 479-31, 48-17, 325);
		}
	}

  void ICON_Pause(bool show)
	{
		if(show)
		{
			DWIN_ICON_Show(ICON,ICON_Pause_1, 96, 252);
			if(HMI_flag.language_flag) 
				DWIN_Frame_AreaCopy(1, 181, 447, 271-63, 479-20, 124, 325);
			else 
				DWIN_Frame_AreaCopy(1, 177, 451, 271-55,	479-17, 136-20, 325);
		}
		else
		{
			DWIN_ICON_Show(ICON,ICON_Pause_0, 96, 252);
			if(HMI_flag.language_flag) 
				DWIN_Frame_AreaCopy(1, 181, 405, 271-63, 420, 124, 325);
			else 
				DWIN_Frame_AreaCopy(1, 177, 423, 271-56, 479-46, 136-20, 325);
		}
	}

  void ICON_Continue(bool show)
	{
		if(show)
		{
			DWIN_ICON_Show(ICON,ICON_Continue_1, 96, 252);	
			if(HMI_flag.language_flag) 
				DWIN_Frame_AreaCopy(1, 1, 447, 271-243, 479-19, 124, 325);
			else 
				DWIN_Frame_AreaCopy(1, 1,	451, 271-240,	479-16, 136-15, 325);
		}
		else
		{
			DWIN_ICON_Show(ICON,ICON_Continue_0, 96, 252);
			if(HMI_flag.language_flag) 
				DWIN_Frame_AreaCopy(1, 1, 405, 271-243, 420, 124, 325);
			else 
				DWIN_Frame_AreaCopy(1, 1,	424, 271-240,	479-45, 136-15, 325);
		}
	}
	
  void ICON_Stop(bool show)
	{
		if(show)
		{
			DWIN_ICON_Show(ICON,ICON_Stop_1, 184, 252);
			if(HMI_flag.language_flag) 
				DWIN_Frame_AreaCopy(1, 151, 447, 271-93, 479-20, 210, 325);
			else 
				DWIN_Frame_AreaCopy(1, 219, 451, 271-23,	479-14, 224-15, 325);
		}
		else
		{
			DWIN_ICON_Show(ICON,ICON_Stop_0, 184, 252);
			if(HMI_flag.language_flag) 
				DWIN_Frame_AreaCopy(1, 151, 405, 271-93, 420, 210, 325);
			else 
				DWIN_Frame_AreaCopy(1, 218, 423, 271-24, 479-43, 224-15, 325);
		}
	}

  void Popup_Window_Temperation(char temp)
	{
		DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
		DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	480);
		DWIN_Draw_Rectangle(1, Background_window, 14, 105, 271-13, 479-105);
		if(temp == 2)
		{
			DWIN_Draw_String(false,true,font8x16, Font_window, Background_window, 96, 165, (char*)"WARNNINGS!");
			DWIN_Draw_String(false,true,font8x16, Font_window, Background_window, 72, 250, (char*)"THERMAL RUNAWAY!");
		}
		else if(temp == 1)
		{
			DWIN_ICON_Show(ICON, ICON_TempTooHigh, 102, 165);
			if(HMI_flag.language_flag)
			{
				DWIN_Frame_AreaCopy(1, 103, 371, 237, 479-93, 52, 285);
				DWIN_Frame_AreaCopy(1, 151, 389, 185, 402, 187, 285);
				DWIN_Frame_AreaCopy(1, 189, 389, 271-0, 402, 95, 310);
			}
			else
			{
				DWIN_Draw_String(false,true,font8x16, Font_window, Background_window, 36, 290, (char*)"Nozzle or bed temperature");
				DWIN_Draw_String(false,true,font8x16, Font_window, Background_window, 92, 310, (char*)"is too high");
			}
		}
		else if(temp == 0)
		{
			DWIN_ICON_Show(ICON, ICON_TempTooLow, 102, 165);
			if(HMI_flag.language_flag)
			{
				DWIN_Frame_AreaCopy(1, 103, 371, 271-1, 479-93, 52, 285);
				DWIN_Frame_AreaCopy(1, 189, 389, 271-0, 402, 95, 310);
			}
			else
			{
				DWIN_Draw_String(false,true,font8x16, Font_window, Background_window, 36, 290, (char*)"Nozzle or bed temperature");
				DWIN_Draw_String(false,true,font8x16, Font_window, Background_window, 96, 310, (char*)"is too low");
			}
		}
		DWIN_UpdateLCD();
	}

  void Popup_Window_ETempTooLow(void)
	{
		DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
		DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);
		DWIN_Draw_Rectangle(1, Background_window, 14, 60, 271-13, 330);
		DWIN_ICON_Show(ICON, ICON_TempTooLow, 102, 105);
		if(HMI_flag.language_flag)
		{
			DWIN_Frame_AreaCopy(1, 103, 371, 136, 479-93, 69, 240);
			DWIN_Frame_AreaCopy(1, 170, 371, 271-1, 479-93, 69+33, 240);
			DWIN_ICON_Show(ICON, ICON_Confirm_C, 86, 280);
		}
		else
		{
			DWIN_Draw_String(false,true,font8x16, Font_window, Background_window, 20, 235, (char*)"Nozzle temperature is too low");
			DWIN_ICON_Show(ICON, ICON_Confirm_E, 86, 280);
		}
	}

  void Popup_Window_Resume(void)
	{
		DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
		DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	480);
		DWIN_Draw_Rectangle(1, Background_window, 14, 105, 271-13, 479-105);
		if(HMI_flag.language_flag)
		{
			DWIN_Frame_AreaCopy(1, 160, 338, 271-36, 479-125, 98, 135);
			DWIN_Frame_AreaCopy(1, 103, 321, 271-0, 479-144, 52, 192);
			DWIN_ICON_Show(ICON, ICON_Continue_C, 26, 307);
			DWIN_ICON_Show(ICON, ICON_Cancel_C, 146, 307);
		}
		else
		{
			DWIN_Draw_String(false,true,font8x16, Font_window, Background_window, 120, 135, (char*)"Tips");
			DWIN_Draw_String(false,true,font8x16, Font_window, Background_window, 24, 192, (char*)"Detect the file unexpectedly");
			DWIN_Draw_String(false,true,font8x16, Font_window, Background_window, 68, 212, (char*)"stopped last time");
			DWIN_ICON_Show(ICON, ICON_Continue_E, 26, 307);
			DWIN_ICON_Show(ICON, ICON_Cancel_E, 146, 307);
		}
	}

  void Popup_Window_Home(void)
	{
		DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
		DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);
		DWIN_Draw_Rectangle(1, Background_window, 14, 60, 271-13, 330);
		DWIN_ICON_Show(ICON, ICON_BLTouch, 101, 105);
		if(HMI_flag.language_flag)
		{
			DWIN_Frame_AreaCopy(1, 0, 371, 33, 386, 85, 240);
			DWIN_Frame_AreaCopy(1, 203, 286, 271, 302, 118, 240);
			DWIN_Frame_AreaCopy(1, 0, 389, 150, 402, 61, 280);
		}
		else
		{
			DWIN_Draw_String(false,true,font8x16, Font_window, Background_window, 80, 230, (char*)"Auto homing...");
			DWIN_Draw_String(false,true,font8x16, Font_window, Background_window, 24, 260, (char*)"Please don't other operation");
		}
	}

  void Popup_Window_Leveing(void)
	{
		DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
		DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);
		DWIN_Draw_Rectangle(1, Background_window, 14, 60, 271-13, 330);
		DWIN_ICON_Show(ICON, ICON_AutoLeveing, 101, 105);
		if(HMI_flag.language_flag)
		{
			DWIN_Frame_AreaCopy(1, 0, 371, 100, 386, 84, 240);
			DWIN_Frame_AreaCopy(1, 0, 389, 150, 402, 61, 280);
		}
		else
		{
			DWIN_Draw_String(false,true,font8x16, Font_window, Background_window, 76, 230, (char*)"Auto leveling...");
			DWIN_Draw_String(false,true,font8x16, Font_window, Background_window, 24, 260, (char*)"Please don't other operation");
		}
	}

	void Popup_wiundow_PauseOrStop(void)
	{
		DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
		DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);
		DWIN_Draw_Rectangle(1, Background_window, 14, 60, 271-13, 330);
		if(HMI_flag.language_flag)
		{
			if(select_print == 1) DWIN_Frame_AreaCopy(1, 237, 338, 269, 356, 98, 150);
			else if(select_print == 2) DWIN_Frame_AreaCopy(1, 221, 320, 253, 336, 98, 150);
			DWIN_Frame_AreaCopy(1, 220, 304, 264, 319, 130, 150);
			DWIN_ICON_Show(ICON, ICON_Confirm_C, 26, 280);
			DWIN_ICON_Show(ICON, ICON_Cancel_C, 146, 280);
		}
		else
		{
			if(select_print == 1) DWIN_Draw_String(false,true,font8x16, Font_window, Background_window, 88, 150, (char*)"Pause print?");
			else if(select_print == 2) DWIN_Draw_String(false,true,font8x16, Font_window, Background_window, 92, 150, (char*)"Stop print?");
			DWIN_ICON_Show(ICON, ICON_Confirm_E, 26, 280);
			DWIN_ICON_Show(ICON, ICON_Cancel_E, 146, 280);
		}
		DWIN_Draw_Rectangle(0, Background_window, 145, 279, 246, 318);
		DWIN_Draw_Rectangle(0, Background_window, 144, 278, 247, 319);
		DWIN_Draw_Rectangle(0, Select_Color, 25, 279, 126, 318);
		DWIN_Draw_Rectangle(0, Select_Color, 24, 278, 127, 319);
	}

	void Popup_window_Filament(void)
	{
		DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
		DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);
		DWIN_Draw_Rectangle(1, Background_window, 14, 60, 271-13, 330);
		if(HMI_flag.language_flag)
		{
			DWIN_Frame_AreaCopy(1, 160, 338, 271-36, 479-125, 98, 100);
			DWIN_Frame_AreaCopy(1, 0, 286, 204, 302, 34, 160);
			DWIN_Frame_AreaCopy(1, 0, 303, 159, 319, 56, 180);
			DWIN_Frame_AreaCopy(1, 169, 303, 185, 318, 95, 200);
			DWIN_Frame_AreaCopy(1, 33, 321, 99, 335, 111, 200);
			DWIN_ICON_Show(ICON, ICON_Confirm_C, 26, 280);
			DWIN_ICON_Show(ICON, ICON_NoTips_C, 146, 280);
		}
		else
		{
			DWIN_Draw_String(false,true,font8x16, Font_window, Background_window, 120, 100, (char*)"Tips");
			DWIN_Draw_String(false,true,font8x16, Font_window, Background_window, 32, 150, (char*)"Filement has been used up,");
			DWIN_Draw_String(false,true,font8x16, Font_window, Background_window, 20, 170, (char*)"please replacce the filement,");
			DWIN_Draw_String(false,true,font8x16, Font_window, Background_window, 20, 190, (char*)"click confirm after finishing");
			DWIN_Draw_String(false,true,font8x16, Font_window, Background_window, 28, 210, (char*)"replacement, or not prompt.");
			DWIN_ICON_Show(ICON, ICON_Confirm_E, 26, 280);
			DWIN_ICON_Show(ICON, ICON_NoTips_E, 146, 280);
		}
		DWIN_Draw_Rectangle(0, Background_window, 145, 279, 246, 318);
		DWIN_Draw_Rectangle(0, Background_window, 144, 278, 247, 319);
		DWIN_Draw_Rectangle(0, Select_Color, 25, 279, 126, 318);
		DWIN_Draw_Rectangle(0, Select_Color, 24, 278, 127, 319);
	}

	void Goto_PrintProcess(void)
	{
		checkkey = PrintProcess;

		DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
		DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);

		if(HMI_flag.language_flag) 
		{
			DWIN_Frame_AreaCopy(1, 30, 1, 271-200, 479-465, 14, 9);
			DWIN_Frame_AreaCopy(1, 0, 72, 271-208, 479-393, 41, 188);
			DWIN_Frame_AreaCopy(1, 65, 72, 271-143, 479-393, 176, 188);
		}
		else 
		{
			DWIN_Frame_AreaCopy(1, 40, 2, 271-179, 479-464-1, 14, 9);
			DWIN_Frame_AreaCopy(1, 0, 44, 271-175, 479-420-1, 41, 188);
			DWIN_Frame_AreaCopy(1, 98, 44, 271-119, 479-420-1, 176, 188);
		}

		if(select_print == 0) ICON_Setting(1);
		else ICON_Setting(0);
		if(select_print == 1) {
			if(printingIsPaused()) ICON_Continue(1);
			else ICON_Pause(1);
		}
		else {
			if(printingIsPaused()) ICON_Continue(0);
			else ICON_Pause(0);
		}
		if(select_print == 2) ICON_Stop(1);
		else ICON_Stop(0);

		DWIN_Draw_String(false,false,font8x16,White,Background_black, 136-(strlen(filebuf)*8)/2, 60, filebuf);	// filename
		DWIN_ICON_Show(ICON,ICON_Bar, 15, 93);
		DWIN_Draw_Rectangle(1, BarFill_Color,	16+Percentrecord*240/100,	93,	256, 113);
		DWIN_Draw_IntVariate(true,true,0,font8x16,Percent_Color,Background_black, 2, 117, 133, card.percentDone());
		DWIN_Draw_String(false,false,font8x16,Percent_Color,Background_black, 117+16, 133, (char*)"%");

		duration_t elapsed = print_job_timer.duration();	// print timer
		DWIN_ICON_Show(ICON,ICON_PrintTime, 17, 193);
		DWIN_ICON_Show(ICON,ICON_RemainTime, 150, 191);
		DWIN_Draw_IntVariate(true,true,1,font8x16,White,Background_black, 2, 42, 212, elapsed.value/3600);
		DWIN_Draw_String(false,false,font8x16,White,Background_black, 42+16, 212, (char*)":");	
		DWIN_Draw_IntVariate(true,true,1,font8x16,White,Background_black, 2, 42+24, 212, (elapsed.value%3600)/60);
		DWIN_Draw_IntVariate(true,true,1,font8x16,White,Background_black, 2, 176, 212, remain_time/3600);
		DWIN_Draw_String(false,false,font8x16,White,Background_black, 176+16, 212, (char*)":");	
		DWIN_Draw_IntVariate(true,true,1,font8x16,White,Background_black, 2, 176+24, 212, (remain_time%3600)/60);
	}

	void Goto_ProcessFrame(void)
	{
		checkkey = ProcessFrame;
		
		DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
		DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);

		if(HMI_flag.language_flag) 
			DWIN_Frame_AreaCopy(1, 2, 2, 271-244, 479-465, 14, 9);
		else 
			DWIN_Frame_AreaCopy(1, 0,	2, 271-232,	479-467, 14, 9);

		DWIN_ICON_Show(ICON,ICON_LOGO, 71, 52);

		if(select_page == 0) ICON_Print(1);
		else ICON_Print(0);
		if(select_page == 1) ICON_Prepare(1);
		else ICON_Prepare(0);
		if(select_page == 2) ICON_Control(1);
		else ICON_Control(0);
		if(select_page == 3) {
			#if HAS_LEVELING
				ICON_Leveing(1);
			#else
				ICON_StartInfo(1);
			#endif
		}
		else {
			#if HAS_LEVELING
				ICON_Leveing(0);
			#else
				ICON_StartInfo(0);
			#endif
		}
	}

	void HMI_Move_X(void)
	{
		ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
		if(encoder_diffState != ENCODER_DIFF_NO)
		{
			if(encoder_diffState == ENCODER_DIFF_CW) HMI_ValueStruct.Move_X_scale += EncoderRate.encoderMoveValue;
			else if(encoder_diffState == ENCODER_DIFF_CCW) HMI_ValueStruct.Move_X_scale -= EncoderRate.encoderMoveValue;
			else if(encoder_diffState == ENCODER_DIFF_ENTER) 
			{
				checkkey = AxisMove;
				EncoderRate.encoderRateEnabled = 0;
				DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Background_black, 3, 1, 216, 104, HMI_ValueStruct.Move_X_scale);
				if (!planner.is_full())
				{
					// Wait for planner moves to finish!
					planner.synchronize();
					planner.buffer_line(current_position, MMM_TO_MMS(HOMING_FEEDRATE_XY), active_extruder);
				}
				DWIN_UpdateLCD();
				return;
			}
			if (HMI_ValueStruct.Move_X_scale < X_MIN_PRINT_POS * MinUnitMult) HMI_ValueStruct.Move_X_scale = X_MIN_PRINT_POS * MinUnitMult;
			else if(HMI_ValueStruct.Move_X_scale > X_MAX_PRINT_POS * MinUnitMult) HMI_ValueStruct.Move_X_scale = X_MAX_PRINT_POS * MinUnitMult;
			current_position[X_AXIS] = HMI_ValueStruct.Move_X_scale/10;
			DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Select_Color, 3, 1, 216, 104, HMI_ValueStruct.Move_X_scale);
			DWIN_UpdateLCD();
		}
	}

	void HMI_Move_Y(void)
	{
		ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
		if(encoder_diffState != ENCODER_DIFF_NO)
		{
			if(encoder_diffState == ENCODER_DIFF_CW) HMI_ValueStruct.Move_Y_scale += EncoderRate.encoderMoveValue;
			else if(encoder_diffState == ENCODER_DIFF_CCW) HMI_ValueStruct.Move_Y_scale -= EncoderRate.encoderMoveValue;
			else if(encoder_diffState == ENCODER_DIFF_ENTER) 
			{
				checkkey = AxisMove;
				EncoderRate.encoderRateEnabled = 0;
				DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Background_black, 3, 1, 216, 157, HMI_ValueStruct.Move_Y_scale);
				if (!planner.is_full())
				{
					// Wait for planner moves to finish!
					planner.synchronize();
					planner.buffer_line(current_position, MMM_TO_MMS(HOMING_FEEDRATE_XY), active_extruder);
				}
				DWIN_UpdateLCD();
				return;
			}
			if (HMI_ValueStruct.Move_Y_scale < Y_MIN_PRINT_POS * MinUnitMult) HMI_ValueStruct.Move_Y_scale = Y_MIN_PRINT_POS * MinUnitMult;
			else if(HMI_ValueStruct.Move_Y_scale > Y_MAX_PRINT_POS * MinUnitMult) HMI_ValueStruct.Move_Y_scale = Y_MAX_PRINT_POS * MinUnitMult;
			current_position[Y_AXIS] = HMI_ValueStruct.Move_Y_scale/10;
			DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Select_Color, 3, 1, 216, 157, HMI_ValueStruct.Move_Y_scale);
			DWIN_UpdateLCD();
		}
	}

	void HMI_Move_Z(void)
	{
		ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
		if(encoder_diffState != ENCODER_DIFF_NO)
		{
			if(encoder_diffState == ENCODER_DIFF_CW) HMI_ValueStruct.Move_Z_scale += EncoderRate.encoderMoveValue;
			else if(encoder_diffState == ENCODER_DIFF_CCW) HMI_ValueStruct.Move_Z_scale -= EncoderRate.encoderMoveValue;
			else if(encoder_diffState == ENCODER_DIFF_ENTER) 
			{
				checkkey = AxisMove;
				EncoderRate.encoderRateEnabled = 0;
				DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Background_black, 3, 1, 216, 210, HMI_ValueStruct.Move_Z_scale);
				if (!planner.is_full())
				{
					// Wait for planner moves to finish!
					planner.synchronize();
					planner.buffer_line(current_position, MMM_TO_MMS(HOMING_FEEDRATE_Z), active_extruder);
				}
				DWIN_UpdateLCD();
				return;
			}
			if (HMI_ValueStruct.Move_Z_scale < Z_MIN_POS*MinUnitMult) HMI_ValueStruct.Move_Z_scale = Z_MIN_POS*MinUnitMult;
			else if(HMI_ValueStruct.Move_Z_scale > Z_MAX_POS*MinUnitMult) HMI_ValueStruct.Move_Z_scale = Z_MAX_POS*MinUnitMult;
			current_position[Z_AXIS] = HMI_ValueStruct.Move_Z_scale/10;
			DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Select_Color, 3, 1, 216, 210, HMI_ValueStruct.Move_Z_scale);
			DWIN_UpdateLCD();
		}
	}

	void HMI_Extruder(void)
	{
		ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
		if(encoder_diffState != ENCODER_DIFF_NO)
		{
			if(encoder_diffState == ENCODER_DIFF_CW) HMI_ValueStruct.Move_E_scale += EncoderRate.encoderMoveValue;
			else if(encoder_diffState == ENCODER_DIFF_CCW) HMI_ValueStruct.Move_E_scale -= EncoderRate.encoderMoveValue;
			else if(encoder_diffState == ENCODER_DIFF_ENTER) 
			{
				checkkey = AxisMove;
				EncoderRate.encoderRateEnabled = 0;
				last_E_scale = HMI_ValueStruct.Move_E_scale;
				show_plus_or_minus(font8x16, Background_black, 3, 1, 216, 263, HMI_ValueStruct.Move_E_scale);
				if (!planner.is_full())
				{
					// Wait for planner moves to finish!
					planner.synchronize();
					planner.buffer_line(current_position, MMM_TO_MMS(FEEDRATE_E), active_extruder);
				}
				DWIN_UpdateLCD();
				return;
			}
			if((HMI_ValueStruct.Move_E_scale-last_E_scale) > EXTRUDE_MAXLENGTH*MinUnitMult) HMI_ValueStruct.Move_E_scale = last_E_scale + EXTRUDE_MAXLENGTH*MinUnitMult;
			else if((last_E_scale-HMI_ValueStruct.Move_E_scale) > EXTRUDE_MAXLENGTH*MinUnitMult) HMI_ValueStruct.Move_E_scale = last_E_scale - EXTRUDE_MAXLENGTH*MinUnitMult;
			current_position.e = HMI_ValueStruct.Move_E_scale/10;
			show_plus_or_minus(font8x16, Select_Color, 3, 1, 216, 263, HMI_ValueStruct.Move_E_scale);
			DWIN_UpdateLCD();
		}
	}

	void HMI_Zoffset(void)
	{
		ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
		if(encoder_diffState != ENCODER_DIFF_NO)
		{
			last_zoffset = zprobe_zoffset;
			if(encoder_diffState == ENCODER_DIFF_CW) HMI_ValueStruct.offest_value += EncoderRate.encoderMoveValue;
			else if(encoder_diffState == ENCODER_DIFF_CCW) HMI_ValueStruct.offest_value -= EncoderRate.encoderMoveValue;
			else if(encoder_diffState == ENCODER_DIFF_ENTER) 
			{
				EncoderRate.encoderRateEnabled = 0;
				zprobe_zoffset = HMI_ValueStruct.offest_value/100;
				#if HAS_BED_PROBE
					if (WITHIN(zprobe_zoffset - last_zoffset, Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX))
						probe_offset.z = zprobe_zoffset;
				#endif
				babystep.add_mm(Z_AXIS, (zprobe_zoffset - last_zoffset));
				settings.save();

				if(HMI_ValueStruct.show_mode == -4)
				{
					checkkey = Perpare;
					#if HAS_LEVELING
						show_plus_or_minus(font8x16, Background_black, 2, 2, 202, 263-(index_perpare-5)*53,  probe_offset.z*100);
					#else
						show_plus_or_minus(font8x16, Background_black, 2, 2, 202, 263-(index_perpare-5)*53, HMI_ValueStruct.offest_value);
					#endif
				}
				else
				{
					checkkey = Tune;
					#if HAS_LEVELING
						show_plus_or_minus(font8x16, Background_black, 2, 2, 202, 316-(index_tune-5)*53, probe_offset.z*100);
					#else
						show_plus_or_minus(font8x16, Background_black, 2, 2, 202, 316-(index_tune-5)*53, HMI_ValueStruct.offest_value);
					#endif
				}
				DWIN_UpdateLCD();
				return;
			}
			if (HMI_ValueStruct.offest_value < Z_PROBE_OFFSET_RANGE_MIN*100) HMI_ValueStruct.offest_value = Z_PROBE_OFFSET_RANGE_MIN*100;
			else if(HMI_ValueStruct.offest_value > Z_PROBE_OFFSET_RANGE_MAX*100) HMI_ValueStruct.offest_value = Z_PROBE_OFFSET_RANGE_MAX*100;
			if(HMI_ValueStruct.show_mode == -4) show_plus_or_minus(font8x16, Select_Color, 2, 2, 202, 263-(index_perpare-5)*53, HMI_ValueStruct.offest_value);
			else show_plus_or_minus(font8x16, Select_Color, 2, 2, 202, 316-(index_tune-5)*53, HMI_ValueStruct.offest_value);
			DWIN_UpdateLCD();
		}
	}

	void HMI_ETemp(void)
	{
		ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
		if(encoder_diffState != ENCODER_DIFF_NO)
		{
			if(encoder_diffState == ENCODER_DIFF_CW) HMI_ValueStruct.E_Temp += EncoderRate.encoderMoveValue;
			else if(encoder_diffState == ENCODER_DIFF_CCW) HMI_ValueStruct.E_Temp -= EncoderRate.encoderMoveValue;
			else if(encoder_diffState == ENCODER_DIFF_ENTER) // return
			{
				EncoderRate.encoderRateEnabled = 0;
				if(HMI_ValueStruct.show_mode == -1)	// temperature
				{
					checkkey = Temperation;
					DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 104, HMI_ValueStruct.E_Temp);
				}
				else if(HMI_ValueStruct.show_mode == -2)
				{
					checkkey = PLAPreheat;
					HMI_ValueStruct.preheat_hotend_temp[0] = HMI_ValueStruct.E_Temp;
					DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 104, HMI_ValueStruct.preheat_hotend_temp[0]);
					return;
				}
				else if(HMI_ValueStruct.show_mode == -3)
				{
					checkkey = ABSPreheat;
					HMI_ValueStruct.preheat_hotend_temp[1] = HMI_ValueStruct.E_Temp;
					DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 104, HMI_ValueStruct.preheat_hotend_temp[1]);
					return;
				}
				else // tune
				{
					checkkey = Tune;	
					DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 157-(index_tune-5)*53, HMI_ValueStruct.E_Temp);
				}
				thermalManager.setTargetHotend(HMI_ValueStruct.E_Temp, 0);
				return;
			}
			// E_Temp limit
			if(HMI_ValueStruct.E_Temp > max_E_Temp) HMI_ValueStruct.E_Temp = max_E_Temp;	
			else if(HMI_ValueStruct.E_Temp < min_E_Temp) HMI_ValueStruct.E_Temp = min_E_Temp;
			// E_Temp value
			if(HMI_ValueStruct.show_mode >= 0)	// tune
				DWIN_Draw_IntVariate(true,true,0,font8x16,White,Select_Color, 3, 216, 157-(index_tune-5)*53, HMI_ValueStruct.E_Temp);
			else 	// other page
				DWIN_Draw_IntVariate(true,true,0,font8x16,White,Select_Color, 3, 216, 104, HMI_ValueStruct.E_Temp);				
		}
	}

	void HMI_BedTemp(void)
	{
		ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
		if(encoder_diffState != ENCODER_DIFF_NO)
		{
			if(encoder_diffState == ENCODER_DIFF_CW) HMI_ValueStruct.Bed_Temp += EncoderRate.encoderMoveValue;
			else if(encoder_diffState == ENCODER_DIFF_CCW) HMI_ValueStruct.Bed_Temp -= EncoderRate.encoderMoveValue;
			else if(encoder_diffState == ENCODER_DIFF_ENTER) // return
			{
				EncoderRate.encoderRateEnabled = 0;
				if(HMI_ValueStruct.show_mode == -1) 
				{
					checkkey = Temperation;
					DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 157, HMI_ValueStruct.Bed_Temp);
				}
				else if(HMI_ValueStruct.show_mode == -2)
				{
					checkkey = PLAPreheat;
					HMI_ValueStruct.preheat_bed_temp[0] = HMI_ValueStruct.Bed_Temp;
					DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 157, HMI_ValueStruct.preheat_bed_temp[0]);
					return;
				}
				else if(HMI_ValueStruct.show_mode == -3)
				{
					checkkey = ABSPreheat;
					HMI_ValueStruct.preheat_bed_temp[1] = HMI_ValueStruct.Bed_Temp;
					DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 157, HMI_ValueStruct.preheat_bed_temp[1]);
					return;
				}				
				else 
				{
					checkkey = Tune;	
					DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 210-(index_tune-5)*53, HMI_ValueStruct.Bed_Temp);
				}
				thermalManager.setTargetBed(HMI_ValueStruct.Bed_Temp);
				return;
			}
			//Bed_Temp limit
			if(HMI_ValueStruct.Bed_Temp > max_Bed_Temp) HMI_ValueStruct.Bed_Temp = max_Bed_Temp;	
			else if(HMI_ValueStruct.Bed_Temp < min_Bed_Temp) HMI_ValueStruct.Bed_Temp = min_Bed_Temp;
			//Bed_Temp value
			if(HMI_ValueStruct.show_mode >= 0) // tune page
				DWIN_Draw_IntVariate(true,true,0,font8x16,White,Select_Color, 3, 216, 210-(index_tune-5)*53, HMI_ValueStruct.Bed_Temp);
			else // other page
				DWIN_Draw_IntVariate(true,true,0,font8x16,White,Select_Color, 3, 216, 157, HMI_ValueStruct.Bed_Temp);				
		}
	}

	void HMI_FanSpeed(void)
	{
		ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
		if(encoder_diffState != ENCODER_DIFF_NO)
		{
			if(encoder_diffState == ENCODER_DIFF_CW) HMI_ValueStruct.Fan_speed += EncoderRate.encoderMoveValue;
			else if(encoder_diffState == ENCODER_DIFF_CCW) HMI_ValueStruct.Fan_speed -= EncoderRate.encoderMoveValue;
			else if(encoder_diffState == ENCODER_DIFF_ENTER) // return
			{
				EncoderRate.encoderRateEnabled = 0;
				if(HMI_ValueStruct.show_mode == -1) 
				{
					checkkey = Temperation;
					DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 210, HMI_ValueStruct.Fan_speed);
				}
				else if(HMI_ValueStruct.show_mode == -2)
				{
					checkkey = PLAPreheat;
					HMI_ValueStruct.preheat_fan_speed[0] = HMI_ValueStruct.Fan_speed;
					DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 210, HMI_ValueStruct.preheat_fan_speed[0]);
					return;
				}
				else if(HMI_ValueStruct.show_mode == -3)
				{
					checkkey = ABSPreheat;
					HMI_ValueStruct.preheat_fan_speed[1] = HMI_ValueStruct.Fan_speed;
					DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 210, HMI_ValueStruct.preheat_fan_speed[1]);
					return;
				}				
				else 
				{
					checkkey = Tune;	
					DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 263-(index_tune-5)*53, HMI_ValueStruct.Fan_speed);
				}
				thermalManager.set_fan_speed(0, HMI_ValueStruct.Fan_speed);
				return;
			}
			//Fan_speed limit
			if(HMI_ValueStruct.Fan_speed > FanOn) HMI_ValueStruct.Fan_speed = FanOn;	
			else if(HMI_ValueStruct.Fan_speed < FanOff) HMI_ValueStruct.Fan_speed = FanOff;
			//Fan_speed value
			if(HMI_ValueStruct.show_mode >= 0) // tune page
				DWIN_Draw_IntVariate(true,true,0,font8x16,White,Select_Color, 3, 216, 263-(index_tune-5)*53, HMI_ValueStruct.Fan_speed);
			else // other page
				DWIN_Draw_IntVariate(true,true,0,font8x16,White,Select_Color, 3, 216, 210, HMI_ValueStruct.Fan_speed);
		}
	}

	void HMI_PrintSpeed(void)
	{
		ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
		if(encoder_diffState != ENCODER_DIFF_NO)
		{
			if(encoder_diffState == ENCODER_DIFF_CW) HMI_ValueStruct.print_speed += EncoderRate.encoderMoveValue;
			else if(encoder_diffState == ENCODER_DIFF_CCW) HMI_ValueStruct.print_speed -= EncoderRate.encoderMoveValue;
			else if(encoder_diffState == ENCODER_DIFF_ENTER) // return
			{
				checkkey = Tune;
				EncoderRate.encoderRateEnabled = 0;
				feedrate_percentage = HMI_ValueStruct.print_speed;
				DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 104-(index_tune-5)*53, HMI_ValueStruct.print_speed);
				return;
			}
			//print_speed limit
			if(HMI_ValueStruct.print_speed > max_print_speed) HMI_ValueStruct.print_speed = max_print_speed;	
			else if(HMI_ValueStruct.print_speed < min_print_speed) HMI_ValueStruct.print_speed = min_print_speed;
			//print_speed value
			DWIN_Draw_IntVariate(true,true,0,font8x16,White,Select_Color, 3, 216, 104-(index_tune-5)*53, HMI_ValueStruct.print_speed);
		}
	}

	void HMI_MaxFeedspeedXYZE(void)
	{
		ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
		if(encoder_diffState != ENCODER_DIFF_NO)
		{
			if(encoder_diffState == ENCODER_DIFF_CW) HMI_ValueStruct.Max_Feedspeed += EncoderRate.encoderMoveValue;
			else if(encoder_diffState == ENCODER_DIFF_CCW) HMI_ValueStruct.Max_Feedspeed -= EncoderRate.encoderMoveValue;
			else if(encoder_diffState == ENCODER_DIFF_ENTER) // return
			{
				checkkey = MaxSpeed;
				EncoderRate.encoderRateEnabled = 0;
				if(HMI_flag.feedspeed_flag == X_AXIS) planner.set_max_feedrate(X_AXIS, HMI_ValueStruct.Max_Feedspeed);
				else if(HMI_flag.feedspeed_flag == Y_AXIS) planner.set_max_feedrate(Y_AXIS, HMI_ValueStruct.Max_Feedspeed);
				else if(HMI_flag.feedspeed_flag == Z_AXIS) planner.set_max_feedrate(Z_AXIS, HMI_ValueStruct.Max_Feedspeed);
				else if(HMI_flag.feedspeed_flag == E_AXIS) planner.set_max_feedrate(E_AXIS, HMI_ValueStruct.Max_Feedspeed);
				DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 4, 210, 104+(select_speed-1)*53, HMI_ValueStruct.Max_Feedspeed);
				return;
			}
			//MaxFeedspeed limit
			if(HMI_flag.feedspeed_flag == X_AXIS) {if(HMI_ValueStruct.Max_Feedspeed > default_max_feedrate[X_AXIS]*2) HMI_ValueStruct.Max_Feedspeed = default_max_feedrate[X_AXIS]*2;}
			else if(HMI_flag.feedspeed_flag == Y_AXIS) {if(HMI_ValueStruct.Max_Feedspeed > default_max_feedrate[Y_AXIS]*2) HMI_ValueStruct.Max_Feedspeed = default_max_feedrate[Y_AXIS]*2;}
			else if(HMI_flag.feedspeed_flag == Z_AXIS) {if(HMI_ValueStruct.Max_Feedspeed > default_max_feedrate[Z_AXIS]*2) HMI_ValueStruct.Max_Feedspeed = default_max_feedrate[Z_AXIS]*2;}
			else if(HMI_flag.feedspeed_flag == E_AXIS) {if(HMI_ValueStruct.Max_Feedspeed > default_max_feedrate[E_AXIS]*2) HMI_ValueStruct.Max_Feedspeed = default_max_feedrate[E_AXIS]*2;}
			if(HMI_ValueStruct.Max_Feedspeed < min_MaxFeedspeed) HMI_ValueStruct.Max_Feedspeed = min_MaxFeedspeed;
			//MaxFeedspeed value
			DWIN_Draw_IntVariate(true,true,0,font8x16,White,Select_Color, 4, 210, 104+(select_speed-1)*53, HMI_ValueStruct.Max_Feedspeed);
		}
	}

	void HMI_MaxAccelerationXYZE(void)
	{
		ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
		if(encoder_diffState != ENCODER_DIFF_NO)
		{
			if(encoder_diffState == ENCODER_DIFF_CW) HMI_ValueStruct.Max_Acceleration += EncoderRate.encoderMoveValue;
			else if(encoder_diffState == ENCODER_DIFF_CCW) HMI_ValueStruct.Max_Acceleration -= EncoderRate.encoderMoveValue;
			else if(encoder_diffState == ENCODER_DIFF_ENTER) // return
			{
				checkkey = MaxAcceleration;
				EncoderRate.encoderRateEnabled = 0;
				if(HMI_flag.acc_flag == X_AXIS) planner.set_max_acceleration(X_AXIS, HMI_ValueStruct.Max_Acceleration);
				else if(HMI_flag.acc_flag == Y_AXIS) planner.set_max_acceleration(Y_AXIS, HMI_ValueStruct.Max_Acceleration);
				else if(HMI_flag.acc_flag == Z_AXIS) planner.set_max_acceleration(Z_AXIS, HMI_ValueStruct.Max_Acceleration);
				else if(HMI_flag.acc_flag == E_AXIS) planner.set_max_acceleration(E_AXIS, HMI_ValueStruct.Max_Acceleration);
				DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 4, 210, 104+(select_acc-1)*53, HMI_ValueStruct.Max_Acceleration);
				return;
			}
			//MaxAcceleration limit
			if(HMI_flag.acc_flag == X_AXIS) {if(HMI_ValueStruct.Max_Acceleration > default_max_acceleration[X_AXIS]*2) HMI_ValueStruct.Max_Acceleration = default_max_acceleration[X_AXIS]*2;}
			else if(HMI_flag.acc_flag == Y_AXIS) {if(HMI_ValueStruct.Max_Acceleration > default_max_acceleration[Y_AXIS]*2) HMI_ValueStruct.Max_Acceleration = default_max_acceleration[Y_AXIS]*2;}
			else if(HMI_flag.acc_flag == Z_AXIS) {if(HMI_ValueStruct.Max_Acceleration > default_max_acceleration[Z_AXIS]*2) HMI_ValueStruct.Max_Acceleration = default_max_acceleration[Z_AXIS]*2;}
			else if(HMI_flag.acc_flag == E_AXIS) {if(HMI_ValueStruct.Max_Acceleration > default_max_acceleration[E_AXIS]*2) HMI_ValueStruct.Max_Acceleration = default_max_acceleration[E_AXIS]*2;}
			if(HMI_ValueStruct.Max_Acceleration < min_MaxAcceleration) HMI_ValueStruct.Max_Acceleration = min_MaxAcceleration;
			//MaxAcceleration value
			DWIN_Draw_IntVariate(true,true,0,font8x16,White,Select_Color, 4, 210, 104+(select_acc-1)*53, HMI_ValueStruct.Max_Acceleration);
		}
	}

	void HMI_MaxCornerXYZE(void)
	{
		ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
		if(encoder_diffState != ENCODER_DIFF_NO)
		{
			if(encoder_diffState == ENCODER_DIFF_CW) HMI_ValueStruct.Max_Corner += EncoderRate.encoderMoveValue;
			else if(encoder_diffState == ENCODER_DIFF_CCW) HMI_ValueStruct.Max_Corner -= EncoderRate.encoderMoveValue;
			else if(encoder_diffState == ENCODER_DIFF_ENTER) // return
			{
				checkkey = MaxCorner;
				EncoderRate.encoderRateEnabled = 0;
				if(HMI_flag.corner_flag == X_AXIS) planner.set_max_jerk(X_AXIS, HMI_ValueStruct.Max_Corner/10);
				else if(HMI_flag.corner_flag == Y_AXIS) planner.set_max_jerk(Y_AXIS, HMI_ValueStruct.Max_Corner/10);
				else if(HMI_flag.corner_flag == Z_AXIS) planner.set_max_jerk(Z_AXIS, HMI_ValueStruct.Max_Corner/10);
				else if(HMI_flag.corner_flag == E_AXIS) planner.set_max_jerk(E_AXIS, HMI_ValueStruct.Max_Corner/10);
				DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Background_black, 3, 1, 210, 104+(select_corner-1)*53, HMI_ValueStruct.Max_Corner);
				return;
			}
			//MaxCorner limit
			if(HMI_flag.corner_flag == X_AXIS) {if(HMI_ValueStruct.Max_Corner > default_max_jerk[X_AXIS]*2*MinUnitMult) HMI_ValueStruct.Max_Corner = default_max_jerk[X_AXIS]*2*MinUnitMult;}
			else if(HMI_flag.corner_flag == Y_AXIS) {if(HMI_ValueStruct.Max_Corner > default_max_jerk[Y_AXIS]*2*MinUnitMult) HMI_ValueStruct.Max_Corner = default_max_jerk[Y_AXIS]*2*MinUnitMult;}
			else if(HMI_flag.corner_flag == Z_AXIS) {if(HMI_ValueStruct.Max_Corner > default_max_jerk[Z_AXIS]*2*MinUnitMult) HMI_ValueStruct.Max_Corner = default_max_jerk[Z_AXIS]*2*MinUnitMult;}
			else if(HMI_flag.corner_flag == E_AXIS) {if(HMI_ValueStruct.Max_Corner > default_max_jerk[E_AXIS]*2*MinUnitMult) HMI_ValueStruct.Max_Corner = default_max_jerk[E_AXIS]*2*MinUnitMult;}
			if(HMI_ValueStruct.Max_Corner < min_MaxCorner*MinUnitMult) HMI_ValueStruct.Max_Corner = min_MaxCorner*MinUnitMult;
			//MaxCorner value
			DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Select_Color, 3, 1, 210, 104+(select_corner-1)*53, HMI_ValueStruct.Max_Corner);
		}
	}

	void HMI_StepXYZE(void)
	{
		ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
		if(encoder_diffState != ENCODER_DIFF_NO)
		{
			if(encoder_diffState == ENCODER_DIFF_CW) HMI_ValueStruct.Max_Steip += EncoderRate.encoderMoveValue;
			else if(encoder_diffState == ENCODER_DIFF_CCW) HMI_ValueStruct.Max_Steip -= EncoderRate.encoderMoveValue;
			else if(encoder_diffState == ENCODER_DIFF_ENTER) // return
			{
				checkkey = Step;
				EncoderRate.encoderRateEnabled = 0;
				if(HMI_flag.step_flag == X_AXIS) planner.settings.axis_steps_per_mm[X_AXIS] = HMI_ValueStruct.Max_Steip/10;
				else if(HMI_flag.step_flag == Y_AXIS) planner.settings.axis_steps_per_mm[Y_AXIS] = HMI_ValueStruct.Max_Steip/10;
				else if(HMI_flag.step_flag == Z_AXIS) planner.settings.axis_steps_per_mm[Z_AXIS] = HMI_ValueStruct.Max_Steip/10;
				else if(HMI_flag.step_flag == E_AXIS) planner.settings.axis_steps_per_mm[E_AXIS] = HMI_ValueStruct.Max_Steip/10;
				DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Background_black, 3, 1, 210, 104+(select_step-1)*53, HMI_ValueStruct.Max_Steip);
				return;
			}
			//Step limit
			if(HMI_flag.step_flag == X_AXIS) {if(HMI_ValueStruct.Max_Steip > default_axis_steps_per_unit[X_AXIS]*2*MinUnitMult) HMI_ValueStruct.Max_Steip = default_axis_steps_per_unit[X_AXIS]*2*MinUnitMult;}
			else if(HMI_flag.step_flag == Y_AXIS) {if(HMI_ValueStruct.Max_Steip > default_axis_steps_per_unit[Y_AXIS]*2*MinUnitMult) HMI_ValueStruct.Max_Steip = default_axis_steps_per_unit[Y_AXIS]*2*MinUnitMult;}
			else if(HMI_flag.step_flag == Z_AXIS) {if(HMI_ValueStruct.Max_Steip > default_axis_steps_per_unit[Z_AXIS]*2*MinUnitMult) HMI_ValueStruct.Max_Steip = default_axis_steps_per_unit[Z_AXIS]*2*MinUnitMult;}
			else if(HMI_flag.step_flag == E_AXIS) {if(HMI_ValueStruct.Max_Steip > default_axis_steps_per_unit[E_AXIS]*2*MinUnitMult) HMI_ValueStruct.Max_Steip = default_axis_steps_per_unit[E_AXIS]*2*MinUnitMult;}
			if(HMI_ValueStruct.Max_Steip < min_Step) HMI_ValueStruct.Max_Steip = min_Step;
			//Step value
			DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Select_Color, 3, 1, 210, 104+(select_step-1)*53, HMI_ValueStruct.Max_Steip);
		}
	}

	void update_variable(void)
	{
		/* Tune page temperation update */
		if(checkkey == Tune)
		{
			if(last_temp_hotend_target != thermalManager.temp_hotend[0].target)
			{
				DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 157-(index_tune-5)*53, thermalManager.temp_hotend[0].target);
			}
			if(last_temp_bed_target != thermalManager.temp_bed.target)
			{
				DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 210-(index_tune-5)*53, thermalManager.temp_bed.target);
			}
			if(last_fan_speed != thermalManager.fan_speed[0])
			{
				DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 263-(index_tune-5)*53, thermalManager.fan_speed[0]);
				last_fan_speed = thermalManager.fan_speed[0];
			}
		}

		/* Temperation page temperation update */
		if(checkkey == Temperation)
		{
			if(last_temp_hotend_target != thermalManager.temp_hotend[0].target)
			{
				DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 104, thermalManager.temp_hotend[0].target);
			}
			if(last_temp_bed_target != thermalManager.temp_bed.target)
			{
				DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 157, thermalManager.temp_bed.target);
			}
			if(last_fan_speed != thermalManager.fan_speed[0])
			{
				DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 210, thermalManager.fan_speed[0]);
				last_fan_speed = thermalManager.fan_speed[0];
			}
		}

		/* Bottom temperation update */
		if(last_temp_hotend_current != thermalManager.temp_hotend[0].celsius)
		{
			DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 33, 382, thermalManager.temp_hotend[0].celsius);
			last_temp_hotend_current = thermalManager.temp_hotend[0].celsius;
		}
		if(last_temp_hotend_target != thermalManager.temp_hotend[0].target)
		{
			DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 71, 382, thermalManager.temp_hotend[0].target);
			last_temp_hotend_target = thermalManager.temp_hotend[0].target;
		}
		if(last_temp_bed_current != thermalManager.temp_bed.celsius)
		{
			DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 178, 382, thermalManager.temp_bed.celsius);
			last_temp_bed_current = thermalManager.temp_bed.celsius;
		}
		if(last_temp_bed_target != thermalManager.temp_bed.target)
		{
			DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 382, thermalManager.temp_bed.target);
			last_temp_bed_target = thermalManager.temp_bed.target;
		}
		if(last_speed != feedrate_percentage)
		{
			DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 33, 429, feedrate_percentage);
			last_speed = feedrate_percentage;
		}
		#if HAS_LEVELING
			if(last_probe_zoffset != probe_offset.z)
			{
				show_plus_or_minus(font8x16, Background_black, 2, 2, 178+8, 429, probe_offset.z*100);
				last_probe_zoffset = probe_offset.z;
			}
		#else
			if(last_zoffset != zprobe_zoffset)
			{
				show_plus_or_minus(font8x16, Background_black, 2, 2, 178+8, 429, zprobe_zoffset*100);
				last_zoffset = zprobe_zoffset;
			}
		#endif
	}

	/* SD card init */
	void HMI_SDCardInit(void)
	{
		if(!IS_SD_INSERTED())
		{
			card.mount(); 
		}
		delay(2);
		if (IS_SD_INSERTED())
		{
			fileCnt = card.get_num_Files();
			card.getWorkDirName();
			if (card.filename[0] == '/') 
			{
				card.mount();
			}
			else 
			{
				card.cdup();
			}
			
			int addrnum =0;
			int num = 0;
			for (uint16_t i = 0; i < fileCnt && i < MaxFileNumber + addrnum; i++) 
			{
				card.selectFileByIndex(fileCnt-1-i);
				char *pointFilename = card.longFilename;
				int filenamelen = strlen(card.longFilename);
				int j = 1;

				while((strncmp(&pointFilename[j],".gcode",6) && strncmp(&pointFilename[j],".GCODE",6)) && (j++) < filenamelen);
				if(j >= filenamelen)
				{
					addrnum++;
					continue;
				}

				if(j >= TEXTBYTELEN)	
				{
					strncpy(&card.longFilename[TEXTBYTELEN-3],"..",2);
					card.longFilename[TEXTBYTELEN-1] = '\0';
					j = TEXTBYTELEN-1;
				}
				delay(3);

				strncpy(Cardbuf.Cardshowfilename[num], card.longFilename,j);
				strcpy(Cardbuf.Cardfilename[num], card.filename);
				Cardbuf.Filesum = (++num);
				if(Cardbuf.Filesum < 5) __fileCnt = Cardbuf.Filesum;
				else __fileCnt = 5;
			}
			DWIN_lcd_sd_status = IS_SD_INSERTED();
		}
		else
		{
			SERIAL_ECHOLN("***Initing card fails***");
		}
	}

	/* SD card staus update */
	void HMI_SDCardUpdate(void)
	{
		const bool sd_status = IS_SD_INSERTED();
		if (sd_status != DWIN_lcd_sd_status && !HMI_flag.home_flag)
		{
			if (sd_status)
			{
        delay(20);  // SD card power on
				card.mount();
				HMI_SDCardInit();
				if(checkkey == PrintFile)
				{
					for(short int i = 0; i < __fileCnt; i++) 
					{
						DWIN_Draw_String(false,false,font8x16,White,Background_black, 60, 101+i*53, Cardbuf.Cardshowfilename[i]);
						DWIN_ICON_Show(ICON,ICON_File, 26, 99+i*53);
						DWIN_Draw_Line(Line_Color, 16, 135+i*53, 256, 136+i*53);
					}
				}
			}
			else
			{
				Cardbuf.Filesum = __fileCnt = fileCnt = 0;
				card.release();
				// clean file icon 
				if(checkkey == PrintFile) 
				{
					last_select_file = select_file = 0;
					index_file = 5;

					DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);

					if(HMI_flag.language_flag) 
						DWIN_Frame_AreaCopy(1, 129, 72, 271-115, 479-395, 60, 49);
					else 
						DWIN_Frame_AreaCopy(1, 226, 179, 271-15, 479-290, 60, 49);

					DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31);
					DWIN_ICON_Show(ICON,ICON_Back, 26, 46);
					DWIN_Draw_Line(Line_Color, 16, 82, 256, 83);
				}
				else if(checkkey == PrintProcess || checkkey == Tune || printingIsActive() || printingIsPaused())
				{
					card.flag.abort_sd_printing = 1;
					wait_for_heatup = false;
					abort_flag = true;
				}
				memset(&Cardbuf,0,sizeof(Cardbuf));
			}
			DWIN_lcd_sd_status = sd_status;
			DWIN_UpdateLCD();
		}
	}

	/* Start */
	void HMI_StartFrame(void)
	{
		Goto_ProcessFrame();

		DWIN_Draw_Rectangle(1, Background_black,	0,	360,	272,	479);

		DWIN_ICON_Show(ICON,ICON_HotendTemp, 13, 381);
		DWIN_ICON_Show(ICON,ICON_Celsius, 100, 381);
		DWIN_ICON_Show(ICON,ICON_BedTemp, 158, 381);
		DWIN_ICON_Show(ICON,ICON_Celsius, 245, 381);
		DWIN_ICON_Show(ICON,ICON_Speed, 13, 428);
		DWIN_ICON_Show(ICON,ICON_Zoffest, 158, 428);

		// show value
		DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 33, 382, thermalManager.temp_hotend[0].celsius);
		DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 71, 382, thermalManager.temp_hotend[0].target);	
		DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 178, 382, thermalManager.temp_bed.celsius);
		DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 382, thermalManager.temp_bed.target);	
		DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 33, 429, feedrate_percentage);
		#if HAS_LEVELING
			show_plus_or_minus(font8x16, Background_black, 2, 2, 178+8, 429, probe_offset.z*100);
		#else
			show_plus_or_minus(font8x16, Background_black, 2, 2, 178+8, 429, zprobe_zoffset*100);
		#endif
		DWIN_Draw_String(false,false,font8x16,White,Background_black, 62, 383, (char*)"/");
		DWIN_Draw_String(false,false,font8x16,White,Background_black, 207, 383, (char*)"/");
		DWIN_Draw_String(false,false,font8x16,White,Background_black, 58, 429, (char*)"%");

		DWIN_UpdateLCD();
		
		delay(5);
	}

	/* Main Process */
	void HMI_ProcessFrame(void)
	{
		millis_t ms = millis();
		if(ms > Encoder_ms)
		{
			ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
			if(encoder_diffState != ENCODER_DIFF_NO)
			{
				Encoder_ms = ms + Encoder_wait;
				if(encoder_diffState == ENCODER_DIFF_CW)
				{
					if(select_page < 3) select_page++;
					else select_page = 3;
					if(last_select_page != select_page)
					{
						if(select_page == 0) 
						{
							ICON_Print(1);
						}
						else if(select_page == 1) 
						{
							ICON_Print(0);
							ICON_Prepare(1);
						}
						else if(select_page == 2) 
						{
							ICON_Prepare(0);
							ICON_Control(1);
						}
						else if(select_page == 3) 
						{
							ICON_Control(0);
							#if HAS_LEVELING
								ICON_Leveing(1);
							#else
								ICON_StartInfo(1);
							#endif
						}
						last_select_page = select_page;
					}
				}
				else if(encoder_diffState == ENCODER_DIFF_CCW)
				{
					if(select_page != 0) select_page--;
					else select_page = 0;
					if(last_select_page != select_page)
					{
						if(select_page == 0) 
						{
							ICON_Print(1);
							ICON_Prepare(0);
						}
						else if(select_page == 1) 
						{
							ICON_Prepare(1);
							ICON_Control(0);
						}
						else if(select_page == 2) 
						{
							ICON_Control(1);
							#if HAS_LEVELING
								ICON_Leveing(0);
							#else
								ICON_StartInfo(0);
							#endif
						}
						else if(select_page == 3) 
						{
							#if HAS_LEVELING
								ICON_Leveing(1);
							#else
								ICON_StartInfo(1);
							#endif
						}
						last_select_page = select_page;
					}
				}
				else if(encoder_diffState == ENCODER_DIFF_ENTER)
				{
					switch(select_page)
					{
						/* Print File */
						case 0:
							checkkey = PrintFile;
							last_select_file = select_file = 0;
							index_file = 5;

							DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
							DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);

							if(HMI_flag.language_flag) 
							{
								DWIN_Frame_AreaCopy(1, 0, 31, 271-216, 479-435, 14, 8);
								DWIN_Frame_AreaCopy(1, 129, 72, 271-115, 479-395, 60, 49);
							}
							else 
							{
								DWIN_Frame_AreaCopy(1, 52, 31, 271-134,	479-438, 14, 8);
								DWIN_Frame_AreaCopy(1, 226, 179, 271-15, 479-290, 60, 49);
							}

							DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31);
							DWIN_ICON_Show(ICON,ICON_Back, 26, 46);
							DWIN_Draw_Line(Line_Color, 16, 82, 256, 83);

							for(short int i = 0; i < __fileCnt; i++)
							{
								DWIN_Draw_String(false,false,font8x16,White,Background_black, 60, 101+i*53, Cardbuf.Cardshowfilename[i]);
								DWIN_ICON_Show(ICON,ICON_File, 26, 99+i*53);
								DWIN_Draw_Line(Line_Color, 16, 135+i*53, 256, 136+i*53);
							}
 
							break;
						
						/* Perpare */
						case 1:
							checkkey = Perpare;
							last_select_perpare = select_perpare = 0;
							index_perpare = 5;

							DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
							DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);

							if(HMI_flag.language_flag) 
							{
								DWIN_Frame_AreaCopy(1, 133, 1, 271-111, 479-465-1, 14, 8);
								DWIN_Frame_AreaCopy(1, 129, 72, 271-115, 479-395, 60, 49);

								DWIN_Frame_AreaCopy(1, 159, 70, 271-71, 479-395, 60, 102);
								DWIN_Frame_AreaCopy(1, 204, 70, 271-12, 479-397, 60, 155);
								DWIN_Frame_AreaCopy(1, 0, 89, 271-230, 479-378, 60, 208);
								#if HAS_BED_PROBE
									DWIN_Frame_AreaCopy(1, 174, 164, 271-48, 479-302, 60, 261);
									show_plus_or_minus(font8x16, Background_black, 2, 2, 202, 263-(index_perpare-5)*53, probe_offset.z*100);
								#else
									DWIN_Frame_AreaCopy(1, 43, 89, 271-173, 479-378, 60, 261);
								#endif
								DWIN_Frame_AreaCopy(1, 100, 89, 271-93-27, 479-378, 60, 314);
							}
							else 
							{
								DWIN_Frame_AreaCopy(1, 178, 2, 271-42, 479-464-1, 14, 8);
								DWIN_Frame_AreaCopy(1, 226, 179, 271-15, 479-290, 60, 49);

								DWIN_Frame_AreaCopy(1, 69, 61, 271-169, 479-408, 60, 102);
								DWIN_Frame_AreaCopy(1, 103, 59, 271-71, 479-405, 60, 155);
								DWIN_Frame_AreaCopy(1, 202, 61, 271-0, 479-408, 60, 208);
								#if HAS_BED_PROBE
									DWIN_Frame_AreaCopy(1, 93, 179, 271-130, 479-290, 60, 261);
									show_plus_or_minus(font8x16, Background_black, 2, 2, 202, 263-(index_perpare-5)*53, probe_offset.z*100);
								#else
									DWIN_Frame_AreaCopy(1, 1, 76, 271-165, 479-393, 60, 261);
								#endif
								DWIN_Frame_AreaCopy(1, 107, 76, 271-115, 479-393, 60, 314);
								DWIN_Frame_AreaCopy(1, 157, 76, 181, 479-393, 60+49+3, 314);
							}

							DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31);
							DWIN_ICON_Show(ICON,ICON_Back, 26, 46);
							DWIN_Draw_Line(Line_Color, 16, 82, 256, 83);
							for(short int i = 0; i < 5; i++) 
							{
								DWIN_ICON_Show(ICON,ICON_Axis+i, 26, 99+i*53);
								DWIN_Draw_Line(Line_Color, 16, 135+i*53, 256, 136+i*53);
							}

							DWIN_ICON_Show(ICON,ICON_More, 226, 99);
							    
							break;

						/* Control */
						case 2:
							checkkey = Control;
							last_select_control = select_control = 0;
							index_control = 5;

							DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
							DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);

							if(HMI_flag.language_flag) 
							{
								DWIN_Frame_AreaCopy(1, 103, 1, 271-141, 479-465, 14, 8);
								DWIN_Frame_AreaCopy(1, 129, 72, 271-115, 479-395, 60, 49);

								DWIN_Frame_AreaCopy(1, 57, 104, 271-187, 479-363, 60, 102);
								DWIN_Frame_AreaCopy(1, 87, 104, 271-157, 479-363, 60, 155);
								DWIN_Frame_AreaCopy(1, 117, 104, 271-99, 479-363, 60, 208);
								DWIN_Frame_AreaCopy(1, 174, 103, 271-42, 479-363, 60, 261);
								DWIN_Frame_AreaCopy(1, 1, 118, 271-215, 479-348, 60, 314);
							}
							else 
							{
								DWIN_Frame_AreaCopy(1, 128, 2, 271-95, 479-467, 14, 8);
								DWIN_Frame_AreaCopy(1, 226, 179, 271-15, 479-290, 60, 49);

								DWIN_Frame_AreaCopy(1, 1, 89, 271-188, 479-377-1, 60, 102);
								DWIN_Frame_AreaCopy(1, 84, 89, 271-143, 479-380, 60, 155);
								DWIN_Frame_AreaCopy(1, 131, 89, 271-3, 479-377-1, 60, 208);
								DWIN_Frame_AreaCopy(1, 26, 104, 271-214, 479-365, 60, 261);
								DWIN_Frame_AreaCopy(1, 182, 89, 271-3, 479-377-1, 60+31+3, 261);
								DWIN_Frame_AreaCopy(1, 59, 104, 271-178, 479-365, 60, 314);	
								DWIN_Frame_AreaCopy(1, 182, 89, 271-3, 479-377-1, 60+34+3, 314);							
							}

							DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31);
							DWIN_ICON_Show(ICON,ICON_Back, 26, 46);
							DWIN_Draw_Line(Line_Color, 16, 82, 256, 83);
							for(short int i = 0; i < 5; i++) 
							{
								DWIN_ICON_Show(ICON,ICON_Temperation+i, 26, 99+i*53);
								DWIN_Draw_Line(Line_Color, 16, 135+i*53, 256, 136+i*53);
							}

							DWIN_ICON_Show(ICON,ICON_More, 226, 99);
							DWIN_ICON_Show(ICON,ICON_More, 226, 152);

							break;

						/* Leveing or Info*/
						case 3:
							#if HAS_LEVELING
								checkkey = Leveing;
								HMI_Leveing();
							#else
								checkkey = Info;

								DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
								DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);

								DWIN_Draw_String(false,false,font8x16,White,Background_black, 136-(strlen(MACHINE_SIZE)*8)/2, 122, (char*)MACHINE_SIZE);
								DWIN_Draw_String(false,false,font8x16,White,Background_black, 136-(strlen(SHORT_BUILD_VERSION)*8)/2, 195, (char*)SHORT_BUILD_VERSION);
								if(HMI_flag.language_flag) 
								{
									DWIN_Frame_AreaCopy(1, 30, 17, 271-214, 479-450, 14, 8);
									DWIN_Frame_AreaCopy(1, 129, 72, 271-115, 479-395, 60, 49);

									DWIN_Frame_AreaCopy(1, 197, 149, 271-19, 479-318, 108, 102);
									DWIN_Frame_AreaCopy(1, 1, 164, 271-215, 479-303, 108, 175);
									DWIN_Frame_AreaCopy(1, 58, 164, 271-158, 479-303, 105, 248);
									DWIN_Draw_String(false,false,font8x16,White,Background_black, 136-(strlen(CORP_WEBSITE_C)*8)/2, 268, (char*)CORP_WEBSITE_C);
								}
								else 
								{
									DWIN_Frame_AreaCopy(1, 190, 16, 271-56, 479-453, 14, 8);
									DWIN_Frame_AreaCopy(1, 226, 179, 271-15, 479-290, 60, 49);

									DWIN_Frame_AreaCopy(1, 120, 150, 146, 479-318, 124, 102);
									DWIN_Frame_AreaCopy(1, 146, 151, 271-17, 479-318, 82, 175);
									DWIN_Frame_AreaCopy(1, 0, 165, 271-177, 479-304, 89, 248);
									DWIN_Draw_String(false,false,font8x16,White,Background_black, 136-(strlen(CORP_WEBSITE_E)*8)/2, 268, (char*)CORP_WEBSITE_E);
								}

								DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31);
								DWIN_ICON_Show(ICON,ICON_Back, 26, 46);
								DWIN_Draw_Line(Line_Color, 16, 82, 256, 83);
								for(short int i = 0; i < 3; i++) 
								{
									DWIN_ICON_Show(ICON,ICON_PrintSize+i, 26, 99+i*73);
									DWIN_Draw_Line(Line_Color, 16, 155+i*73, 256, 156+i*73);
								}
							#endif

							break;
					}
				}
				DWIN_UpdateLCD();
			}
		}
	}

	/* Print File */
	void HMI_Printfile(void)
	{
		millis_t ms = millis();
		if(ms > Encoder_ms)
		{
			ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
			if(encoder_diffState != ENCODER_DIFF_NO)
			{
				Encoder_ms = ms + Encoder_wait;
				if(encoder_diffState == ENCODER_DIFF_CW && Cardbuf.Filesum)
				{
					if(select_file < Cardbuf.Filesum) select_file++;
					else select_file = Cardbuf.Filesum;
					if(last_select_file != select_file)
					{
						if(select_file > 5 && select_file > index_file)
						{
							DWIN_Frame_AreaMove(1, 2, 53, Background_black, 0, 31, 272, 349);
							DWIN_ICON_Show(ICON,ICON_File, 26, 311);
							index_file = select_file;
							DWIN_Draw_Rectangle(1, Background_black,	0,	243,	14,	296);
							DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 296);
							DWIN_Draw_Line(Line_Color, 16, 347, 256, 348);
							DWIN_Draw_String(false,false,font8x16,White,Background_black, 60, 313, Cardbuf.Cardshowfilename[select_file-1]);
							last_select_file = select_file;
						}
						else
						{
							DWIN_Draw_Rectangle(1, Background_black,	0,	31+((select_file-index_file+5)-1)*53,	14,	31+(select_file-index_file+5)*53);
							DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31+(select_file-index_file+5)*53);
							last_select_file = select_file;
						}
					}
				}
				else if(encoder_diffState == ENCODER_DIFF_CCW && Cardbuf.Filesum)
				{
					if(select_file != 0) select_file--;
					else select_file = 0;
					if(last_select_file != select_file)
					{
						if(select_file < index_file-5)
						{
							DWIN_Frame_AreaMove(1, 3, 53, Background_black, 0, 31, 272, 349);
							if(select_file) DWIN_ICON_Show(ICON,ICON_File, 26, 46);
							else DWIN_ICON_Show(ICON,ICON_Back, 26, 46);
							index_file--;
							DWIN_Draw_Rectangle(1, Background_black,	0,	84,	14,	137);
							DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31);
							DWIN_Draw_Line(Line_Color, 16, 82, 256, 83);
							last_select_file = select_file;
							if(index_file == 5) 
							{
								if(HMI_flag.language_flag) 
									DWIN_Frame_AreaCopy(1, 129, 72, 271-115, 479-395, 60, 49);
								else 
									DWIN_Frame_AreaCopy(1, 226, 179, 271-15, 479-290, 60, 49);
							}
							else DWIN_Draw_String(false,false,font8x16,White,Background_black, 60, 48, Cardbuf.Cardshowfilename[select_file-1]);
						}
						else
						{
							DWIN_Draw_Rectangle(1, Background_black,	0,	31+((select_file-index_file+5)+1)*53,	14,	31+((select_file-index_file+5)+2)*53);
							DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31+(select_file-index_file+5)*53);
							last_select_file = select_file;
						}
					}
				}
				else if(encoder_diffState == ENCODER_DIFF_ENTER)
				{
					if(select_file == 0)
					{
						/* back */
						select_page = 0;
						Goto_ProcessFrame();
					}
					else
					{
						/* choose printing G-code file and start print*/
						checkkey = PrintProcess;
						select_print = 0;
						HMI_flag.heat_flag = 1;
						HMI_flag.print_finish = 0;
						HMI_ValueStruct.show_mode = 0;

						// start choice and print SD file
						char cmd[30];
						sprintf_P(cmd, PSTR("M23 %s"), Cardbuf.Cardfilename[select_file-1]);
						SERIAL_ECHO("Start printing file: ");
						SERIAL_ECHOLN(Cardbuf.Cardfilename[select_file-1]);
						queue.enqueue_one_now(cmd);
						queue.enqueue_now_P(PSTR("M24"));

						card.removeJobRecoveryFile();

						#if FAN_COUNT > 0
							for (uint8_t i = 0; i < FAN_COUNT; i++) thermalManager.fan_speed[i] = FanOn;
						#endif

						DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
						DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);

						if(HMI_flag.language_flag) 
						{
							DWIN_Frame_AreaCopy(1, 30, 1, 271-200, 479-465, 14, 9);
							DWIN_Frame_AreaCopy(1, 0, 72, 271-208, 479-393, 41, 188);
							DWIN_Frame_AreaCopy(1, 65, 72, 271-143, 479-393, 176, 188);
						}
						else 
						{
							DWIN_Frame_AreaCopy(1, 40, 2, 271-179, 479-464-1, 14, 9);
							DWIN_Frame_AreaCopy(1, 0, 44, 271-175, 479-420-1, 41, 188);
							DWIN_Frame_AreaCopy(1, 98, 44, 271-119, 479-420-1, 176, 188);
						}

						ICON_Setting(1);
						ICON_Pause(0);
						ICON_Stop(0);

						HMI_ValueStruct.print_speed = feedrate_percentage = 100;
						sprintf(filebuf,"%s\056gcode",Cardbuf.Cardshowfilename[select_file-1]);
						DWIN_Draw_String(false,false,font8x16,White,Background_black, 136-(strlen(filebuf)*8)/2, 60, filebuf);	// filename
						DWIN_ICON_Show(ICON,ICON_Bar, 15, 93);
						DWIN_Draw_Rectangle(1, BarFill_Color,	16,	93,	256,	113);
						DWIN_Draw_IntVariate(true,true,0,font8x16,Percent_Color,Background_black, 2, 117, 133, card.percentDone());
						DWIN_Draw_String(false,false,font8x16,Percent_Color,Background_black, 117+16, 133, (char*)"%");

						DWIN_ICON_Show(ICON,ICON_PrintTime, 17, 193);
						DWIN_ICON_Show(ICON,ICON_RemainTime, 150, 191);
						DWIN_Draw_IntVariate(true,true,1,font8x16,White,Background_black, 2, 42, 212, 0);
						DWIN_Draw_String(false,false,font8x16,White,Background_black, 42+16, 212, (char*)":");	
						DWIN_Draw_IntVariate(true,true,1,font8x16,White,Background_black, 2, 42+24, 212, 0);
						DWIN_Draw_IntVariate(true,true,1,font8x16,White,Background_black, 2, 176, 212, 0);
						DWIN_Draw_String(false,false,font8x16,White,Background_black, 176+16, 212, (char*)":");	
						DWIN_Draw_IntVariate(true,true,1,font8x16,White,Background_black, 2, 176+24, 212, 0);
					}
				}
				DWIN_UpdateLCD();
			}
		}
	}

	/* Printing */
	void HMI_Printing(void)
	{
		millis_t ms = millis();
		if(ms > Encoder_ms)
		{
			ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
			if(encoder_diffState != ENCODER_DIFF_NO)
			{
				Encoder_ms = ms + Encoder_wait;
				if(HMI_flag.confirm_flag)
				{
					if(encoder_diffState == ENCODER_DIFF_ENTER)
					{
						HMI_flag.confirm_flag = 0;
						HMI_ValueStruct.print_speed = feedrate_percentage = 100;
						#if HAS_LEVELING
							zprobe_zoffset = probe_offset.z;
						#else
							zprobe_zoffset = 0;
						#endif

						select_page = 0;
						Goto_ProcessFrame();
						queue.enqueue_now_P(PSTR("G92 E0"));
					}
					return;
				}
				// Avoid flicker by updating only the previous menu
				if(encoder_diffState == ENCODER_DIFF_CW)
				{
					if(select_print < 2) select_print++;
					else select_print = 2;
					if(last_select_print != select_print)
					{
						if(select_print == 0)
						{
							ICON_Setting(1);
						}
						else if(select_print == 1)
						{
							ICON_Setting(0);
							if(printingIsPaused()) ICON_Continue(1);
							else ICON_Pause(1);
						}
						else if(select_print == 2)
						{
							if(printingIsPaused()) ICON_Continue(0);
							else ICON_Pause(0);
							ICON_Stop(1);
						}
						last_select_print = select_print;
					}
				}
				else if(encoder_diffState == ENCODER_DIFF_CCW)
				{
					if(select_print != 0) select_print--;
					else select_print = 0;

					if(last_select_print != select_print)
					{
						if(select_print == 0)
						{
							ICON_Setting(1);
							if(printingIsPaused()) ICON_Continue(0);
							else ICON_Pause(0);
						}
						else if(select_print == 1)
						{
							if(printingIsPaused()) ICON_Continue(1);
							else ICON_Pause(1);
							ICON_Stop(0);
						}
						else if(select_print == 2)
						{
							ICON_Stop(1);
						}
						last_select_print = select_print;
					}
				}
				else if(encoder_diffState == ENCODER_DIFF_ENTER)
				{
						switch(select_print)
						{
							case 0:	// setting

								checkkey = Tune;
								HMI_ValueStruct.show_mode = 0;
								last_select_tune = select_tune = 0;
								index_tune = 5;

								DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
								DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);

								if(HMI_flag.language_flag) 
								{
									DWIN_Frame_AreaCopy(1, 73, 2, 271-171, 479-466, 14, 9);
									DWIN_Frame_AreaCopy(1, 129, 72, 271-115, 479-395, 60, 49);

									DWIN_Frame_AreaCopy(1, 116, 164, 271-100, 479-303, 60, 102);
									DWIN_Frame_AreaCopy(1, 1, 134, 271-215, 479-333, 60, 155);
									DWIN_Frame_AreaCopy(1, 58, 134, 271-158, 479-333, 60, 208);
									DWIN_Frame_AreaCopy(1, 115, 134, 271-99, 479-333, 60, 261);
									DWIN_Frame_AreaCopy(1, 174, 164, 271-48, 479-302, 60, 314);
								}
								else 
								{
									DWIN_Frame_AreaCopy(1, 94, 2, 271-145, 479-467, 14, 9);
									DWIN_Frame_AreaCopy(1, 226, 179, 271-15, 479-290, 60, 49);

									DWIN_Frame_AreaCopy(1, 1, 179, 271-179, 479-287, 60, 102);		// print speed
									DWIN_Frame_AreaCopy(1, 197, 104, 271-31, 479-365, 60, 155);
									DWIN_Frame_AreaCopy(1, 1, 89, 271-188, 479-377-1, 60+41+3, 155);	// nozzle temp
									DWIN_Frame_AreaCopy(1, 240, 104, 271-7, 479-365, 60, 208);
									DWIN_Frame_AreaCopy(1, 1, 89, 271-188, 479-377-1, 60+24+3, 208);	// bed temp
									DWIN_Frame_AreaCopy(1, 0, 119, 271-207, 479-347, 60, 261);		// fan temp
									DWIN_Frame_AreaCopy(1, 93, 179, 271-130, 479-290, 60, 314);		// Z offest
								}

								DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31);
								DWIN_ICON_Show(ICON,ICON_Back, 26, 46);
								for(short int i = 0; i < 6; i++) 
								{
									DWIN_Draw_Line(Line_Color, 16, 82+i*53, 256, 83+i*53);
								}

								DWIN_ICON_Show(ICON,ICON_Speed, 26, 99);
								DWIN_ICON_Show(ICON,ICON_HotendTemp, 26, 152);
								DWIN_ICON_Show(ICON,ICON_BedTemp, 26, 205);
								DWIN_ICON_Show(ICON,ICON_FanSpeed, 26, 258);
								DWIN_ICON_Show(ICON,ICON_Zoffest, 26, 311);

								DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 104-(index_tune-5)*53, feedrate_percentage);
								DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 157-(index_tune-5)*53, thermalManager.temp_hotend[0].target);
								DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 210-(index_tune-5)*53, thermalManager.temp_bed.target);
								DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 263-(index_tune-5)*53, thermalManager.fan_speed[0]);
								#if HAS_LEVELING
									show_plus_or_minus(font8x16, Background_black, 2, 2, 202, 316-(index_tune-5)*53, probe_offset.z*100);
								#else
									show_plus_or_minus(font8x16, Background_black, 2, 2, 202, 316-(index_tune-5)*53, zprobe_zoffset*100);
								#endif

								break;
							case 1: // pause
								/* pause */
								if(HMI_flag.pause_flag && printingIsPaused()) 
								{
									pause_action_flag = 0;
									ICON_Pause(1);

									#if PAUSE_HEAT
										char cmd[20];
										if(tempbed > 0)	{
											sprintf_P(cmd, PSTR("M190 S%i"), tempbed);
											gcode.process_subcommands_now(cmd);
										}
										if(temphot > 0)	{
											sprintf_P(cmd, PSTR("M109 S%i"), temphot);
											gcode.process_subcommands_now(cmd);
										}
									#endif

									gcode.process_subcommands_now_P(PSTR("M24"));
								}
								else if(printingIsActive())
								{
									HMI_flag.select_flag = 1;
									checkkey = Print_window;
									Popup_wiundow_PauseOrStop();
								}
								break;

							case 2: // stop
								/* stop */
								HMI_flag.select_flag = 1;
								checkkey = Print_window;
								Popup_wiundow_PauseOrStop();
								break;

							default:
								break;
						}
				}
				DWIN_UpdateLCD();
			}
		}	
	}

	/* pause and stop window */
	void HMI_PauseOrStop(void)
	{
		millis_t ms = millis();
		if(ms > Encoder_ms)
		{
			ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
			if(encoder_diffState != ENCODER_DIFF_NO)
			{
				Encoder_ms = ms + Encoder_wait;
				if(encoder_diffState == ENCODER_DIFF_CW)
				{
					HMI_flag.select_flag = 0;
					DWIN_Draw_Rectangle(0, Background_window, 25, 279, 126, 318);
					DWIN_Draw_Rectangle(0, Background_window, 24, 278, 127, 319);
					DWIN_Draw_Rectangle(0, Select_Color, 145, 279, 246, 318);
					DWIN_Draw_Rectangle(0, Select_Color, 144, 278, 247, 319);
				}
				else if(encoder_diffState == ENCODER_DIFF_CCW)
				{
					HMI_flag.select_flag = 1;
					DWIN_Draw_Rectangle(0, Background_window, 145, 279, 246, 318);
					DWIN_Draw_Rectangle(0, Background_window, 144, 278, 247, 319);
					DWIN_Draw_Rectangle(0, Select_Color, 25, 279, 126, 318);
					DWIN_Draw_Rectangle(0, Select_Color, 24, 278, 127, 319);
				}
				else if(encoder_diffState == ENCODER_DIFF_ENTER)
				{
					if(select_print == 1) // pause window
					{
						if(HMI_flag.select_flag) 
						{
							pause_action_flag = 1;
							ICON_Continue(1);
							#if ENABLED(POWER_LOSS_RECOVERY)
								if (recovery.enabled) recovery.save(true, false);
							#endif
							queue.inject_P(PSTR("M25"));
						}
						else 
						{
							// cancel pause
						}
						Goto_PrintProcess();
					}
					else if(select_print == 2) // stop window
					{
						if(HMI_flag.select_flag) 
						{
							if(HMI_flag.heat_flag)
							{
								checkkey = Back_Main;
								// Wait for planner moves to finish!
								if(HMI_flag.home_flag) planner.synchronize();
								card.stopSDPrint(
									#if SD_RESORT
										true
									#endif
								);
								queue.clear();
								quickstop_stepper();
								print_job_timer.stop();
								#if DISABLED(SD_ABORT_NO_COOLDOWN)
									thermalManager.disable_all_heaters();
								#endif
								thermalManager.zero_fan_speeds();
								wait_for_heatup = false;
								#if ENABLED(POWER_LOSS_RECOVERY)
									card.removeJobRecoveryFile();
								#endif
								#ifdef EVENT_GCODE_SD_STOP
									Popup_Window_Home();
									queue.inject_P(PSTR(EVENT_GCODE_SD_STOP));
									HMI_flag.home_flag = true;
								#endif
							}
							else
							{
								checkkey = Back_Main;
								if(HMI_flag.home_flag) planner.synchronize();
								Popup_Window_Home();
								HMI_flag.home_flag = true;
								card.flag.abort_sd_printing = 1;
							}
							abort_flag = true;
						}
						else 
						{
							Goto_PrintProcess();	// cancel stop
						}
					}
				}
				DWIN_UpdateLCD();
			}
		}
	}

	/* Check filament */
	void HMI_Filament(void)
	{
		millis_t ms = millis();
		if(ms > Encoder_ms)
		{
			ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
			if(encoder_diffState != ENCODER_DIFF_NO)
			{
				Encoder_ms = ms + Encoder_wait;
				if(encoder_diffState == ENCODER_DIFF_CW)
				{
					HMI_flag.select_flag = 0;
					DWIN_Draw_Rectangle(0, Background_window, 25, 279, 126, 318);
					DWIN_Draw_Rectangle(0, Background_window, 24, 278, 127, 319);
					DWIN_Draw_Rectangle(0, Select_Color, 145, 279, 246, 318);
					DWIN_Draw_Rectangle(0, Select_Color, 144, 278, 247, 319);
				}
				else if(encoder_diffState == ENCODER_DIFF_CCW)
				{
					HMI_flag.select_flag = 1;
					DWIN_Draw_Rectangle(0, Background_window, 145, 279, 246, 318);
					DWIN_Draw_Rectangle(0, Background_window, 144, 278, 247, 319);
					DWIN_Draw_Rectangle(0, Select_Color, 25, 279, 126, 318);
					DWIN_Draw_Rectangle(0, Select_Color, 24, 278, 127, 319);
				}
				else if(encoder_diffState == ENCODER_DIFF_ENTER)
				{
					if(HMI_flag.select_flag) 
					{
						if(READ(CHECKFILAMENT_PIN))
						{
							pause_action_flag = 0;
							Goto_PrintProcess();

							#if PAUSE_HEAT
								char cmd[20];
								if(tempbed > 0)	{
									sprintf_P(cmd, PSTR("M190 S%i"), tempbed);
									gcode.process_subcommands_now(cmd);
								}
								if(temphot > 0)	{
									sprintf_P(cmd, PSTR("M109 S%i"), temphot);
									gcode.process_subcommands_now(cmd);
								}
							#endif

							gcode.process_subcommands_now_P(PSTR("M24"));
						}
					}
					else 
					{
						Goto_PrintProcess();
					}
				}
				DWIN_UpdateLCD();
			}
		}		
	}

	/* Perpare */
	void HMI_Perpare(void)
	{
		millis_t ms = millis();
		if(ms > Encoder_ms)
		{
			ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
			if(encoder_diffState != ENCODER_DIFF_NO)
			{
				Encoder_ms = ms + Encoder_wait;
				// Avoid flicker by updating only the previous menu
				if(encoder_diffState == ENCODER_DIFF_CW)
				{
					if(select_perpare < 8) select_perpare++;
					else select_perpare = 8;

					if(last_select_perpare != select_perpare)
					{
						if(select_perpare > 5 && select_perpare > index_perpare)
						{
							DWIN_Frame_AreaMove(1, 2, 53, Background_black, 0, 31, 272, 349);
							DWIN_ICON_Show(ICON,ICON_Axis+select_perpare-1, 26, 311);
							index_perpare = select_perpare;
							DWIN_Draw_Rectangle(1, Background_black,	0,	243,	14,	296);
							DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 296);
							DWIN_Draw_Line(Line_Color, 16, 347, 256, 348);
							last_select_perpare = select_perpare;
							if(index_perpare < 7) DWIN_ICON_Show(ICON,ICON_More, 226, 99-(index_perpare-5)*53);
							if(HMI_flag.language_flag)
							{
								if(index_perpare == 6) DWIN_Frame_AreaCopy(1, 180, 89, 271-11-27, 479-379, 60, 314);
								else if(index_perpare == 7) DWIN_Frame_AreaCopy(1, 1, 104, 271-215, 479-362, 60, 314);
								else if(index_perpare == 8) {DWIN_Frame_AreaCopy(1, 239, 134, 271-5, 479-333, 60, 314); DWIN_Draw_String(false,false,font8x16,White,Background_black, 226, 314, (char*)"CN");}
							}
							else
							{
								if(index_perpare == 6) 
								{
									DWIN_Frame_AreaCopy(1, 107, 76, 271-115, 479-393, 60, 314);
									DWIN_Frame_AreaCopy(1, 172, 76, 198, 479-393, 60+49+3, 314);									
								}
								else if(index_perpare == 7) DWIN_Frame_AreaCopy(1, 200, 76, 271-7, 479-393, 60, 314);
								else if(index_perpare == 8) {DWIN_Frame_AreaCopy(1, 0, 194, 271-150, 479-272, 60, 314); DWIN_Draw_String(false,false,font8x16,White,Background_black, 226, 314, (char*)"EN");}		
							}
						}
						else
						{
							DWIN_Draw_Rectangle(1, Background_black,	0,	31+((select_perpare-index_perpare+5)-1)*53,	14,	31+(select_perpare-index_perpare+5)*53);
							DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31+(select_perpare-index_perpare+5)*53);
							last_select_perpare = select_perpare;
						}
					}
				}
				else if(encoder_diffState == ENCODER_DIFF_CCW)
				{
					if(select_perpare != 0) select_perpare--;
					else select_perpare = 0;

					if(last_select_perpare != select_perpare)
					{
						if(select_perpare < index_perpare-5)
						{
							DWIN_Frame_AreaMove(1, 3, 53, Background_black, 0, 31, 272, 349);
							if(select_perpare) DWIN_ICON_Show(ICON,ICON_Axis+select_perpare-1, 26, 46);
							else DWIN_ICON_Show(ICON,ICON_Back, 26, 46);
							index_perpare--;
							DWIN_Draw_Rectangle(1, Background_black,	0,	84,	14,	137);
							DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31);
							DWIN_Draw_Line(Line_Color, 16, 82, 256, 83);
							last_select_perpare = select_perpare;
							if(index_perpare < 7) DWIN_ICON_Show(ICON,ICON_More, 226, 99-(index_perpare-5)*53);
							if(HMI_flag.language_flag)
							{
								if(index_perpare == 5) DWIN_Frame_AreaCopy(1, 129, 72, 271-115, 479-395, 60, 49);
								else if(index_perpare == 6) DWIN_Frame_AreaCopy(1, 159, 70, 271-71, 479-395, 60, 49);
								else if(index_perpare == 7) DWIN_Frame_AreaCopy(1, 204, 70, 271-12, 479-397, 60, 49);
								else if(index_perpare == 8) DWIN_Frame_AreaCopy(1, 0, 89, 271-230, 479-378, 60, 49);
							}
							else
							{
								if(index_perpare == 5) DWIN_Frame_AreaCopy(1, 226, 179, 271-15, 479-290, 60, 49);
								else if(index_perpare == 6) DWIN_Frame_AreaCopy(1, 69, 61, 271-169, 479-408, 60, 49);
								else if(index_perpare == 7) DWIN_Frame_AreaCopy(1, 103, 59, 271-71, 479-405, 60, 49);
								else if(index_perpare == 8) DWIN_Frame_AreaCopy(1, 202, 61, 271-0, 479-408, 60, 49);
							}
						}
						else
						{
							DWIN_Draw_Rectangle(1, Background_black,	0,	31+((select_perpare-index_perpare+5)+1)*53,	14,	31+((select_perpare-index_perpare+5)+2)*53);
							DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31+(select_perpare-index_perpare+5)*53);
							last_select_perpare = select_perpare;
						}
					}
				}
				else if(encoder_diffState == ENCODER_DIFF_ENTER)
				{
					switch(select_perpare)
					{
						case 0:	// back
							select_page = 1;
							Goto_ProcessFrame();
							break;
						case 1: // axis move
							checkkey = AxisMove;
							last_select_axis = select_axis = 0;

							DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
							DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);

							if(HMI_flag.language_flag) 
							{
								DWIN_Frame_AreaCopy(1, 192, 1, 271-38, 479-465, 14, 8);
								DWIN_Frame_AreaCopy(1, 129, 72, 271-115, 479-395, 60, 49);

								DWIN_Frame_AreaCopy(1, 58, 118, 271-165, 479-347, 60, 102);
								DWIN_Frame_AreaCopy(1, 109, 118, 271-114, 479-347, 60, 155);
								DWIN_Frame_AreaCopy(1, 160, 118, 271-62, 479-347, 60, 208);
								DWIN_Frame_AreaCopy(1, 212, 118, 271-18, 479-348, 60, 261);
							}
							else 
							{
								DWIN_Frame_AreaCopy(1, 231, 2, 271-6, 479-467, 14, 8);
								DWIN_Frame_AreaCopy(1, 226, 179, 271-15, 479-290, 60, 49);

								DWIN_Frame_AreaCopy(1, 69, 61, 271-169, 479-408, 60, 102);
								DWIN_Frame_AreaCopy(1, 95, 104, 271-169, 479-365, 60+33+3, 102);
								DWIN_Frame_AreaCopy(1, 69, 61, 271-169, 479-408, 60, 155);
								DWIN_Frame_AreaCopy(1, 104, 104, 271-161, 479-365, 60+33+3, 155);
								DWIN_Frame_AreaCopy(1, 69, 61, 271-169, 479-408, 60, 208);
								DWIN_Frame_AreaCopy(1, 112, 104, 271-151, 479-365, 60+33+3, 208);
								DWIN_Frame_AreaCopy(1, 123, 192, 271-95, 479-277, 60, 261);
							}

							DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31);
							DWIN_ICON_Show(ICON,ICON_Back, 26, 46);
							for(short int i = 0; i < 5; i++) 
							{
								DWIN_ICON_Show(ICON,ICON_MoveX+i, 26, 99+i*53);
								DWIN_Draw_Line(Line_Color, 16, 82+i*53, 256, 83+i*53);
							}

							DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Background_black, 3, 1, 216, 104, current_position[X_AXIS]*MinUnitMult);
							DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Background_black, 3, 1, 216, 157, current_position[Y_AXIS]*MinUnitMult);
							DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Background_black, 3, 1, 216, 210, current_position[Z_AXIS]*MinUnitMult);
							show_plus_or_minus(font8x16, Background_black, 3, 1, 216, 263, current_position.e*MinUnitMult);

							break;
						case 2: // close motion
							queue.enqueue_now_P(PSTR("M84"));
							break;
						case 3:	// homing
							checkkey = Last_Perpare;
							HMI_flag.home_flag = true;
							index_perpare = 5;
							queue.enqueue_now_P(PSTR("G28"));
							Popup_Window_Home();
							break;
						case 4:	// Z-offest
							#if HAS_LEVELING
								checkkey = Homeoffset;
								HMI_ValueStruct.show_mode = -4;
								HMI_ValueStruct.offest_value = probe_offset.z*100;
								show_plus_or_minus(font8x16, Select_Color, 2, 2, 202, 263-(index_perpare-5)*53, HMI_ValueStruct.offest_value);
								EncoderRate.encoderRateEnabled = 1;
							#else
								queue.enqueue_now_P(PSTR("G92 X0 Y0 Z0"));
								buzzer.tone(100, 659);
								buzzer.tone(100, 698);
							#endif
							break;
						case 5:	// PLA preheat
							thermalManager.setTargetHotend(HMI_ValueStruct.preheat_hotend_temp[0], 0);
							thermalManager.setTargetBed(HMI_ValueStruct.preheat_bed_temp[0]);
							thermalManager.set_fan_speed(0, HMI_ValueStruct.preheat_fan_speed[0]);
							break;
						case 6: // ABS preheat
							thermalManager.setTargetHotend(HMI_ValueStruct.preheat_hotend_temp[1], 0);
							thermalManager.setTargetBed(HMI_ValueStruct.preheat_bed_temp[1]);
							thermalManager.set_fan_speed(0, HMI_ValueStruct.preheat_fan_speed[1]);
							break;
						case 7: // cool
							thermalManager.zero_fan_speeds();
							thermalManager.disable_all_heaters();
							break;
						case 8: // language
							/* select language */
							HMI_flag.language_flag = !HMI_flag.language_flag;

							DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
							DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);

							if(HMI_flag.language_flag) 
							{
								set_chinese_to_eeprom();
								DWIN_JPG_CacheTo1(Language_Chinese);

								DWIN_Frame_AreaCopy(1, 133, 1, 271-111, 479-465, 14, 8);

								DWIN_Frame_AreaCopy(1, 0, 89, 271-230, 479-378, 60, 49);
								#if HAS_BED_PROBE
									DWIN_Frame_AreaCopy(1, 174, 164, 271-48, 479-302, 60, 102);
									show_plus_or_minus(font8x16, Background_black, 2, 2, 202, 263-(index_perpare-5)*53, probe_offset.z*100);
								#else
									DWIN_Frame_AreaCopy(1, 43, 89, 271-173, 479-378, 60, 102);
								#endif
								DWIN_Frame_AreaCopy(1, 100, 89, 271-93-27, 479-379, 60, 155);
								DWIN_Frame_AreaCopy(1, 180, 89, 271-11-27, 479-379, 60, 208);
								DWIN_Frame_AreaCopy(1, 1, 104, 271-215, 479-362, 60, 261);
								{DWIN_Frame_AreaCopy(1, 239, 134, 271-5, 479-333, 60, 314); DWIN_Draw_String(false,false,font8x16,White,Background_black, 226, 314, (char*)"CN");}
							}
							else 
							{
								set_english_to_eeprom();
								DWIN_JPG_CacheTo1(Language_English);

								DWIN_Frame_AreaCopy(1, 178, 2, 271-42, 479-464-1, 14, 8);

								DWIN_Frame_AreaCopy(1, 202, 61, 271-0, 479-408, 60, 49);
								#if HAS_BED_PROBE
									DWIN_Frame_AreaCopy(1, 93, 179, 271-130, 479-290, 60, 102);
									show_plus_or_minus(font8x16, Background_black, 2, 2, 202, 263-(index_perpare-5)*53, probe_offset.z*100);
								#else
									DWIN_Frame_AreaCopy(1, 1, 76, 271-165, 479-393, 60, 102);
								#endif
								DWIN_Frame_AreaCopy(1, 107, 76, 271-115, 479-393, 60, 155);
								DWIN_Frame_AreaCopy(1, 157, 76, 181, 479-393, 60+49+3, 155);
								DWIN_Frame_AreaCopy(1, 107, 76, 271-115, 479-393, 60, 208);
								DWIN_Frame_AreaCopy(1, 172, 76, 198, 479-393, 60+49+3, 208);									
								DWIN_Frame_AreaCopy(1, 200, 76, 271-7, 479-393, 60, 261);
								{DWIN_Frame_AreaCopy(1, 0, 194, 271-150, 479-272, 60, 314); DWIN_Draw_String(false,false,font8x16,White,Background_black, 226, 314, (char*)"EN");}		
							}

							DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 296);
							for(short int i = 0; i < 6; i++) 
							{
								DWIN_ICON_Show(ICON,ICON_Homeing+i, 26, 46+i*53);
								DWIN_Draw_Line(Line_Color, 16, 82+i*53, 256, 83+i*53);
							}

							break;
						default:
							break;
					}
				}
				DWIN_UpdateLCD();
			}
		}		
	}

	/* Control */
	void HMI_Control(void)
	{
		millis_t ms = millis();
		if(ms > Encoder_ms)
		{
			ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
			if(encoder_diffState != ENCODER_DIFF_NO)
			{
				Encoder_ms = ms + Encoder_wait;
				// Avoid flicker by updating only the previous menu
				if(encoder_diffState == ENCODER_DIFF_CW)
				{
					#if HAS_LEVELING
						if(select_control < 6) select_control++;
						else select_control = 6;
					#else
						if(select_control < 5) select_control++;
						else select_control = 5;
					#endif

					if(last_select_control != select_control)
					{
						if(select_control > 5 && select_control > index_control)
						{
							DWIN_Frame_AreaMove(1, 2, 53, Background_black, 0, 31, 272, 349);
							DWIN_ICON_Show(ICON,ICON_Temperation+select_control-1, 26, 311);
							index_control = select_control;
							DWIN_Draw_Rectangle(1, Background_black,	0,	243,	14,	296);
							DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 296);
							DWIN_Draw_Line(Line_Color, 16, 347, 256, 348);
							last_select_control = select_control;
							DWIN_ICON_Show(ICON,ICON_More, 226, 99-(index_control-5)*53);
							DWIN_ICON_Show(ICON,ICON_More, 226, 152-(index_control-5)*53);
							if(index_control > 5) 
							{
								DWIN_ICON_Show(ICON,ICON_More, 226, 311-(index_control-6)*53);
								if(HMI_flag.language_flag)
									DWIN_Frame_AreaCopy(1, 231, 103, 271-13, 479-363, 60, 314);
								else
									DWIN_Frame_AreaCopy(1, 0, 104, 271-247, 479-365, 60, 314);
							}
						}
						else
						{						
							DWIN_Draw_Rectangle(1, Background_black,	0,	31+((select_control-index_control+5)-1)*53,	14,	31+(select_control-index_control+5)*53);
							DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31+(select_control-index_control+5)*53);
							last_select_control = select_control;
						}
					}
				}
				else if(encoder_diffState == ENCODER_DIFF_CCW)
				{
					if(select_control != 0) select_control--;
					else select_control = 0;

					if(last_select_control != select_control)
					{
						if(select_control < index_control-5)
						{
							DWIN_Frame_AreaMove(1, 3, 53, Background_black, 0, 31, 272, 349);
							if(select_control) DWIN_ICON_Show(ICON,ICON_Temperation+select_control-1, 26, 46);
							else DWIN_ICON_Show(ICON,ICON_Back, 26, 46);
							index_control--;
							DWIN_Draw_Rectangle(1, Background_black,	0,	84,	14,	137);
							DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31);
							DWIN_Draw_Line(Line_Color, 16, 82, 256, 83);
							last_select_control = select_control;
							DWIN_ICON_Show(ICON,ICON_More, 226, 99-(index_control-5)*53);
							DWIN_ICON_Show(ICON,ICON_More, 226, 152-(index_control-5)*53);
							if(index_control == 5) 
							{
								if(HMI_flag.language_flag)
									DWIN_Frame_AreaCopy(1, 129, 72, 271-115, 479-395, 60, 49);
								else
									DWIN_Frame_AreaCopy(1, 226, 179, 271-15, 479-290, 60, 49);
							}
						}
						else
						{						
							DWIN_Draw_Rectangle(1, Background_black,	0,	31+((select_control-index_control+5)+1)*53,	14,	31+((select_control-index_control+5)+2)*53);
							DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31+(select_control-index_control+5)*53);
							last_select_control = select_control;
						}
					}
				}
				else if(encoder_diffState == ENCODER_DIFF_ENTER)
				{
						switch(select_control)
						{
							case 0:	// back
								select_page = 2;
								Goto_ProcessFrame();
								break;
							case 1: // temperation

								checkkey = Temperation;
								HMI_ValueStruct.show_mode = -1;
								last_select_temp = select_temp = 0;

								DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
								DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);

								if(HMI_flag.language_flag) 
								{
									DWIN_Frame_AreaCopy(1, 236, 2, 271-8, 479-466, 14, 8);
									DWIN_Frame_AreaCopy(1, 129, 72, 271-115, 479-395, 60, 49);

									DWIN_Frame_AreaCopy(1, 1, 134, 271-215, 479-333, 60, 102);
									DWIN_Frame_AreaCopy(1, 58, 134, 271-158, 479-333, 60, 155);
									DWIN_Frame_AreaCopy(1, 115, 134, 271-99, 479-333, 60, 208);
									DWIN_Frame_AreaCopy(1, 100, 89, 271-93, 479-378, 60, 261);
									DWIN_Frame_AreaCopy(1, 180, 89, 271-11, 479-379, 60, 314);
								}
								else 
								{
									DWIN_Frame_AreaCopy(1, 56, 16, 139, 479-450-1, 14, 8);
									DWIN_Frame_AreaCopy(1, 226, 179, 271-15, 479-290, 60, 49);

									DWIN_Frame_AreaCopy(1, 197, 104, 271-31, 479-365, 60, 102);
									DWIN_Frame_AreaCopy(1, 1, 89, 271-188, 479-377-1, 60+41+3, 102);
									DWIN_Frame_AreaCopy(1, 240, 104, 271-7, 479-365, 60, 155);
									DWIN_Frame_AreaCopy(1, 1, 89, 271-188, 479-377-1, 60+24+3, 155);
									DWIN_Frame_AreaCopy(1, 0, 119, 271-207, 479-347, 60, 208);
									DWIN_Frame_AreaCopy(1, 107, 76, 271-115, 479-393, 60, 261);
									DWIN_Frame_AreaCopy(1, 157, 76, 181, 479-393, 60+49+3, 261);
									DWIN_Frame_AreaCopy(1, 131, 119, 271-89, 479-347, 60+49+24+6, 261);	// PLA setting
									DWIN_Frame_AreaCopy(1, 107, 76, 271-115, 479-393, 60, 314);
									DWIN_Frame_AreaCopy(1, 172, 76, 198, 479-393, 60+49+3, 314);
									DWIN_Frame_AreaCopy(1, 131, 119, 271-89, 479-347, 60+49+24+6, 314);	// ABS setting									
								}

								DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31);
								DWIN_ICON_Show(ICON,ICON_Back, 26, 46);
								DWIN_Draw_Line(Line_Color, 16, 82, 256, 83);
								for(short int i = 0; i < 5; i++) 
								{
									DWIN_ICON_Show(ICON,ICON_SetEndTemp+i, 26, 99+i*53);
									DWIN_Draw_Line(Line_Color, 16, 135+i*53, 256, 136+i*53);
								}

								DWIN_ICON_Show(ICON,ICON_More, 226, 258);
								DWIN_ICON_Show(ICON,ICON_More, 226, 311);

								DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 104, thermalManager.temp_hotend[0].target);
								DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 157, thermalManager.temp_bed.target);
								DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 210, thermalManager.fan_speed[0]);

								break;
							case 2: // motion

								checkkey = Motion;
								last_select_motion = select_motion = 0;

								DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
								DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);

								if(HMI_flag.language_flag) 
								{
									DWIN_Frame_AreaCopy(1, 1, 16, 271-243, 479-451, 14, 8);
									DWIN_Frame_AreaCopy(1, 129, 72, 271-115, 479-395, 60, 49);

									DWIN_Frame_AreaCopy(1, 173, 133, 228, 479-332, 60, 102);				// max speed
									DWIN_Frame_AreaCopy(1, 173, 133, 200, 479-332, 60, 155);
									DWIN_Frame_AreaCopy(1, 28, 149, 271-202, 479-318, 60+27, 155+1);	// max acceleration
									DWIN_Frame_AreaCopy(1, 173, 133, 200, 479-332, 60, 208);
									DWIN_Frame_AreaCopy(1, 1, 180, 271-243, 479-287, 60+27, 208+1);
									DWIN_Frame_AreaCopy(1, 202, 133, 228, 479-332, 60+54, 208);				// max corner
									DWIN_Frame_AreaCopy(1, 153, 148, 271-77, 479-318, 60, 261);			// transmission ratio
								}
								else 
								{
									DWIN_Frame_AreaCopy(1, 144, 16, 271-82, 479-453, 14, 8);
									DWIN_Frame_AreaCopy(1, 226, 179, 271-15, 479-290, 60, 49);
									
									DWIN_Frame_AreaCopy(1, 245, 119, 271-2, 479-350, 60, 102);					
									DWIN_Frame_AreaCopy(1, 184, 119, 224, 479-347, 60+24+3, 102);				// max speed
									DWIN_Frame_AreaCopy(1, 245, 119, 271-2, 479-350, 60, 155);	
									DWIN_Frame_AreaCopy(1, 1, 135, 271-192, 479-334, 60+24+3, 155);		// max acceleration
									DWIN_Frame_AreaCopy(1, 245, 119, 271-2, 479-350, 60, 208);
									DWIN_Frame_AreaCopy(1, 64, 119, 271-165, 479-350, 60+24+3, 208);	// max corner
									DWIN_Frame_AreaCopy(1, 1, 151, 271-154, 479-318, 60, 261);			// transmission ratio
								}

								DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31);
								DWIN_ICON_Show(ICON,ICON_Back, 26, 46);
								DWIN_Draw_Line(Line_Color, 16, 82, 256, 83);
								for(short int i = 0; i < 4; i++) 
								{
									DWIN_ICON_Show(ICON,ICON_MaxSpeed+i, 26, 99+i*53);
									DWIN_Draw_Line(Line_Color, 16, 135+i*53, 256, 136+i*53);
								}

								DWIN_ICON_Show(ICON,ICON_More, 226, 99);
								DWIN_ICON_Show(ICON,ICON_More, 226, 152);
								DWIN_ICON_Show(ICON,ICON_More, 226, 205);
								DWIN_ICON_Show(ICON,ICON_More, 226, 258);

								break;
							case 3: // write EEPROM
								if (settings.save()) {
									buzzer.tone(100, 659);
									buzzer.tone(100, 698);
								}
								else buzzer.tone(20, 440);
								break;
							case 4: // read EEPROM
									if (settings.load()) {
										buzzer.tone(100, 659);
										buzzer.tone(100, 698);
									}
									else buzzer.tone(20, 440);
								break;
							case 5: // resume EEPROM
								settings.reset();
								if (settings.save()) {
									buzzer.tone(100, 659);
									buzzer.tone(100, 698);
								}
								else buzzer.tone(20, 440);
								break;
							case 6:	// info

								checkkey = Info;

								DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
								DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);

								DWIN_Draw_String(false,false,font8x16,White,Background_black, 136-(strlen(MACHINE_SIZE)*8)/2, 122, (char*)MACHINE_SIZE);
								DWIN_Draw_String(false,false,font8x16,White,Background_black, 136-(strlen(SHORT_BUILD_VERSION)*8)/2, 195, (char*)SHORT_BUILD_VERSION);
								if(HMI_flag.language_flag) 
								{
									DWIN_Frame_AreaCopy(1, 30, 17, 271-214, 479-450, 14, 8);
									DWIN_Frame_AreaCopy(1, 129, 72, 271-115, 479-395, 60, 49);

									DWIN_Frame_AreaCopy(1, 197, 149, 271-19, 479-318, 108, 102);
									DWIN_Frame_AreaCopy(1, 1, 164, 271-215, 479-303, 108, 175);
									DWIN_Frame_AreaCopy(1, 58, 164, 271-158, 479-303, 105, 248);
									DWIN_Draw_String(false,false,font8x16,White,Background_black, 136-(strlen(CORP_WEBSITE_C)*8)/2, 268, (char*)CORP_WEBSITE_C);
								}
								else 
								{
									DWIN_Frame_AreaCopy(1, 190, 16, 271-56, 479-453, 14, 8);
									DWIN_Frame_AreaCopy(1, 226, 179, 271-15, 479-290, 60, 49);

									DWIN_Frame_AreaCopy(1, 120, 150, 146, 479-318, 124, 102);
									DWIN_Frame_AreaCopy(1, 146, 151, 271-17, 479-318, 82, 175);
									DWIN_Frame_AreaCopy(1, 0, 165, 271-177, 479-304, 89, 248);
									DWIN_Draw_String(false,false,font8x16,White,Background_black, 136-(strlen(CORP_WEBSITE_E)*8)/2, 268, (char*)CORP_WEBSITE_E);
								}

								DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31);
								DWIN_ICON_Show(ICON,ICON_Back, 26, 46);
								DWIN_Draw_Line(Line_Color, 16, 82, 256, 83);
								for(short int i = 0; i < 3; i++) 
								{
									DWIN_ICON_Show(ICON,ICON_PrintSize+i, 26, 99+i*73);
									DWIN_Draw_Line(Line_Color, 16, 155+i*73, 256, 156+i*73);
								}

								break;
							default:
								break;
						}
				}
				DWIN_UpdateLCD();
			}
		}		
	}

	/* Leveing */
	void HMI_Leveing(void)
	{
		Popup_Window_Leveing();
		DWIN_UpdateLCD();
		queue.enqueue_now_P(PSTR("G28""\n""G29"));
	}

	/* Axis Move */
	void HMI_AxisMove(void)
	{
		millis_t ms = millis();
		if(ms > Encoder_ms)
		{
			ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();	
			if(encoder_diffState != ENCODER_DIFF_NO)
			{
				Encoder_ms = ms + Encoder_wait;
				// popup window resume
				if(HMI_flag.ETempTooLow_flag)
				{
					if(encoder_diffState == ENCODER_DIFF_ENTER)
					{
						HMI_flag.ETempTooLow_flag = 0;

						DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
						DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);

						if(HMI_flag.language_flag) 
						{
							DWIN_Frame_AreaCopy(1, 192, 1, 271-38, 479-465, 14, 8);
							DWIN_Frame_AreaCopy(1, 129, 72, 271-115, 479-395, 60, 49);

							DWIN_Frame_AreaCopy(1, 58, 118, 271-165, 479-347, 60, 102);
							DWIN_Frame_AreaCopy(1, 109, 118, 271-114, 479-347, 60, 155);
							DWIN_Frame_AreaCopy(1, 160, 118, 271-62, 479-347, 60, 208);
							DWIN_Frame_AreaCopy(1, 212, 118, 271-18, 479-348, 60, 261);
						}
						else 
						{
							DWIN_Frame_AreaCopy(1, 231, 2, 271-6, 479-467, 14, 8);
							DWIN_Frame_AreaCopy(1, 226, 179, 271-15, 479-290, 60, 49);

							DWIN_Frame_AreaCopy(1, 69, 61, 271-169, 479-408, 60, 102);
							DWIN_Frame_AreaCopy(1, 95, 104, 271-169, 479-365, 60+33+3, 102);
							DWIN_Frame_AreaCopy(1, 69, 61, 271-169, 479-408, 60, 155);
							DWIN_Frame_AreaCopy(1, 104, 104, 271-161, 479-365, 60+33+3, 155);
							DWIN_Frame_AreaCopy(1, 69, 61, 271-169, 479-408, 60, 208);
							DWIN_Frame_AreaCopy(1, 112, 104, 271-151, 479-365, 60+33+3, 208);
							DWIN_Frame_AreaCopy(1, 123, 192, 271-95, 479-277, 60, 261);
						}

						DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 243);
						DWIN_ICON_Show(ICON,ICON_Back, 26, 46);
						for(short int i = 0; i < 5; i++) 
						{
							DWIN_ICON_Show(ICON,ICON_MoveX+i, 26, 99+i*53);
							DWIN_Draw_Line(Line_Color, 16, 82+i*53, 256, 83+i*53);
						}

						DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Background_black, 3, 1, 216, 104, current_position[X_AXIS]*MinUnitMult);
						DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Background_black, 3, 1, 216, 157, current_position[Y_AXIS]*MinUnitMult);
						DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Background_black, 3, 1, 216, 210, current_position[Z_AXIS]*MinUnitMult);
						show_plus_or_minus(font8x16, Background_black, 3, 1, 216, 263, current_position.e*MinUnitMult);

						DWIN_UpdateLCD();
					}
					return;
				}
				// Avoid flicker by updating only the previous menu
				if(encoder_diffState == ENCODER_DIFF_CW)
				{
					if(select_axis < 4) select_axis++;
					else select_axis = 4;

					if(last_select_axis != select_axis)
					{
						DWIN_Draw_Rectangle(1, Background_black,	0,	31+(select_axis-1)*53,	14,	31+select_axis*53);
						DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31+select_axis*53);
						last_select_axis = select_axis;
					}
				}
				else if(encoder_diffState == ENCODER_DIFF_CCW)
				{
					if(select_axis != 0) select_axis--;
					else select_axis = 0;

					if(last_select_axis != select_axis)
					{
						DWIN_Draw_Rectangle(1, Background_black,	0,	31+(select_axis+1)*53,	14,	31+(select_axis+2)*53);
						DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31+select_axis*53);
						last_select_axis = select_axis;
					}
				}
				else if(encoder_diffState == ENCODER_DIFF_ENTER)
				{
					switch(select_axis)
					{
						case 0:	// back

							checkkey = Perpare;
							select_perpare = 1;
							index_perpare = 5;

							DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
							DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);

							if(HMI_flag.language_flag) 
							{
								DWIN_Frame_AreaCopy(1, 133, 1, 271-111, 479-465, 14, 8);
								DWIN_Frame_AreaCopy(1, 129, 72, 271-115, 479-395, 60, 49);

								DWIN_Frame_AreaCopy(1, 159, 70, 271-71, 479-395, 60, 102);
								DWIN_Frame_AreaCopy(1, 204, 70, 271-12, 479-397, 60, 155);
								DWIN_Frame_AreaCopy(1, 0, 89, 271-230, 479-378, 60, 208);
								#if HAS_BED_PROBE
									DWIN_Frame_AreaCopy(1, 174, 164, 271-48, 479-302, 60, 261);
									show_plus_or_minus(font8x16, Background_black, 2, 2, 202, 263-(index_perpare-5)*53, probe_offset.z*100);
								#else
									DWIN_Frame_AreaCopy(1, 43, 89, 271-173, 479-378, 60, 261);
								#endif
								DWIN_Frame_AreaCopy(1, 100, 89, 271-93-27, 479-378, 60, 314);
							}
							else 
							{
								DWIN_Frame_AreaCopy(1, 178, 2, 271-42, 479-464-1, 14, 8);
								DWIN_Frame_AreaCopy(1, 226, 179, 271-15, 479-290, 60, 49);

								DWIN_Frame_AreaCopy(1, 69, 61, 271-169, 479-408, 60, 102);
								DWIN_Frame_AreaCopy(1, 103, 59, 271-71, 479-405, 60, 155);
								DWIN_Frame_AreaCopy(1, 202, 61, 271-0, 479-408, 60, 208);
								#if HAS_BED_PROBE
									DWIN_Frame_AreaCopy(1, 93, 179, 271-130, 479-290, 60, 261);
									show_plus_or_minus(font8x16, Background_black, 2, 2, 202, 263-(index_perpare-5)*53, probe_offset.z*100);
								#else
									DWIN_Frame_AreaCopy(1, 1, 76, 271-165, 479-393, 60, 261);
								#endif
								DWIN_Frame_AreaCopy(1, 107, 76, 271-115, 479-393, 60, 314);
								DWIN_Frame_AreaCopy(1, 157, 76, 181, 479-393, 60+49+3, 314);									
							}

							DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 84);
							DWIN_ICON_Show(ICON,ICON_Back, 26, 46);
							DWIN_ICON_Show(ICON,ICON_More, 226, 99);
							DWIN_Draw_Line(Line_Color, 16, 81, 256, 83);
							for(short int i = 0; i < 5; i++) 
							{
								DWIN_ICON_Show(ICON,ICON_Axis+(index_perpare-5)+i, 26, 99+i*53);
								DWIN_Draw_Line(Line_Color, 16, 135+i*53, 256, 136+i*53);
							}

							break;
						case 1: // X axis move
							checkkey = Move_X;
							HMI_ValueStruct.Move_X_scale = current_position[X_AXIS]*MinUnitMult;
							DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Select_Color, 3, 1, 216, 104, HMI_ValueStruct.Move_X_scale);
							EncoderRate.encoderRateEnabled = 1;
							break;
						case 2: // Y axis move
							checkkey = Move_Y;
							HMI_ValueStruct.Move_Y_scale = current_position[Y_AXIS]*MinUnitMult;
							DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Select_Color, 3, 1, 216, 157, HMI_ValueStruct.Move_Y_scale);
							EncoderRate.encoderRateEnabled = 1;
							break;
						case 3: // Z axis move
							checkkey = Move_Z;
							HMI_ValueStruct.Move_Z_scale = current_position[Z_AXIS]*MinUnitMult;
							DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Select_Color, 3, 1, 216, 210, HMI_ValueStruct.Move_Z_scale);
							EncoderRate.encoderRateEnabled = 1;
							break;
						case 4: // Extruder
							// window tips
							#ifdef PREVENT_COLD_EXTRUSION
								if(thermalManager.temp_hotend[0].celsius < EXTRUDE_MINTEMP)
								{
									HMI_flag.ETempTooLow_flag = 1;
									Popup_Window_ETempTooLow();
									DWIN_UpdateLCD();
									return;
								}
							#endif							
							checkkey = Exturder;
							HMI_ValueStruct.Move_E_scale = current_position.e*MinUnitMult;
							show_plus_or_minus(font8x16, Select_Color, 3, 1, 216, 263, HMI_ValueStruct.Move_E_scale);
							EncoderRate.encoderRateEnabled = 1;
							break;
						default:
							break;
					}
				}
				DWIN_UpdateLCD();
			}
		}	
	}

  /* Temperation */
  void HMI_Temperation(void)
	{
		millis_t ms = millis();
		if(ms > Encoder_ms)
		{
			ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
			if(encoder_diffState != ENCODER_DIFF_NO)
			{
				Encoder_ms = ms + Encoder_wait;
				// Avoid flicker by updating only the previous menu
				if(encoder_diffState == ENCODER_DIFF_CW)
				{
					if(select_temp < 5) select_temp++;
					else select_temp = 5;

					if(last_select_temp != select_temp)
					{
						DWIN_Draw_Rectangle(1, Background_black,	0,	31+(select_temp-1)*53,	14,	31+select_temp*53);
						DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31+select_temp*53);
						last_select_temp = select_temp;
					}
				}
				else if(encoder_diffState == ENCODER_DIFF_CCW)
				{
					if(select_temp != 0) select_temp--;
					else select_temp = 0;

					if(last_select_temp != select_temp)
					{
						DWIN_Draw_Rectangle(1, Background_black,	0,	31+(select_temp+1)*53,	14,	31+(select_temp+2)*53);
						DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31+select_temp*53);
						last_select_temp = select_temp;
					}
				}
				else if(encoder_diffState == ENCODER_DIFF_ENTER)
				{
						switch(select_temp)
						{
							case 0:	// back

								checkkey = Control;
								select_control = 1;
								index_control = 5;

								DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
								DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);

								if(HMI_flag.language_flag) 
								{
									DWIN_Frame_AreaCopy(1, 103, 1, 271-141, 479-465, 14, 8);
									DWIN_Frame_AreaCopy(1, 129, 72, 271-115, 479-395, 60, 49);

									DWIN_Frame_AreaCopy(1, 57, 104, 271-187, 479-363, 60, 102);
									DWIN_Frame_AreaCopy(1, 87, 104, 271-157, 479-363, 60, 155);
									DWIN_Frame_AreaCopy(1, 117, 104, 271-99, 479-363, 60, 208);
									DWIN_Frame_AreaCopy(1, 174, 103, 271-42, 479-363, 60, 261);
									DWIN_Frame_AreaCopy(1, 1, 118, 271-215, 479-348, 60, 314);
								}
								else 
								{
									DWIN_Frame_AreaCopy(1, 128, 2, 271-95, 479-467, 14, 8);
									DWIN_Frame_AreaCopy(1, 226, 179, 271-15, 479-290, 60, 49);

									DWIN_Frame_AreaCopy(1, 1, 89, 271-188, 479-377-1, 60, 102);
									DWIN_Frame_AreaCopy(1, 84, 89, 271-143, 479-380, 60, 155);
									DWIN_Frame_AreaCopy(1, 131, 89, 271-3, 479-377, 60, 208);
									DWIN_Frame_AreaCopy(1, 26, 104, 271-214, 479-365, 60, 261);
									DWIN_Frame_AreaCopy(1, 182, 89, 271-3, 479-377-1, 60+31+3, 261);
									DWIN_Frame_AreaCopy(1, 59, 104, 271-178, 479-365, 60, 314);	
									DWIN_Frame_AreaCopy(1, 182, 89, 271-3, 479-377-1, 60+34+3, 314);									
								}

								DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 84);
								DWIN_ICON_Show(ICON,ICON_Back, 26, 46);
								DWIN_Draw_Line(Line_Color, 16, 82, 256, 83);
								for(short int i = 0; i < 5; i++) 
								{
									DWIN_ICON_Show(ICON,ICON_Temperation+i, 26, 99+i*53);
									DWIN_Draw_Line(Line_Color, 16, 135+i*53, 256, 136+i*53);
								}

								DWIN_ICON_Show(ICON,ICON_More, 226, 99);
								DWIN_ICON_Show(ICON,ICON_More, 226, 152);

								break;
							case 1: // nozzle temperation
								checkkey = ETemp;
								HMI_ValueStruct.E_Temp = thermalManager.temp_hotend[0].target;
								DWIN_Draw_IntVariate(true,true,0,font8x16,White,Select_Color, 3, 216, 104, thermalManager.temp_hotend[0].target);
								EncoderRate.encoderRateEnabled = 1;
								break;
							case 2: // bed temperation
								checkkey = BedTemp;
								HMI_ValueStruct.Bed_Temp = thermalManager.temp_bed.target;
								DWIN_Draw_IntVariate(true,true,0,font8x16,White,Select_Color, 3, 216, 157, thermalManager.temp_bed.target);
								EncoderRate.encoderRateEnabled = 1;
								break;
							case 3: // fan speed
								checkkey = FanSpeed;
								HMI_ValueStruct.Fan_speed = thermalManager.fan_speed[0];
								DWIN_Draw_IntVariate(true,true,0,font8x16,White,Select_Color, 3, 216, 210, thermalManager.fan_speed[0]);
								EncoderRate.encoderRateEnabled = 1;
								break;
							case 4: // PLA preheat setting
								
								checkkey = PLAPreheat;
								last_select_PLA = select_PLA = 0;
								HMI_ValueStruct.show_mode = -2;

								DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
								DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);

								if(HMI_flag.language_flag) 
								{
									DWIN_Frame_AreaCopy(1, 59, 16, 271-132, 479-450, 14, 8);
									DWIN_Frame_AreaCopy(1, 129, 72, 271-115, 479-395, 60, 49);

									DWIN_Frame_AreaCopy(1, 100, 89, 124, 479-378, 60, 102);
									DWIN_Frame_AreaCopy(1, 1, 134, 271-215, 479-333, 60+24, 102);		// PLA nozzle temp
									DWIN_Frame_AreaCopy(1, 100, 89, 124, 479-378, 60, 155);
									DWIN_Frame_AreaCopy(1, 58, 134, 271-158, 479-333, 60+24, 155);	// PLA bed temp				
									DWIN_Frame_AreaCopy(1, 100, 89, 124, 479-378, 60, 208);
									DWIN_Frame_AreaCopy(1, 115, 134, 271-99, 479-333, 60+24, 208);	// PLA fan speed
									DWIN_Frame_AreaCopy(1, 72, 148, 271-120, 479-317, 60, 261);			// save PLA configuration
								}
								else 
								{
									DWIN_Frame_AreaCopy(1, 56, 16, 139, 479-450-1, 14, 8);
									DWIN_Frame_AreaCopy(1, 226, 179, 271-15, 479-290, 60, 49);
									
									DWIN_Frame_AreaCopy(1, 157, 76, 181, 479-393, 60, 102);			
									DWIN_Frame_AreaCopy(1, 197, 104, 271-31, 479-365, 60+24+3, 102);
									DWIN_Frame_AreaCopy(1, 1, 89, 271-188, 479-377-1, 60+24+41+6, 102);	// PLA nozzle temp
									DWIN_Frame_AreaCopy(1, 157, 76, 181, 479-393, 60, 158);
									DWIN_Frame_AreaCopy(1, 240, 104, 271-7, 479-365, 60+24+3, 158);
									DWIN_Frame_AreaCopy(1, 1, 89, 271-188, 479-377-1, 60+24+24+6, 158);	// PLA bed temp
									DWIN_Frame_AreaCopy(1, 157, 76, 181, 479-393, 60, 208);
									DWIN_Frame_AreaCopy(1, 0, 119, 271-207, 479-347, 60+24+3, 208);		// PLA fan speed
									DWIN_Frame_AreaCopy(1, 97, 165, 271-42, 479-301-1, 60, 261);			// save PLA configuration
								}

								DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31);
								DWIN_ICON_Show(ICON,ICON_Back, 26, 46);
								for(short int i = 0; i < 5; i++) 
								{
									DWIN_Draw_Line(Line_Color, 16, 82+i*53, 256, 83+i*53);
								}

								DWIN_ICON_Show(ICON,ICON_SetEndTemp, 26, 99);
								DWIN_ICON_Show(ICON,ICON_SetBedTemp, 26, 152);
								DWIN_ICON_Show(ICON,ICON_FanSpeed, 26, 205);
								DWIN_ICON_Show(ICON,ICON_WriteEEPROM, 26, 258);

								DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 104, HMI_ValueStruct.preheat_hotend_temp[0]);
								DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 157, HMI_ValueStruct.preheat_bed_temp[0]);
								DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 210, HMI_ValueStruct.preheat_fan_speed[0]);

								break;
							case 5: // ABS preheat setting

								checkkey = ABSPreheat;
								last_select_ABS = select_ABS = 0;
								HMI_ValueStruct.show_mode = -3;

								DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
								DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);

								if(HMI_flag.language_flag) 
								{
									DWIN_Frame_AreaCopy(1, 142, 16, 271-48, 479-450, 14, 8);
									DWIN_Frame_AreaCopy(1, 129, 72, 271-115, 479-395, 60, 49);

									DWIN_Frame_AreaCopy(1, 180, 89, 204, 479-379, 60, 102);
									DWIN_Frame_AreaCopy(1, 1, 134, 271-215, 479-333, 60+24, 102);		// ABS nozzle temp
									DWIN_Frame_AreaCopy(1, 180, 89, 204, 479-379, 60, 155);
									DWIN_Frame_AreaCopy(1, 58, 134, 271-158, 479-333, 60+24, 155);	// ABS bed temp
									DWIN_Frame_AreaCopy(1, 180, 89, 204, 479-379, 60, 208);
									DWIN_Frame_AreaCopy(1, 115, 134, 271-99, 479-333, 60+24, 208);	// ABS fan speed
									DWIN_Frame_AreaCopy(1, 72, 148, 271-120, 479-317, 60, 261);	
									DWIN_Frame_AreaCopy(1, 180, 89, 204, 479-379, 60+28, 261+2);			// save ABS configuration
								}
								else 
								{
									DWIN_Frame_AreaCopy(1, 56, 16, 139, 479-450-1, 14, 8);
									DWIN_Frame_AreaCopy(1, 226, 179, 271-15, 479-290, 60, 49);

									DWIN_Frame_AreaCopy(1, 172, 76, 198, 479-393, 60, 102);
									DWIN_Frame_AreaCopy(1, 197, 104, 271-31, 479-365, 60+24+3, 102);
									DWIN_Frame_AreaCopy(1, 1, 89, 271-188, 479-377-1, 60+24+41+6, 102);	// ABS nozzle temp
									DWIN_Frame_AreaCopy(1, 172, 76, 198, 479-393, 60, 158);
									DWIN_Frame_AreaCopy(1, 240, 104, 271-7, 479-365, 60+24+3, 158);
									DWIN_Frame_AreaCopy(1, 1, 89, 271-188, 479-377-1, 60+24+24+6, 158);	// ABS bed temp
									DWIN_Frame_AreaCopy(1, 172, 76, 198, 479-393, 60, 208);
									DWIN_Frame_AreaCopy(1, 0, 119, 271-207, 479-347, 60+24+3, 208);		// ABS fan speed
									DWIN_Frame_AreaCopy(1, 97, 165, 271-42, 479-301-1, 60, 261);			
									DWIN_Frame_AreaCopy(1, 172, 76, 198, 479-393, 60+33, 261);					// save ABS configuration
								}

								DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31);
								DWIN_ICON_Show(ICON,ICON_Back, 26, 46);
								for(short int i = 0; i < 5; i++) 
								{
									DWIN_Draw_Line(Line_Color, 16, 82+i*53, 256, 83+i*53);
								}

								DWIN_ICON_Show(ICON,ICON_SetEndTemp, 26, 99);
								DWIN_ICON_Show(ICON,ICON_SetBedTemp, 26, 152);
								DWIN_ICON_Show(ICON,ICON_FanSpeed, 26, 205);
								DWIN_ICON_Show(ICON,ICON_WriteEEPROM, 26, 258);

								DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 104, HMI_ValueStruct.preheat_hotend_temp[1]);
								DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 157, HMI_ValueStruct.preheat_bed_temp[1]);
								DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 210, HMI_ValueStruct.preheat_fan_speed[1]);

								break;
							default:
								break;
						}
				}
				DWIN_UpdateLCD();
			}
		}	
	}

  /* Motion */
  void HMI_Motion(void)
	{
		millis_t ms = millis();
		if(ms > Encoder_ms)
		{
			ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
			if(encoder_diffState != ENCODER_DIFF_NO)
			{
				Encoder_ms = ms + Encoder_wait;
				// Avoid flicker by updating only the previous menu
				if(encoder_diffState == ENCODER_DIFF_CW)
				{
					if(select_motion < 4) select_motion++;
					else select_motion = 4;

					if(last_select_motion != select_motion)
					{
						DWIN_Draw_Rectangle(1, Background_black,	0,	31+(select_motion-1)*53,	14,	31+select_motion*53);
						DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31+select_motion*53);
						last_select_motion = select_motion;
					}
				}
				else if(encoder_diffState == ENCODER_DIFF_CCW)
				{
					if(select_motion != 0) select_motion--;
					else select_motion = 0;

					if(last_select_motion != select_motion)
					{
						DWIN_Draw_Rectangle(1, Background_black,	0,	31+(select_motion+1)*53,	14,	31+(select_motion+2)*53);
						DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31+select_motion*53);
						last_select_motion = select_motion;
					}
				}
				else if(encoder_diffState == ENCODER_DIFF_ENTER)
				{
						switch(select_motion)
						{
							case 0:	// back

								checkkey = Control;
								select_control = 2;
								index_control = 5;

								DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
								DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);

								if(HMI_flag.language_flag) 
								{
									DWIN_Frame_AreaCopy(1, 103, 1, 271-141, 479-465, 14, 8);
									DWIN_Frame_AreaCopy(1, 129, 72, 271-115, 479-395, 60, 49);

									DWIN_Frame_AreaCopy(1, 57, 104, 271-187, 479-363, 60, 102);
									DWIN_Frame_AreaCopy(1, 87, 104, 271-157, 479-363, 60, 155);
									DWIN_Frame_AreaCopy(1, 117, 104, 271-99, 479-363, 60, 208);
									DWIN_Frame_AreaCopy(1, 174, 103, 271-42, 479-363, 60, 261);
									DWIN_Frame_AreaCopy(1, 1, 118, 271-215, 479-348, 60, 314);
								}
								else 
								{
									DWIN_Frame_AreaCopy(1, 128, 2, 271-95, 479-467, 14, 8);
									DWIN_Frame_AreaCopy(1, 226, 179, 271-15, 479-290, 60, 49);

									DWIN_Frame_AreaCopy(1, 1, 89, 271-188, 479-377-1, 60, 102);
									DWIN_Frame_AreaCopy(1, 84, 89, 271-143, 479-380, 60, 155);
									DWIN_Frame_AreaCopy(1, 131, 89, 271-3, 479-377, 60, 208);
									DWIN_Frame_AreaCopy(1, 26, 104, 271-214, 479-365, 60, 261);
									DWIN_Frame_AreaCopy(1, 182, 89, 271-3, 479-377-1, 60+31+3, 261);
									DWIN_Frame_AreaCopy(1, 59, 104, 271-178, 479-365, 60, 314);	
									DWIN_Frame_AreaCopy(1, 182, 89, 271-3, 479-377-1, 60+34+3, 314);
								}

								DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 137);
								DWIN_ICON_Show(ICON,ICON_Back, 26, 46);
								DWIN_Draw_Line(Line_Color, 16, 82, 256, 83);
								for(short int i = 0; i < 5; i++) 
								{
									DWIN_ICON_Show(ICON,ICON_Temperation+i, 26, 99+i*53);
									DWIN_Draw_Line(Line_Color, 16, 135+i*53, 256, 136+i*53);
								}

								DWIN_ICON_Show(ICON,ICON_More, 226, 99);
								DWIN_ICON_Show(ICON,ICON_More, 226, 152);

								break;
							case 1: // max speed

								checkkey = MaxSpeed;
								last_select_speed = select_speed = 0;

								DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
								DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);

								if(HMI_flag.language_flag) 
								{
									DWIN_Frame_AreaCopy(1, 1, 16, 271-243, 479-451, 14, 8);
									DWIN_Frame_AreaCopy(1, 129, 72, 271-115, 479-395, 60, 49);

									DWIN_Frame_AreaCopy(1, 173, 133, 228, 479-332, 60, 102);	
									DWIN_Frame_AreaCopy(1, 229, 133, 236, 479-332, 60+55+3, 102);			// max speed X
									DWIN_Frame_AreaCopy(1, 173, 133, 228, 479-332, 60, 155);	
									DWIN_Frame_AreaCopy(1, 1, 150, 271-264, 479-319, 60+55+3, 155+3);		// max speed Y
									DWIN_Frame_AreaCopy(1, 173, 133, 228, 479-332, 60, 208);	
									DWIN_Frame_AreaCopy(1, 9, 150, 271-255, 479-319, 60+55+3, 208+3);		// max speed Z
									DWIN_Frame_AreaCopy(1, 173, 133, 228, 479-332, 60, 261);	
									DWIN_Frame_AreaCopy(1, 18, 150, 271-246, 479-319, 60+55+3, 261+3);	// max speed E
								}
								else
								{
									DWIN_Frame_AreaCopy(1, 144, 16, 271-82, 479-453, 14, 8);
									DWIN_Frame_AreaCopy(1, 226, 179, 271-15, 479-290, 60, 49);

									DWIN_Frame_AreaCopy(1, 245, 119, 271-2, 479-350, 60, 102);					
									DWIN_Frame_AreaCopy(1, 184, 119, 271-37, 479-347, 60+24+3, 102);				// max speed X
									DWIN_Frame_AreaCopy(1, 245, 119, 271-2, 479-350, 60, 155);					
									DWIN_Frame_AreaCopy(1, 184, 119, 224, 479-347, 60+24+3, 155);
									DWIN_Frame_AreaCopy(1, 104, 104, 271-161, 479-365, 60+24+40+6, 155);		// max speed Y
									DWIN_Frame_AreaCopy(1, 245, 119, 271-2, 479-350, 60, 208);					
									DWIN_Frame_AreaCopy(1, 184, 119, 224, 479-347, 60+24+3, 208);
									DWIN_Frame_AreaCopy(1, 112, 104, 271-151, 479-365, 60+24+40+6, 208);		// max speed Z
									DWIN_Frame_AreaCopy(1, 245, 119, 271-2, 479-350, 60, 261);					
									DWIN_Frame_AreaCopy(1, 184, 119, 224, 479-347, 60+24+3, 261);
									DWIN_Frame_AreaCopy(1, 237, 119, 271-27, 479-350, 60+24+40+6, 261);			// max speed E

								}

								DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31);
								DWIN_ICON_Show(ICON,ICON_Back, 26, 46);
								DWIN_Draw_Line(Line_Color, 16, 82, 256, 83);
								for(short int i = 0; i < 4; i++) 
								{
									DWIN_ICON_Show(ICON,ICON_MaxSpeedX+i, 26, 99+i*53);									
									DWIN_Draw_Line(Line_Color, 16, 135+i*53, 256, 136+i*53);
								}

								DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 4, 210, 104, planner.settings.max_feedrate_mm_s[X_AXIS]);
								DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 4, 210, 157, planner.settings.max_feedrate_mm_s[Y_AXIS]);
								DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 4, 210, 210, planner.settings.max_feedrate_mm_s[Z_AXIS]);
								DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 4, 210, 263, planner.settings.max_feedrate_mm_s[E_AXIS]);

								break;
							case 2: // max acceleration

								checkkey = MaxAcceleration;
								last_select_acc = select_acc = 0;

								DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
								DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);

								if(HMI_flag.language_flag) 
								{
									DWIN_Frame_AreaCopy(1, 1, 16, 271-243, 479-451, 14, 8);
									DWIN_Frame_AreaCopy(1, 129, 72, 271-115, 479-395, 60, 49);

									DWIN_Frame_AreaCopy(1, 173, 133, 200, 479-332, 60, 102);
									DWIN_Frame_AreaCopy(1, 28, 149, 271-202, 479-318, 60+27, 102+1);
									DWIN_Frame_AreaCopy(1, 229, 133, 236, 479-332, 60+27+41+3, 102);		// max acceleration X
									DWIN_Frame_AreaCopy(1, 173, 133, 200, 479-332, 60, 155);
									DWIN_Frame_AreaCopy(1, 28, 149, 271-202, 479-318, 60+27, 155+1);
									DWIN_Frame_AreaCopy(1, 1, 150, 271-264, 479-319, 60+27+41+3, 155+2);	// max acceleration Y
									DWIN_Frame_AreaCopy(1, 173, 133, 200, 479-332, 60, 208);
									DWIN_Frame_AreaCopy(1, 28, 149, 271-202, 479-318, 60+27, 208+1);
									DWIN_Frame_AreaCopy(1, 9, 150, 271-255, 479-319, 60+27+41+3, 208+2);	// max acceleration Z
									DWIN_Frame_AreaCopy(1, 173, 133, 200, 479-332, 60, 261);
									DWIN_Frame_AreaCopy(1, 28, 149, 271-202, 479-318, 60+27, 261+1);
									DWIN_Frame_AreaCopy(1, 18, 150, 271-246, 479-319, 60+27+41+3, 261+2);	// max acceleration E
								}
								else 
								{
									DWIN_Frame_AreaCopy(1, 144, 16, 271-82, 479-453, 14, 8);
									DWIN_Frame_AreaCopy(1, 226, 179, 271-15, 479-290, 60, 49);
									
									DWIN_Frame_AreaCopy(1, 245, 119, 271-2, 479-350, 60, 102);	
									DWIN_Frame_AreaCopy(1, 1, 135, 271-192, 479-334, 60+24+3, 102);		
									DWIN_Frame_AreaCopy(1, 95, 104, 271-169, 479-365, 60+24+78+6, 102);		// max acceleration X
									DWIN_Frame_AreaCopy(1, 245, 119, 271-2, 479-350, 60, 155);	
									DWIN_Frame_AreaCopy(1, 1, 135, 271-192, 479-334, 60+24+3, 155);		
									DWIN_Frame_AreaCopy(1, 104, 104, 271-161, 479-365, 60+24+78+6, 155);	// max acceleration Y
									DWIN_Frame_AreaCopy(1, 245, 119, 271-2, 479-350, 60, 208);	
									DWIN_Frame_AreaCopy(1, 1, 135, 271-192, 479-334, 60+24+3, 208);		
									DWIN_Frame_AreaCopy(1, 112, 104, 271-151, 479-365, 60+24+78+6, 208);	// max acceleration Z
									DWIN_Frame_AreaCopy(1, 245, 119, 271-2, 479-350, 60, 261);	
									DWIN_Frame_AreaCopy(1, 1, 135, 271-192, 479-334, 60+24+3, 261);		
									DWIN_Frame_AreaCopy(1, 237, 119, 271-27, 479-350, 60+24+78+6, 261);		// max acceleration E
								}

								DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31);
								DWIN_ICON_Show(ICON,ICON_Back, 26, 46);
								DWIN_Draw_Line(Line_Color, 16, 82, 256, 83);
								for(short int i = 0; i < 4; i++) 
								{
									DWIN_ICON_Show(ICON,ICON_MaxAccX+i, 26, 99+i*53);
									DWIN_Draw_Line(Line_Color, 16, 135+i*53, 256, 136+i*53);
								}

								DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 4, 210, 104, planner.settings.max_acceleration_mm_per_s2[X_AXIS]);
								DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 4, 210, 157, planner.settings.max_acceleration_mm_per_s2[Y_AXIS]);
								DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 4, 210, 210, planner.settings.max_acceleration_mm_per_s2[Z_AXIS]);
								DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 4, 210, 263, planner.settings.max_acceleration_mm_per_s2[E_AXIS]);

								break;
							case 3: // max corner speed

								checkkey = MaxCorner;
								last_select_corner = select_corner = 0;

								DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
								DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);

								if(HMI_flag.language_flag) 
								{
									DWIN_Frame_AreaCopy(1, 1, 16, 271-243, 479-451, 14, 8);
									DWIN_Frame_AreaCopy(1, 129, 72, 271-115, 479-395, 60, 49);

									DWIN_Frame_AreaCopy(1, 173, 133, 200, 479-332, 60, 102);
									DWIN_Frame_AreaCopy(1, 1, 180, 271-243, 479-287, 60+27, 102+1);
									DWIN_Frame_AreaCopy(1, 202, 133, 228, 479-332, 60+53, 102);				
									DWIN_Frame_AreaCopy(1, 229, 133, 236, 479-332, 60+80+3, 102);			// max corner speed X
									DWIN_Frame_AreaCopy(1, 173, 133, 200, 479-332, 60, 155);
									DWIN_Frame_AreaCopy(1, 1, 180, 271-243, 479-287, 60+27, 155+1);
									DWIN_Frame_AreaCopy(1, 202, 133, 228, 479-332, 60+53, 155);				
									DWIN_Frame_AreaCopy(1, 1, 150, 271-264, 479-319, 60+80+3, 155+3);		// max corner speed Y
									DWIN_Frame_AreaCopy(1, 173, 133, 200, 479-332, 60, 208);
									DWIN_Frame_AreaCopy(1, 1, 180, 271-243, 479-287, 60+27, 208+1);
									DWIN_Frame_AreaCopy(1, 202, 133, 228, 479-332, 60+53, 208);				
									DWIN_Frame_AreaCopy(1, 9, 150, 271-255, 479-319, 60+80+3, 208+3);		// max corner speed Z
									DWIN_Frame_AreaCopy(1, 173, 133, 200, 479-332, 60, 261);
									DWIN_Frame_AreaCopy(1, 1, 180, 271-243, 479-287, 60+27, 261+1);
									DWIN_Frame_AreaCopy(1, 202, 133, 228, 479-332, 60+53, 261);				
									DWIN_Frame_AreaCopy(1, 18, 150, 271-246, 479-319, 60+80+3, 261+3);	// max corner speed E
								}
								else 
								{
									DWIN_Frame_AreaCopy(1, 144, 16, 271-82, 479-453, 14, 8);
									DWIN_Frame_AreaCopy(1, 226, 179, 271-15, 479-290, 60, 49);
									
									DWIN_Frame_AreaCopy(1, 245, 119, 271-2, 479-350, 60, 102);
									DWIN_Frame_AreaCopy(1, 64, 119, 271-165, 479-350, 60+24+3, 102);
									DWIN_Frame_AreaCopy(1, 184, 119, 224, 479-347, 60+66+6, 102);
									DWIN_Frame_AreaCopy(1, 95, 104, 271-169, 479-365, 60+106+9, 102);		// max corner X
									DWIN_Frame_AreaCopy(1, 245, 119, 271-2, 479-350, 60, 155);
									DWIN_Frame_AreaCopy(1, 64, 119, 271-165, 479-350, 60+24+3, 155);
									DWIN_Frame_AreaCopy(1, 184, 119, 224, 479-347, 60+66+6, 155);
									DWIN_Frame_AreaCopy(1, 104, 104, 271-161, 479-365, 60+106+9, 155);	// max corner Y
									DWIN_Frame_AreaCopy(1, 245, 119, 271-2, 479-350, 60, 208);
									DWIN_Frame_AreaCopy(1, 64, 119, 271-165, 479-350, 60+24+3, 208);
									DWIN_Frame_AreaCopy(1, 184, 119, 224, 479-347, 60+66+6, 208);
									DWIN_Frame_AreaCopy(1, 112, 104, 271-151, 479-365, 60+106+9, 208);	// max corner Z
									DWIN_Frame_AreaCopy(1, 245, 119, 271-2, 479-350, 60, 261);
									DWIN_Frame_AreaCopy(1, 64, 119, 271-165, 479-350, 60+24+3, 261);
									DWIN_Frame_AreaCopy(1, 184, 119, 224, 479-347, 60+66+6, 261);
									DWIN_Frame_AreaCopy(1, 237, 119, 271-27, 479-350, 60+106+9, 261);		// max corner E
								}

								DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31);
								DWIN_ICON_Show(ICON,ICON_Back, 26, 46);
								DWIN_Draw_Line(Line_Color, 16, 82, 256, 83);
								for(short int i = 0; i < 4; i++) 
								{
									DWIN_ICON_Show(ICON,ICON_MaxSpeedCornerX+i, 26, 99+i*53);
									DWIN_Draw_Line(Line_Color, 16, 135+i*53, 256, 136+i*53);
								}

								DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Background_black, 3, 1, 210, 104, planner.max_jerk[X_AXIS]*MinUnitMult);
								DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Background_black, 3, 1, 210, 157, planner.max_jerk[Y_AXIS]*MinUnitMult);
								DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Background_black, 3, 1, 210, 210, planner.max_jerk[Z_AXIS]*MinUnitMult);
								DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Background_black, 3, 1, 210, 263, planner.max_jerk[E_AXIS]*MinUnitMult);

								break;
							case 4: // transmission ratio

								checkkey = Step;
								last_select_step = select_step = 0;

								DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
								DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);

								if(HMI_flag.language_flag) 
								{
									DWIN_Frame_AreaCopy(1, 1, 16, 271-243, 479-451, 14, 8);
									DWIN_Frame_AreaCopy(1, 129, 72, 271-115, 479-395, 60, 49);

									DWIN_Frame_AreaCopy(1, 153, 148, 271-77, 479-318, 60, 102);
									DWIN_Frame_AreaCopy(1, 229, 133, 236, 479-332, 60+41+3, 102);			// transmission ratio X
									DWIN_Frame_AreaCopy(1, 153, 148, 271-77, 479-318, 60, 155);
									DWIN_Frame_AreaCopy(1, 1, 150, 271-264, 479-319, 60+41+3, 155+3);		// transmission ratio Y
									DWIN_Frame_AreaCopy(1, 153, 148, 271-77, 479-318, 60, 208);
									DWIN_Frame_AreaCopy(1, 9, 150, 271-255, 479-319, 60+41+3, 208+3);		// transmission ratio Z
									DWIN_Frame_AreaCopy(1, 153, 148, 271-77, 479-318, 60, 261);
									DWIN_Frame_AreaCopy(1, 18, 150, 271-246, 479-319, 60+41+3, 261+3);	// transmission ratio E
								}
								else 
								{
									DWIN_Frame_AreaCopy(1, 144, 16, 271-82, 479-453, 14, 8);
									DWIN_Frame_AreaCopy(1, 226, 179, 271-15, 479-290, 60, 49);
									
									DWIN_Frame_AreaCopy(1, 1, 151, 271-154, 479-318, 60, 102);			
									DWIN_Frame_AreaCopy(1, 95, 104, 271-169, 479-365, 60+116+3, 102);	// transmission ratio X
									DWIN_Frame_AreaCopy(1, 1, 151, 271-154, 479-318, 60, 155);
									DWIN_Frame_AreaCopy(1, 104, 104, 271-161, 479-365, 60+116+3, 155);// transmission ratio Y
									DWIN_Frame_AreaCopy(1, 1, 151, 271-154, 479-318, 60, 208);
									DWIN_Frame_AreaCopy(1, 112, 104, 271-151, 479-365, 60+116+3, 208);// transmission ratio Z
									DWIN_Frame_AreaCopy(1, 1, 151, 271-154, 479-318, 60, 261);
									DWIN_Frame_AreaCopy(1, 237, 119, 271-27, 479-350, 60+116+3, 261);	// transmission ratio E
								}

								DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31);
								DWIN_ICON_Show(ICON,ICON_Back, 26, 46);
								DWIN_Draw_Line(Line_Color, 16, 82, 256, 83);
								for(short int i = 0; i < 4; i++) 
								{
									DWIN_ICON_Show(ICON,ICON_StepX+i, 26, 99+i*53);
									DWIN_Draw_Line(Line_Color, 16, 135+i*53, 256, 136+i*53);
								}

								DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Background_black, 3, 1, 210, 104, planner.settings.axis_steps_per_mm[X_AXIS]*MinUnitMult);
								DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Background_black, 3, 1, 210, 157, planner.settings.axis_steps_per_mm[Y_AXIS]*MinUnitMult);
								DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Background_black, 3, 1, 210, 210, planner.settings.axis_steps_per_mm[Z_AXIS]*MinUnitMult);
								DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Background_black, 3, 1, 210, 263, planner.settings.axis_steps_per_mm[E_AXIS]*MinUnitMult);

								break;
							default:
								break;
						}
				}
				DWIN_UpdateLCD();
			}
		}	
	}

  /* Info */
  void HMI_Info(void)
	{
		millis_t ms = millis();
		if(ms > Encoder_ms)
		{
			ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
			if(encoder_diffState != ENCODER_DIFF_NO)
			{
				Encoder_ms = ms + Encoder_wait;
				if(encoder_diffState == ENCODER_DIFF_ENTER)
				{
					#if HAS_LEVELING
						checkkey = Control;
						select_control = 6;

						DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
						DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);

						if(HMI_flag.language_flag) 
						{
							DWIN_Frame_AreaCopy(1, 103, 1, 271-141, 479-465, 14, 8);

							DWIN_Frame_AreaCopy(1, 57, 104, 271-187, 479-363, 60, 49);
							DWIN_Frame_AreaCopy(1, 87, 104, 271-157, 479-363, 60, 102);
							DWIN_Frame_AreaCopy(1, 117, 104, 271-99, 479-363, 60, 155);
							DWIN_Frame_AreaCopy(1, 174, 103, 271-42, 479-363, 60, 208);
							DWIN_Frame_AreaCopy(1, 1, 118, 271-215, 479-348, 60, 261);
							DWIN_Frame_AreaCopy(1, 231, 103, 271-13, 479-363, 60, 314);
						}
						else 
						{
							DWIN_Frame_AreaCopy(1, 128, 2, 271-95, 479-467, 14, 8);

							DWIN_Frame_AreaCopy(1, 1, 89, 271-188, 479-377-1, 60, 49);
							DWIN_Frame_AreaCopy(1, 84, 89, 271-143, 479-380, 60, 102);
							DWIN_Frame_AreaCopy(1, 131, 89, 271-3, 479-377, 60, 155);
							DWIN_Frame_AreaCopy(1, 26, 104, 271-214, 479-365, 60, 208);
							DWIN_Frame_AreaCopy(1, 182, 89, 271-3, 479-377-1, 60+31+3, 208);
							DWIN_Frame_AreaCopy(1, 59, 104, 271-178, 479-365, 60, 261);	
							DWIN_Frame_AreaCopy(1, 182, 89, 271-3, 479-377-1, 60+34+3, 261);		
							DWIN_Frame_AreaCopy(1, 0, 104, 271-247, 479-365, 60, 314);						
						}

						DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 296);
						for(short int i = 0; i < 6; i++) 
						{
							DWIN_ICON_Show(ICON,ICON_Temperation+i, 26, 46+i*53);
							DWIN_Draw_Line(Line_Color, 16, 82+i*53, 256, 83+i*53);
						}

						DWIN_ICON_Show(ICON,ICON_More, 226, 311);

					#else
						select_page = 3;
						Goto_ProcessFrame();
					#endif
				}
				DWIN_UpdateLCD();
			}
		}	
	}

  /* Tune */
  void HMI_Tune(void)
	{
		millis_t ms = millis();
		if(ms > Encoder_ms)
		{
			ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
			if(encoder_diffState != ENCODER_DIFF_NO)
			{
				Encoder_ms = ms + Encoder_wait;
				// Avoid flicker by updating only the previous menu
				if(encoder_diffState == ENCODER_DIFF_CW)
				{
					if(select_tune < 6) select_tune++;
					else select_tune = 6;
					if(last_select_tune != select_tune)
					{
						if(select_tune > 5 && select_tune > index_tune)
						{
							DWIN_Frame_AreaMove(1, 2, 53, Background_black, 0, 31, 272, 349);
							DWIN_ICON_Show(ICON,ICON_Language, 26, 311);
							index_tune = select_tune;
							DWIN_Draw_Rectangle(1, Background_black,	0,	243,	14,	296);
							DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 296);
							DWIN_Draw_Line(Line_Color, 16, 347, 256, 348);
							last_select_tune = select_tune;
							if(HMI_flag.language_flag)
								{DWIN_Frame_AreaCopy(1, 239, 134, 271-5, 479-333, 60, 314); DWIN_Draw_String(false,false,font8x16,White,Background_black, 226, 314, (char*)"CN");}
							else
								{DWIN_Frame_AreaCopy(1, 0, 194, 271-150, 479-272, 60, 314); DWIN_Draw_String(false,false,font8x16,White,Background_black, 226, 314, (char*)"EN");}
						}
						else
						{							
							DWIN_Draw_Rectangle(1, Background_black,	0,	31+((select_tune-(index_tune-5))-1)*53,	14,	31+(select_tune-(index_tune-5))*53);
							DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31+(select_tune-(index_tune-5))*53);
							last_select_tune = select_tune;
						}
					}
				}
				else if(encoder_diffState == ENCODER_DIFF_CCW)
				{
					if(select_tune != 0) select_tune--;
					else select_tune = 0;
					if(last_select_tune != select_tune)
					{
						if(select_tune < index_tune-5)
						{
							DWIN_Frame_AreaMove(1, 3, 53, Background_black, 0, 31, 272, 349);
							DWIN_ICON_Show(ICON,ICON_Back, 26, 46);
							index_tune--;
							DWIN_Draw_Rectangle(1, Background_black,	0,	84,	14,	137);
							DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31);
							DWIN_Draw_Line(Line_Color, 16, 82, 256, 83);
							last_select_tune = select_tune;
							if(index_control == 5) 
							{
								if(HMI_flag.language_flag)
									DWIN_Frame_AreaCopy(1, 129, 72, 271-115, 479-395, 60, 49);
								else
									DWIN_Frame_AreaCopy(1, 226, 179, 271-15, 479-290, 60, 49);
							}
						}
						else
						{						
							DWIN_Draw_Rectangle(1, Background_black,	0,	31+((select_tune-(index_tune-5))+1)*53,	14,	31+((select_tune-(index_tune-5))+2)*53);
							DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31+(select_tune-(index_tune-5))*53);
							last_select_tune = select_tune;
						}
					}
				}
				else if(encoder_diffState == ENCODER_DIFF_ENTER)
				{
					switch(select_tune)
					{
						case 0:	// back
							select_print = 0;
							Goto_PrintProcess();
							DWIN_Draw_String(false,false,font8x16,White,Background_black, 136-(strlen(filebuf)*8)/2, 60, filebuf);	// filename
							break;
						case 1: // print speed
							checkkey = PrintSpeed;
							HMI_ValueStruct.print_speed = feedrate_percentage;
							DWIN_Draw_IntVariate(true,true,0,font8x16,White,Select_Color, 3, 216, 104-(index_tune-5)*53, feedrate_percentage);
							EncoderRate.encoderRateEnabled = 1;
							break;
						case 2: // nozzle temp
							checkkey = ETemp;
							HMI_ValueStruct.E_Temp = thermalManager.temp_hotend[0].target;
							DWIN_Draw_IntVariate(true,true,0,font8x16,White,Select_Color, 3, 216, 157-(index_tune-5)*53, thermalManager.temp_hotend[0].target);
							EncoderRate.encoderRateEnabled = 1;
							break;
						case 3: // bed temp
							checkkey = BedTemp;
							HMI_ValueStruct.Bed_Temp = thermalManager.temp_bed.target;
							DWIN_Draw_IntVariate(true,true,0,font8x16,White,Select_Color, 3, 216, 210-(index_tune-5)*53, thermalManager.temp_bed.target);
							EncoderRate.encoderRateEnabled = 1;
							break;
						case 4: // fan speed
							checkkey = FanSpeed;
							HMI_ValueStruct.Fan_speed = thermalManager.fan_speed[0];
							DWIN_Draw_IntVariate(true,true,0,font8x16,White,Select_Color, 3, 216, 263-(index_tune-5)*53, thermalManager.fan_speed[0]);
							EncoderRate.encoderRateEnabled = 1;
							break;
						case 5: // z-offest
							checkkey = Homeoffset;
							#if HAS_LEVELING
								HMI_ValueStruct.offest_value = probe_offset.z*100;
							#else
								HMI_ValueStruct.offest_value = zprobe_zoffset*100;
							#endif
							show_plus_or_minus(font8x16, Select_Color, 2, 2, 202, 316-(index_tune-5)*53, HMI_ValueStruct.offest_value);
							EncoderRate.encoderRateEnabled = 1;
							break;
						case 6: // language
							/* select language */
							HMI_flag.language_flag = !HMI_flag.language_flag;

							DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
							DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);

							if(HMI_flag.language_flag) 
							{
								set_chinese_to_eeprom();
								DWIN_JPG_CacheTo1(Language_Chinese);
								
								DWIN_Frame_AreaCopy(1, 73, 2, 271-171, 479-466, 14, 9);

								DWIN_Frame_AreaCopy(1, 116, 164, 271-100, 479-303, 60, 49);
								DWIN_Frame_AreaCopy(1, 1, 134, 271-215, 479-333, 60, 102);
								DWIN_Frame_AreaCopy(1, 58, 134, 271-158, 479-333, 60, 155);
								DWIN_Frame_AreaCopy(1, 115, 134, 271-99, 479-333, 60, 208);
								DWIN_Frame_AreaCopy(1, 174, 164, 271-48, 479-302, 60, 261);
								{DWIN_Frame_AreaCopy(1, 239, 134, 271-5, 479-333, 60, 314); DWIN_Draw_String(false,false,font8x16,White,Background_black, 226, 314, (char*)"CN");}
							}
							else 
							{
								set_english_to_eeprom();
								DWIN_JPG_CacheTo1(Language_English);

								DWIN_Frame_AreaCopy(1, 94, 2, 271-145, 479-467, 14, 9);

								DWIN_Frame_AreaCopy(1, 1, 179, 271-179, 479-287, 60, 49);			// print speed
								DWIN_Frame_AreaCopy(1, 197, 104, 271-31, 479-365, 60, 102);
								DWIN_Frame_AreaCopy(1, 1, 89, 271-188, 479-377-1, 60+41+3, 102);	// nozzle temp
								DWIN_Frame_AreaCopy(1, 240, 104, 271-7, 479-365, 60, 155);
								DWIN_Frame_AreaCopy(1, 1, 89, 271-188, 479-377-1, 60+24+3, 155);	// bed temp
								DWIN_Frame_AreaCopy(1, 0, 119, 271-207, 479-347, 60, 208);		// fan temp
								DWIN_Frame_AreaCopy(1, 93, 179, 271-130, 479-290, 60, 261);		// Z-offest
								{DWIN_Frame_AreaCopy(1, 0, 194, 271-150, 479-272, 60, 314); DWIN_Draw_String(false,false,font8x16,White,Background_black, 226, 314, (char*)"EN");}		// language
							}

							DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 296);
							for(short int i = 0; i < 6; i++) 
							{
								DWIN_Draw_Line(Line_Color, 16, 82+i*53, 256, 83+i*53);
							}

							DWIN_ICON_Show(ICON,ICON_HotendTemp, 26, 46);
							DWIN_ICON_Show(ICON,ICON_BedTemp, 26, 99);
							DWIN_ICON_Show(ICON,ICON_FanSpeed, 26, 152);
							DWIN_ICON_Show(ICON,ICON_Speed, 26, 205);
							DWIN_ICON_Show(ICON,ICON_Zoffest, 26, 258);
							DWIN_ICON_Show(ICON,ICON_Language, 26, 311);

							DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 104-(index_tune-5)*53, feedrate_percentage);
							DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 157-(index_tune-5)*53, thermalManager.temp_hotend[0].target);
							DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 210-(index_tune-5)*53, thermalManager.temp_bed.target);
							DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 263-(index_tune-5)*53, thermalManager.fan_speed[0]);
							#if HAS_LEVELING
								show_plus_or_minus(font8x16, Background_black, 2, 2, 202, 316-(index_tune-5)*53, probe_offset.z*100);
							#else
								show_plus_or_minus(font8x16, Background_black, 2, 2, 202, 316-(index_tune-5)*53, zprobe_zoffset*100);
							#endif
							
							break;
						default:
							break;
					}
				}				
				DWIN_UpdateLCD();
			}
		}	
	}

  /* PLA Preheat */
  void HMI_PLAPreheatSetting(void)
	{
		millis_t ms = millis();
		if(ms > Encoder_ms)
		{
			ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
			if(encoder_diffState != ENCODER_DIFF_NO)
			{
				Encoder_ms = ms + Encoder_wait;
				// Avoid flicker by updating only the previous menu
				if(encoder_diffState == ENCODER_DIFF_CW)
				{
					if(select_PLA < 4) select_PLA++;
					else select_PLA = 4;

					if(last_select_PLA != select_PLA)
					{
						DWIN_Draw_Rectangle(1, Background_black,	0,	31+(select_PLA-1)*53,	14,	31+select_PLA*53);
						DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31+select_PLA*53);
						last_select_PLA = select_PLA;
					}
				}
				else if(encoder_diffState == ENCODER_DIFF_CCW)
				{
					if(select_PLA != 0) select_PLA--;
					else select_PLA = 0;

					if(last_select_PLA != select_PLA)
					{
						DWIN_Draw_Rectangle(1, Background_black,	0,	31+(select_PLA+1)*53,	14,	31+(select_PLA+2)*53);
						DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31+select_PLA*53);
						last_select_PLA = select_PLA;
					}
				}
				else if(encoder_diffState == ENCODER_DIFF_ENTER)
				{
					switch(select_PLA)
					{
						case 0:	// back

							checkkey = Temperation;
							select_temp = 4;
							HMI_ValueStruct.show_mode = -1;

							DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
							DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);

							if(HMI_flag.language_flag) 
							{
								DWIN_Frame_AreaCopy(1, 236, 2, 271-8, 479-466, 14, 8);
								DWIN_Frame_AreaCopy(1, 129, 72, 271-115, 479-395, 60, 49);

								DWIN_Frame_AreaCopy(1, 1, 134, 271-215, 479-333, 60, 102);
								DWIN_Frame_AreaCopy(1, 58, 134, 271-158, 479-333, 60, 155);
								DWIN_Frame_AreaCopy(1, 115, 134, 271-99, 479-333, 60, 208);
								DWIN_Frame_AreaCopy(1, 100, 89, 271-93, 479-378, 60, 261);
								DWIN_Frame_AreaCopy(1, 180, 89, 271-11, 479-379, 60, 314);
							}
							else 
							{
								DWIN_Frame_AreaCopy(1, 56, 16, 139, 479-450-1, 14, 8);
								DWIN_Frame_AreaCopy(1, 226, 179, 271-15, 479-290, 60, 49);

								DWIN_Frame_AreaCopy(1, 197, 104, 271-31, 479-365, 60, 102);
								DWIN_Frame_AreaCopy(1, 1, 89, 271-188, 479-377-1, 60+41+3, 102);
								DWIN_Frame_AreaCopy(1, 240, 104, 271-7, 479-365, 60, 155);
								DWIN_Frame_AreaCopy(1, 1, 89, 271-188, 479-377-1, 60+24+3, 155);
								DWIN_Frame_AreaCopy(1, 0, 119, 271-207, 479-347, 60, 208);
								DWIN_Frame_AreaCopy(1, 107, 76, 271-115, 479-393, 60, 261);
								DWIN_Frame_AreaCopy(1, 157, 76, 181, 479-393, 60+49+3, 261);
								DWIN_Frame_AreaCopy(1, 131, 119, 271-89, 479-347, 60+49+24+6, 261);	// PLA setting
								DWIN_Frame_AreaCopy(1, 107, 76, 271-115, 479-393, 60, 314);
								DWIN_Frame_AreaCopy(1, 172, 76, 198, 479-393, 60+49+3, 314);
								DWIN_Frame_AreaCopy(1, 131, 119, 271-89, 479-347, 60+49+24+6, 314);	// ABS setting									
							}

							DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 243);
							DWIN_ICON_Show(ICON,ICON_Back, 26, 46);
							DWIN_Draw_Line(Line_Color, 16, 82, 256, 83);
							for(short int i = 0; i < 5; i++) 
							{
								DWIN_ICON_Show(ICON,ICON_SetEndTemp+i, 26, 99+i*53);
								DWIN_Draw_Line(Line_Color, 16, 135+i*53, 256, 136+i*53);
							}

							DWIN_ICON_Show(ICON,ICON_More, 226, 258);
							DWIN_ICON_Show(ICON,ICON_More, 226, 311);

							DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 104, thermalManager.temp_hotend[0].target);
							DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 157, thermalManager.temp_bed.target);
							DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 210, thermalManager.fan_speed[0]);

							break;
						case 1: // set nozzle temperation
							checkkey = ETemp;
							HMI_ValueStruct.E_Temp = HMI_ValueStruct.preheat_hotend_temp[0];
							DWIN_Draw_IntVariate(true,true,0,font8x16,White,Select_Color, 3, 216, 104, HMI_ValueStruct.preheat_hotend_temp[0]);
							EncoderRate.encoderRateEnabled = 1;
							break;
						case 2: // set bed temperation
							checkkey = BedTemp;
							HMI_ValueStruct.Bed_Temp = HMI_ValueStruct.preheat_bed_temp[0];
							DWIN_Draw_IntVariate(true,true,0,font8x16,White,Select_Color, 3, 216, 157, HMI_ValueStruct.preheat_bed_temp[0]);
							EncoderRate.encoderRateEnabled = 1;
							break;
						case 3: // set fan speed
							checkkey = FanSpeed;
							HMI_ValueStruct.Fan_speed = HMI_ValueStruct.preheat_fan_speed[0];
							DWIN_Draw_IntVariate(true,true,0,font8x16,White,Select_Color, 3, 216, 210, HMI_ValueStruct.preheat_fan_speed[0]);
							EncoderRate.encoderRateEnabled = 1;
							break;
						case 4: // save PLA configuration
							if (settings.save()) {
								buzzer.tone(100, 659);
								buzzer.tone(100, 698);
							}
							else buzzer.tone(20, 440);
							break;
						default:
							break;
					}
				}
				DWIN_UpdateLCD();
			}
		}	
	}

  /* ABS Preheat */
  void HMI_ABSPreheatSetting(void)
	{
		millis_t ms = millis();
		if(ms > Encoder_ms)
		{
			ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
			if(encoder_diffState != ENCODER_DIFF_NO)
			{
				Encoder_ms = ms + Encoder_wait;
				// Avoid flicker by updating only the previous menu
				if(encoder_diffState == ENCODER_DIFF_CW)
				{
					if(select_ABS < 4) select_ABS++;
					else select_ABS = 4;

					if(last_select_ABS != select_ABS)
					{
						DWIN_Draw_Rectangle(1, Background_black,	0,	31+(select_ABS-1)*53,	14,	31+select_ABS*53);
						DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31+select_ABS*53);
						last_select_ABS = select_ABS;
					}
				}
				else if(encoder_diffState == ENCODER_DIFF_CCW)
				{
					if(select_ABS != 0) select_ABS--;
					else select_ABS = 0;

					if(last_select_ABS != select_ABS)
					{
						DWIN_Draw_Rectangle(1, Background_black,	0,	31+(select_ABS+1)*53,	14,	31+(select_ABS+2)*53);
						DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31+select_ABS*53);
						last_select_ABS = select_ABS;
					}
				}
				else if(encoder_diffState == ENCODER_DIFF_ENTER)
				{
					switch(select_ABS)
					{
						case 0:	// back

							checkkey = Temperation;
							select_temp = 5;
							HMI_ValueStruct.show_mode = -1;

							DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
							DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);

							if(HMI_flag.language_flag) 
							{
								DWIN_Frame_AreaCopy(1, 236, 2, 271-8, 479-466, 14, 8);
								DWIN_Frame_AreaCopy(1, 129, 72, 271-115, 479-395, 60, 49);

								DWIN_Frame_AreaCopy(1, 1, 134, 271-215, 479-333, 60, 102);
								DWIN_Frame_AreaCopy(1, 58, 134, 271-158, 479-333, 60, 155);
								DWIN_Frame_AreaCopy(1, 115, 134, 271-99, 479-333, 60, 208);
								DWIN_Frame_AreaCopy(1, 100, 89, 271-93, 479-378, 60, 261);
								DWIN_Frame_AreaCopy(1, 180, 89, 271-11, 479-379, 60, 314);
							}
							else 
							{
								DWIN_Frame_AreaCopy(1, 56, 16, 139, 479-450-1, 14, 8);
								DWIN_Frame_AreaCopy(1, 226, 179, 271-15, 479-290, 60, 49);

								DWIN_Frame_AreaCopy(1, 197, 104, 271-31, 479-365, 60, 102);
								DWIN_Frame_AreaCopy(1, 1, 89, 271-188, 479-377-1, 60+41+3, 102);
								DWIN_Frame_AreaCopy(1, 240, 104, 271-7, 479-365, 60, 155);
								DWIN_Frame_AreaCopy(1, 1, 89, 271-188, 479-377-1, 60+24+3, 155);
								DWIN_Frame_AreaCopy(1, 0, 119, 271-207, 479-347, 60, 208);
								DWIN_Frame_AreaCopy(1, 107, 76, 271-115, 479-393, 60, 261);
								DWIN_Frame_AreaCopy(1, 157, 76, 181, 479-393, 60+49+3, 261);
								DWIN_Frame_AreaCopy(1, 131, 119, 271-89, 479-347, 60+49+24+6, 261);	// PLA setting
								DWIN_Frame_AreaCopy(1, 107, 76, 271-115, 479-393, 60, 314);
								DWIN_Frame_AreaCopy(1, 172, 76, 198, 479-393, 60+49+3, 314);
								DWIN_Frame_AreaCopy(1, 131, 119, 271-89, 479-347, 60+49+24+6, 314);	// ABS setting									
							}

							DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 296);
							DWIN_ICON_Show(ICON,ICON_Back, 26, 46);
							DWIN_Draw_Line(Line_Color, 16, 82, 256, 83);
							for(short int i = 0; i < 5; i++) 
							{
								DWIN_ICON_Show(ICON,ICON_SetEndTemp+i, 26, 99+i*53);
								DWIN_Draw_Line(Line_Color, 16, 135+i*53, 256, 136+i*53);
							}

							DWIN_ICON_Show(ICON,ICON_More, 226, 258);
							DWIN_ICON_Show(ICON,ICON_More, 226, 311);

							DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 104, thermalManager.temp_hotend[0].target);
							DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 157, thermalManager.temp_bed.target);
							DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 210, thermalManager.fan_speed[0]);

							break;
						case 1: // set nozzle temperation
							checkkey = ETemp;
							HMI_ValueStruct.E_Temp = HMI_ValueStruct.preheat_hotend_temp[1];
							DWIN_Draw_IntVariate(true,true,0,font8x16,White,Select_Color, 3, 216, 104, HMI_ValueStruct.preheat_hotend_temp[1]);
							EncoderRate.encoderRateEnabled = 1;
							break;
						case 2: // set bed temperation
							checkkey = BedTemp;
							HMI_ValueStruct.Bed_Temp = HMI_ValueStruct.preheat_bed_temp[1];
							DWIN_Draw_IntVariate(true,true,0,font8x16,White,Select_Color, 3, 216, 157, HMI_ValueStruct.preheat_bed_temp[1]);
							EncoderRate.encoderRateEnabled = 1;
							break;
						case 3: // set fan speed
							checkkey = FanSpeed;
							HMI_ValueStruct.Fan_speed = HMI_ValueStruct.preheat_fan_speed[1];
							DWIN_Draw_IntVariate(true,true,0,font8x16,White,Select_Color, 3, 216, 210, HMI_ValueStruct.preheat_fan_speed[1]);
							EncoderRate.encoderRateEnabled = 1;
							break;
						case 4: // save PLA configuration
							if (settings.save()) {
								buzzer.tone(100, 659);
								buzzer.tone(100, 698);
							}
							else buzzer.tone(20, 440);
							break;
						default:
							break;
					}
				}
				DWIN_UpdateLCD();
			}
		}
	}

  /* Max Speed */
  void HMI_MaxSpeed(void)
	{
		millis_t ms = millis();
		if(ms > Encoder_ms)
		{
			ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
			if(encoder_diffState != ENCODER_DIFF_NO)
			{
				Encoder_ms = ms + Encoder_wait;
				// Avoid flicker by updating only the previous menu
				if(encoder_diffState == ENCODER_DIFF_CW)
				{
					if(select_speed < 4) select_speed++;
					else select_speed = 4;

					if(last_select_speed != select_speed)
					{
						DWIN_Draw_Rectangle(1, Background_black,	0,	31+(select_speed-1)*53,	14,	31+select_speed*53);
						DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31+select_speed*53);
						last_select_speed = select_speed;
					}
				}
				else if(encoder_diffState == ENCODER_DIFF_CCW)
				{
					if(select_speed != 0) select_speed--;
					else select_speed = 0;

					if(last_select_speed != select_speed)
					{
						DWIN_Draw_Rectangle(1, Background_black,	0,	31+(select_speed+1)*53,	14,	31+(select_speed+2)*53);
						DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31+select_speed*53);
						last_select_speed = select_speed;
					}
				}
				else if(encoder_diffState == ENCODER_DIFF_ENTER)
				{
					switch(select_speed)
					{
						case 0:	// back

							checkkey = Motion;
							select_motion = 1;

							DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
							DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);

							if(HMI_flag.language_flag) 
							{
								DWIN_Frame_AreaCopy(1, 1, 16, 271-243, 479-451, 14, 8);
								DWIN_Frame_AreaCopy(1, 129, 72, 271-115, 479-395, 60, 49);

								DWIN_Frame_AreaCopy(1, 173, 133, 228, 479-332, 60, 102);				// max speed
								DWIN_Frame_AreaCopy(1, 173, 133, 200, 479-332, 60, 155);
								DWIN_Frame_AreaCopy(1, 28, 149, 271-202, 479-318, 60+27, 155+1);	// max acceleration
								DWIN_Frame_AreaCopy(1, 173, 133, 200, 479-332, 60, 208);
								DWIN_Frame_AreaCopy(1, 1, 180, 271-243, 479-287, 60+27, 208+1);
								DWIN_Frame_AreaCopy(1, 202, 133, 228, 479-332, 60+54, 208);				// max corner
								DWIN_Frame_AreaCopy(1, 153, 148, 271-77, 479-318, 60, 261);			// transmission ratio
							}
							else 
							{
								DWIN_Frame_AreaCopy(1, 144, 16, 271-82, 479-453, 14, 8);
								DWIN_Frame_AreaCopy(1, 226, 179, 271-15, 479-290, 60, 49);
								
								DWIN_Frame_AreaCopy(1, 245, 119, 271-2, 479-350, 60, 102);					
								DWIN_Frame_AreaCopy(1, 184, 119, 224, 479-347, 60+24+3, 102);				// max speed
								DWIN_Frame_AreaCopy(1, 245, 119, 271-2, 479-350, 60, 155);	
								DWIN_Frame_AreaCopy(1, 1, 135, 271-192, 479-334, 60+24+3, 155);		// max acceleration
								DWIN_Frame_AreaCopy(1, 245, 119, 271-2, 479-350, 60, 208);
								DWIN_Frame_AreaCopy(1, 64, 119, 271-165, 479-350, 60+24+3, 208);	// max corner
								DWIN_Frame_AreaCopy(1, 1, 151, 271-154, 479-318, 60, 261);			// transmission ratio								
							}

							DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 84);
							DWIN_ICON_Show(ICON,ICON_Back, 26, 46);
							DWIN_Draw_Line(Line_Color, 16, 82, 256, 83);
							for(short int i = 0; i < 4; i++) 
							{
								DWIN_ICON_Show(ICON,ICON_MaxSpeed+i, 26, 99+i*53);
								DWIN_Draw_Line(Line_Color, 16, 135+i*53, 256, 136+i*53);
							}

							DWIN_ICON_Show(ICON,ICON_More, 226, 99);
							DWIN_ICON_Show(ICON,ICON_More, 226, 152);
							DWIN_ICON_Show(ICON,ICON_More, 226, 205);
							DWIN_ICON_Show(ICON,ICON_More, 226, 258);

							break;
						case 1: // max Speed X
							checkkey = MaxSpeed_value;
							HMI_flag.feedspeed_flag = X_AXIS;
							HMI_ValueStruct.Max_Feedspeed = planner.settings.max_feedrate_mm_s[X_AXIS];
							DWIN_Draw_IntVariate(true,true,0,font8x16,White,Select_Color, 4, 210, 104+(select_speed-1)*53, HMI_ValueStruct.Max_Feedspeed);
							EncoderRate.encoderRateEnabled = 1;
							break;
						case 2: // max Speed Y
							checkkey = MaxSpeed_value;
							HMI_flag.feedspeed_flag = Y_AXIS;
							HMI_ValueStruct.Max_Feedspeed = planner.settings.max_feedrate_mm_s[Y_AXIS];
							DWIN_Draw_IntVariate(true,true,0,font8x16,White,Select_Color, 4, 210, 104+(select_speed-1)*53, HMI_ValueStruct.Max_Feedspeed);
							EncoderRate.encoderRateEnabled = 1;
							break;
						case 3: // max Speed Z
							checkkey = MaxSpeed_value;
							HMI_flag.feedspeed_flag = Z_AXIS;
							HMI_ValueStruct.Max_Feedspeed = planner.settings.max_feedrate_mm_s[Z_AXIS];
							DWIN_Draw_IntVariate(true,true,0,font8x16,White,Select_Color, 4, 210, 104+(select_speed-1)*53, HMI_ValueStruct.Max_Feedspeed);
							EncoderRate.encoderRateEnabled = 1;
							break;
						case 4: // max Speed E
							checkkey = MaxSpeed_value;
							HMI_flag.feedspeed_flag = E_AXIS;
							HMI_ValueStruct.Max_Feedspeed = planner.settings.max_feedrate_mm_s[E_AXIS];
							DWIN_Draw_IntVariate(true,true,0,font8x16,White,Select_Color, 4, 210, 104+(select_speed-1)*53, HMI_ValueStruct.Max_Feedspeed);
							EncoderRate.encoderRateEnabled = 1;
							break;
						default:
							break;
					}
				}
				DWIN_UpdateLCD();
			}
		}
	}

  /* Max Acceleration */
  void HMI_MaxAcceleration(void)
	{
		millis_t ms = millis();
		if(ms > Encoder_ms)
		{
			ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
			if(encoder_diffState != ENCODER_DIFF_NO)
			{
				Encoder_ms = ms + Encoder_wait;
				// Avoid flicker by updating only the previous menu
				if(encoder_diffState == ENCODER_DIFF_CW)
				{
					if(select_acc < 4) select_acc++;
					else select_acc = 4;

					if(last_select_acc != select_acc)
					{
						DWIN_Draw_Rectangle(1, Background_black,	0,	31+(select_acc-1)*53,	14,	31+select_acc*53);
						DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31+select_acc*53);
						last_select_acc = select_acc;
					}
				}
				else if(encoder_diffState == ENCODER_DIFF_CCW)
				{
					if(select_acc != 0) select_acc--;
					else select_acc = 0;

					if(last_select_acc != select_acc)
					{
						DWIN_Draw_Rectangle(1, Background_black,	0,	31+(select_acc+1)*53,	14,	31+(select_acc+2)*53);
						DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31+select_acc*53);
						last_select_acc = select_acc;
					}
				}
				else if(encoder_diffState == ENCODER_DIFF_ENTER)
				{
					switch(select_acc)
					{
						case 0:	// back

							checkkey = Motion;
							select_motion = 2;

							DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
							DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);

							if(HMI_flag.language_flag) 
							{
								DWIN_Frame_AreaCopy(1, 1, 16, 271-243, 479-451, 14, 8);
								DWIN_Frame_AreaCopy(1, 129, 72, 271-115, 479-395, 60, 49);

								DWIN_Frame_AreaCopy(1, 173, 133, 228, 479-332, 60, 102);				// max speed
								DWIN_Frame_AreaCopy(1, 173, 133, 200, 479-332, 60, 155);
								DWIN_Frame_AreaCopy(1, 28, 149, 271-202, 479-318, 60+27, 155+1);	// max acceleration
								DWIN_Frame_AreaCopy(1, 173, 133, 200, 479-332, 60, 208);
								DWIN_Frame_AreaCopy(1, 1, 180, 271-243, 479-287, 60+27, 208+1);
								DWIN_Frame_AreaCopy(1, 202, 133, 228, 479-332, 60+54, 208);				// max corner
								DWIN_Frame_AreaCopy(1, 153, 148, 271-77, 479-318, 60, 261);			// transmission ratio
							}
							else 
							{
								DWIN_Frame_AreaCopy(1, 144, 16, 271-82, 479-453, 14, 8);
								DWIN_Frame_AreaCopy(1, 226, 179, 271-15, 479-290, 60, 49);
								
								DWIN_Frame_AreaCopy(1, 245, 119, 271-2, 479-350, 60, 102);					
								DWIN_Frame_AreaCopy(1, 184, 119, 224, 479-347, 60+24+3, 102);				// max speed
								DWIN_Frame_AreaCopy(1, 245, 119, 271-2, 479-350, 60, 155);	
								DWIN_Frame_AreaCopy(1, 1, 135, 271-192, 479-334, 60+24+3, 155);		// max acceleration
								DWIN_Frame_AreaCopy(1, 245, 119, 271-2, 479-350, 60, 208);
								DWIN_Frame_AreaCopy(1, 64, 119, 271-165, 479-350, 60+24+3, 208);	// max corner
								DWIN_Frame_AreaCopy(1, 1, 151, 271-154, 479-318, 60, 261);			// transmission ratio																
							}

							DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 137);
							DWIN_ICON_Show(ICON,ICON_Back, 26, 46);
							DWIN_Draw_Line(Line_Color, 16, 82, 256, 83);
							for(short int i = 0; i < 4; i++) 
							{
								DWIN_ICON_Show(ICON,ICON_MaxSpeed+i, 26, 99+i*53);
								DWIN_Draw_Line(Line_Color, 16, 135+i*53, 256, 136+i*53);
							}

							DWIN_ICON_Show(ICON,ICON_More, 226, 99);
							DWIN_ICON_Show(ICON,ICON_More, 226, 152);
							DWIN_ICON_Show(ICON,ICON_More, 226, 205);
							DWIN_ICON_Show(ICON,ICON_More, 226, 258);

							break;
						case 1: // max acceleration X
							checkkey = MaxAcceleration_value;
							HMI_flag.acc_flag = X_AXIS;
							HMI_ValueStruct.Max_Acceleration = planner.settings.max_acceleration_mm_per_s2[X_AXIS];
							DWIN_Draw_IntVariate(true,true,0,font8x16,White,Select_Color, 4, 210, 104+(select_acc-1)*53, HMI_ValueStruct.Max_Acceleration);
							EncoderRate.encoderRateEnabled = 1;
							break;
						case 2: // max acceleration Y
							checkkey = MaxAcceleration_value;
							HMI_flag.acc_flag = Y_AXIS;
							HMI_ValueStruct.Max_Acceleration = planner.settings.max_acceleration_mm_per_s2[Y_AXIS];
							DWIN_Draw_IntVariate(true,true,0,font8x16,White,Select_Color, 4, 210, 104+(select_acc-1)*53, HMI_ValueStruct.Max_Acceleration);
							EncoderRate.encoderRateEnabled = 1;
							break;
						case 3: // max acceleration Z
							checkkey = MaxAcceleration_value;
							HMI_flag.acc_flag = Z_AXIS;
							HMI_ValueStruct.Max_Acceleration = planner.settings.max_acceleration_mm_per_s2[Z_AXIS];
							DWIN_Draw_IntVariate(true,true,0,font8x16,White,Select_Color, 4, 210, 104+(select_acc-1)*53, HMI_ValueStruct.Max_Acceleration);
							EncoderRate.encoderRateEnabled = 1;
							break;
						case 4: // max acceleration E
							checkkey = MaxAcceleration_value;
							HMI_flag.acc_flag = E_AXIS;
							HMI_ValueStruct.Max_Acceleration = planner.settings.max_acceleration_mm_per_s2[E_AXIS];
							DWIN_Draw_IntVariate(true,true,0,font8x16,White,Select_Color, 4, 210, 104+(select_acc-1)*53, HMI_ValueStruct.Max_Acceleration);
							EncoderRate.encoderRateEnabled = 1;
							break;
						default:
							break;
					}
				}
				DWIN_UpdateLCD();
			}
		}
	}

  /* Max Corner */
  void HMI_MaxCorner(void)
	{
		millis_t ms = millis();
		if(ms > Encoder_ms)
		{
			ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
			if(encoder_diffState != ENCODER_DIFF_NO)
			{
				Encoder_ms = ms + Encoder_wait;
				// Avoid flicker by updating only the previous menu
				if(encoder_diffState == ENCODER_DIFF_CW)
				{
					if(select_corner < 4) select_corner++;
					else select_corner = 4;

					if(last_select_corner != select_corner)
					{
						DWIN_Draw_Rectangle(1, Background_black,	0,	31+(select_corner-1)*53,	14,	31+select_corner*53);
						DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31+select_corner*53);
						last_select_corner = select_corner;
					}
				}
				else if(encoder_diffState == ENCODER_DIFF_CCW)
				{
					if(select_corner != 0) select_corner--;
					else select_corner = 0;

					if(last_select_corner != select_corner)
					{
						DWIN_Draw_Rectangle(1, Background_black,	0,	31+(select_corner+1)*53,	14,	31+(select_corner+2)*53);
						DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31+select_corner*53);
						last_select_corner = select_corner;
					}
				}
				else if(encoder_diffState == ENCODER_DIFF_ENTER)
				{
					switch(select_corner)
					{
						case 0:	// back

							checkkey = Motion;
							select_motion = 3;

							DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
							DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);

							if(HMI_flag.language_flag) 
							{
								DWIN_Frame_AreaCopy(1, 1, 16, 271-243, 479-451, 14, 8);
								DWIN_Frame_AreaCopy(1, 129, 72, 271-115, 479-395, 60, 49);

								DWIN_Frame_AreaCopy(1, 173, 133, 228, 479-332, 60, 102);				// max speed
								DWIN_Frame_AreaCopy(1, 173, 133, 200, 479-332, 60, 155);
								DWIN_Frame_AreaCopy(1, 28, 149, 271-202, 479-318, 60+27, 155+1);	// max acceleration
								DWIN_Frame_AreaCopy(1, 173, 133, 200, 479-332, 60, 208);
								DWIN_Frame_AreaCopy(1, 1, 180, 271-243, 479-287, 60+27, 208+1);
								DWIN_Frame_AreaCopy(1, 202, 133, 228, 479-332, 60+54, 208);				// max corner
								DWIN_Frame_AreaCopy(1, 153, 148, 271-77, 479-318, 60, 261);			// transmission ratio
							}
							else 
							{
								DWIN_Frame_AreaCopy(1, 144, 16, 271-82, 479-453, 14, 8);
								DWIN_Frame_AreaCopy(1, 226, 179, 271-15, 479-290, 60, 49);
								
								DWIN_Frame_AreaCopy(1, 245, 119, 271-2, 479-350, 60, 102);					
								DWIN_Frame_AreaCopy(1, 184, 119, 224, 479-347, 60+24+3, 102);				// max speed
								DWIN_Frame_AreaCopy(1, 245, 119, 271-2, 479-350, 60, 155);	
								DWIN_Frame_AreaCopy(1, 1, 135, 271-192, 479-334, 60+24+3, 155);		// max acceleration
								DWIN_Frame_AreaCopy(1, 245, 119, 271-2, 479-350, 60, 208);
								DWIN_Frame_AreaCopy(1, 64, 119, 271-165, 479-350, 60+24+3, 208);	// max corner
								DWIN_Frame_AreaCopy(1, 1, 151, 271-154, 479-318, 60, 261);			// transmission ratio																
							}

							DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 190);
							DWIN_ICON_Show(ICON,ICON_Back, 26, 46);
							DWIN_Draw_Line(Line_Color, 16, 82, 256, 83);
							for(short int i = 0; i < 4; i++) 
							{
								DWIN_ICON_Show(ICON,ICON_MaxSpeed+i, 26, 99+i*53);
								DWIN_Draw_Line(Line_Color, 16, 135+i*53, 256, 136+i*53);
							}

							DWIN_ICON_Show(ICON,ICON_More, 226, 99);
							DWIN_ICON_Show(ICON,ICON_More, 226, 152);
							DWIN_ICON_Show(ICON,ICON_More, 226, 205);
							DWIN_ICON_Show(ICON,ICON_More, 226, 258);

							break;
						case 1: // max corner X
							checkkey = MaxCorner_value;
							HMI_flag.corner_flag = X_AXIS;
							HMI_ValueStruct.Max_Corner = planner.max_jerk[X_AXIS]*MinUnitMult;
							DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Select_Color, 3, 1, 210, 104+(select_corner-1)*53, HMI_ValueStruct.Max_Corner);
							EncoderRate.encoderRateEnabled = 1;
							break;
						case 2: // max corner Y
							checkkey = MaxCorner_value;
							HMI_flag.corner_flag = Y_AXIS;
							HMI_ValueStruct.Max_Corner = planner.max_jerk[Y_AXIS]*MinUnitMult;
							DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Select_Color, 3, 1, 210, 104+(select_corner-1)*53, HMI_ValueStruct.Max_Corner);
							EncoderRate.encoderRateEnabled = 1;
							break;
						case 3: // max corner Z
							checkkey = MaxCorner_value;
							HMI_flag.corner_flag = Z_AXIS;
							HMI_ValueStruct.Max_Corner = planner.max_jerk[Z_AXIS]*MinUnitMult;
							DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Select_Color, 3, 1, 210, 104+(select_corner-1)*53, HMI_ValueStruct.Max_Corner);
							EncoderRate.encoderRateEnabled = 1;
							break;
						case 4: // max corner E
							checkkey = MaxCorner_value;
							HMI_flag.corner_flag = E_AXIS;
							HMI_ValueStruct.Max_Corner = planner.max_jerk[E_AXIS]*MinUnitMult;
							DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Select_Color, 3, 1, 210, 104+(select_corner-1)*53, HMI_ValueStruct.Max_Corner);
							EncoderRate.encoderRateEnabled = 1;
							break;
						default:
							break;
					}
				}
				DWIN_UpdateLCD();
			}
		}
	}

  /* Step */
  void HMI_Step(void)
	{
		millis_t ms = millis();
		if(ms > Encoder_ms)
		{
			ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
			if(encoder_diffState != ENCODER_DIFF_NO)
			{
				Encoder_ms = ms + Encoder_wait;
				// Avoid flicker by updating only the previous menu
				if(encoder_diffState == ENCODER_DIFF_CW)
				{
					if(select_step < 4) select_step++;
					else select_step = 4;
					if(last_select_step != select_step)
					{
						DWIN_Draw_Rectangle(1, Background_black,	0,	31+(select_step-1)*53,	14,	31+select_step*53);
						DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31+select_step*53);
						last_select_step = select_step;
					}
				}
				else if(encoder_diffState == ENCODER_DIFF_CCW)
				{
					if(select_step != 0) select_step--;
					else select_step = 0;
					if(last_select_step != select_step)
					{
						DWIN_Draw_Rectangle(1, Background_black,	0,	31+(select_step+1)*53,	14,	31+(select_step+2)*53);
						DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31+select_step*53);
						last_select_step = select_step;
					}
				}
				else if(encoder_diffState == ENCODER_DIFF_ENTER)
				{
					switch(select_step)
					{
						case 0:	// back

							checkkey = Motion;
							select_motion = 4;

							DWIN_Draw_Rectangle(1, Background_blue,		0,	0,	272,	30);
							DWIN_Draw_Rectangle(1, Background_black,	0,	31,	272,	360);

							if(HMI_flag.language_flag) 
							{
								DWIN_Frame_AreaCopy(1, 1, 16, 271-243, 479-451, 14, 8);
								DWIN_Frame_AreaCopy(1, 129, 72, 271-115, 479-395, 60, 49);

								DWIN_Frame_AreaCopy(1, 173, 133, 228, 479-332, 60, 102);				// max speed
								DWIN_Frame_AreaCopy(1, 173, 133, 200, 479-332, 60, 155);
								DWIN_Frame_AreaCopy(1, 28, 149, 271-202, 479-318, 60+27, 155+1);	// max acceleration
								DWIN_Frame_AreaCopy(1, 173, 133, 200, 479-332, 60, 208);
								DWIN_Frame_AreaCopy(1, 1, 180, 271-243, 479-287, 60+27, 208+1);
								DWIN_Frame_AreaCopy(1, 202, 133, 228, 479-332, 60+54, 208);				// max corner
								DWIN_Frame_AreaCopy(1, 153, 148, 271-77, 479-318, 60, 261);			// transmission ratio
							}
							else 
							{
								DWIN_Frame_AreaCopy(1, 144, 16, 271-82, 479-453, 14, 8);
								DWIN_Frame_AreaCopy(1, 226, 179, 271-15, 479-290, 60, 49);
								
								DWIN_Frame_AreaCopy(1, 245, 119, 271-2, 479-350, 60, 102);					
								DWIN_Frame_AreaCopy(1, 184, 119, 224, 479-347, 60+24+3, 102);				// max speed
								DWIN_Frame_AreaCopy(1, 245, 119, 271-2, 479-350, 60, 155);	
								DWIN_Frame_AreaCopy(1, 1, 135, 271-192, 479-334, 60+24+3, 155);		// max acceleration
								DWIN_Frame_AreaCopy(1, 245, 119, 271-2, 479-350, 60, 208);
								DWIN_Frame_AreaCopy(1, 64, 119, 271-165, 479-350, 60+24+3, 208);	// max corner
								DWIN_Frame_AreaCopy(1, 1, 151, 271-154, 479-318, 60, 261);			// transmission ratio								
							}

							DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 243);
							DWIN_ICON_Show(ICON,ICON_Back, 26, 46);
							DWIN_Draw_Line(Line_Color, 16, 82, 256, 83);
							for(short int i = 0; i < 4; i++) 
							{
								DWIN_ICON_Show(ICON,ICON_MaxSpeed+i, 26, 99+i*53);
								DWIN_Draw_Line(Line_Color, 16, 135+i*53, 256, 136+i*53);
							}

							DWIN_ICON_Show(ICON,ICON_More, 226, 99);
							DWIN_ICON_Show(ICON,ICON_More, 226, 152);
							DWIN_ICON_Show(ICON,ICON_More, 226, 205);
							DWIN_ICON_Show(ICON,ICON_More, 226, 258);

							break;
						case 1: // max step X
							checkkey = Step_value;
							HMI_flag.step_flag = X_AXIS;
							HMI_ValueStruct.Max_Steip = planner.settings.axis_steps_per_mm[X_AXIS]*MinUnitMult;
							DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Select_Color, 3, 1, 210, 104+(select_step-1)*53, HMI_ValueStruct.Max_Steip);
							EncoderRate.encoderRateEnabled = 1;
							break;
						case 2: // max step Y
							checkkey = Step_value;
							HMI_flag.step_flag = Y_AXIS;
							HMI_ValueStruct.Max_Steip = planner.settings.axis_steps_per_mm[Y_AXIS]*MinUnitMult;
							DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Select_Color, 3, 1, 210, 104+(select_step-1)*53, HMI_ValueStruct.Max_Steip);
							EncoderRate.encoderRateEnabled = 1;
							break;
						case 3: // max step Z
							checkkey = Step_value;
							HMI_flag.step_flag = Z_AXIS;
							HMI_ValueStruct.Max_Steip = planner.settings.axis_steps_per_mm[Z_AXIS]*MinUnitMult;
							DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Select_Color, 3, 1, 210, 104+(select_step-1)*53, HMI_ValueStruct.Max_Steip);
							EncoderRate.encoderRateEnabled = 1;
							break;
						case 4: // max step E
							checkkey = Step_value;
							HMI_flag.step_flag = E_AXIS;
							HMI_ValueStruct.Max_Steip = planner.settings.axis_steps_per_mm[E_AXIS]*MinUnitMult;
							DWIN_Draw_FloatVariate(true,true,0,font8x16,White,Select_Color, 3, 1, 210, 104+(select_step-1)*53, HMI_ValueStruct.Max_Steip);
							EncoderRate.encoderRateEnabled = 1;
							break;
						default:
							break;
					}
				}
				DWIN_UpdateLCD();
			}
		}
	}

  void HMI_Init(void)
	{
		HMI_SDCardInit();

		for(uint16 t = 0; t <= 100; t++)
		{
			DWIN_ICON_Show(ICON,ICON_Bar, 15, 260);
			DWIN_Draw_Rectangle(1, Background_black, 15+t*242/100, 260,	257, 280);
			DWIN_UpdateLCD();
			delay(20);
		}

		lcd_select_language();

		#ifdef FIX_MOUNTED_PROBE
			SET_OUTPUT(COM_PIN);
			WRITE(COM_PIN, 1);
		#endif

		delay(200);		// wait stabillze dalay
	}

  void DWIN_Update(void)
	{
		/* status update */
		EachMomentUpdate();

		#if ENABLED(CHECKFILAMENT)
			/* check filament update */
			Check_Filament_Update();
		#endif

		/* sdcard update */
		HMI_SDCardUpdate();
		
		/* rotary encoder update */
		DWIN_HandleDate();
	}

	void Check_Filament_Update(void)
	{
		if(READ(CHECKFILAMENT_PIN) == 0 && card.isPrinting())
		{
			checkkey = Popup_Window;
			Popup_Window_Home();
			pause_action_flag = 1;
			#if ENABLED(POWER_LOSS_RECOVERY)
				if (recovery.enabled) recovery.save(true, false);
			#endif
			queue.inject_P(PSTR("M25"));
		}
	}

  void EachMomentUpdate(void)
	{
		millis_t ms = millis();
		if(ms > next_rts_update_ms)
		{
			next_rts_update_ms = ms + DWIN_UPDATE_INTERVAL;
			// variable update
			update_variable();

			if(checkkey == PrintProcess)
			{
				// if print done
				if(HMI_flag.print_finish && !HMI_flag.confirm_flag)	
				{
					HMI_flag.print_finish = 0;
					HMI_flag.confirm_flag = 1;

					#if ENABLED(POWER_LOSS_RECOVERY)
						card.removeJobRecoveryFile();
						card.autostart_index = 0;
					#endif

					// show percent bar and value
					DWIN_ICON_Show(ICON,ICON_Bar, 15, 93);
					DWIN_Draw_IntVariate(true,true,0,font8x16,Percent_Color,Background_black, 3, 117-8, 133, 100);
					// show remain time
					DWIN_Draw_IntVariate(true,true,1,font8x16,White,Background_black, 2, 176, 212, 0);
					DWIN_Draw_IntVariate(true,true,1,font8x16,White,Background_black, 2, 176+24, 212, 0);
					// show print done confirm
					DWIN_Draw_Rectangle(1, Background_black, 0,	250,	271, 360);
					if(HMI_flag.language_flag) DWIN_ICON_Show(ICON,ICON_Confirm_C, 86, 302-19);
					else DWIN_ICON_Show(ICON,ICON_Confirm_E, 86, 302-19);
				}
				else if(HMI_flag.pause_flag != printingIsPaused())
				{
					// print status update
					if(select_print == 1)
					{
						if(printingIsPaused()) {HMI_flag.pause_flag = 1; ICON_Continue(1);}
						else {HMI_flag.pause_flag = 0; ICON_Pause(1);}
					}
					else
					{
						if(printingIsPaused()) {HMI_flag.pause_flag = 1; ICON_Continue(0);}
						else {HMI_flag.pause_flag = 0; ICON_Pause(0);}
					}
				}
			}

			// pause after homing
			if(pause_action_flag && printingIsPaused() && !planner.has_blocks_queued()) 
			{
				pause_action_flag = 0;
				#if PAUSE_HEAT
					tempbed = thermalManager.temp_bed.target;
					temphot = thermalManager.temp_hotend[0].target;
					thermalManager.disable_all_heaters();
				#endif
				queue.enqueue_now_P(PSTR("G1 F1200 X0 Y0"));
				/* check filament resume */
				if(checkkey == Popup_Window)
				{
					checkkey = Filament_window;
					Popup_window_Filament();
				}
			}

			if(card.isPrinting() && checkkey == PrintProcess) // print process
			{
				static unsigned int last_cardpercentValue = 101;
				if(last_cardpercentValue != card.percentDone()) // print percent
				{
					if((unsigned int) card.percentDone() >= 0)
					{
						Percentrecord = card.percentDone();
						DWIN_ICON_Show(ICON,ICON_Bar, 15, 93);
						DWIN_Draw_Rectangle(1, BarFill_Color,	16+Percentrecord*240/100,	93,	256, 113); // show percent bar
						DWIN_Draw_IntVariate(true,true,0,font8x16,Percent_Color,Background_black, 3, 117-8, 133, Percentrecord);	// show percent value
					}	
					last_cardpercentValue = card.percentDone();
				}

				duration_t elapsed = print_job_timer.duration();	// print timer
				/* already print time */
				if(last_Printtime != ((elapsed.value%3600)/60))	// 1 minute update
				{
					DWIN_Draw_IntVariate(true,true,1,font8x16,White,Background_black, 2, 42, 212, elapsed.value/3600);
					DWIN_Draw_IntVariate(true,true,1,font8x16,White,Background_black, 2, 42+24, 212, (elapsed.value%3600)/60);
					last_Printtime = (elapsed.value%3600)/60;
				}
				/* remain print time */
				if(elapsed.minute() > 5 && ms > next_remain_time_update && HMI_flag.heat_flag == 0) // show after 5 min and 20s update
				{
					remain_time = ((elapsed.value - heat_time) * ((float)card.getfilesize() / (float)card.getIndex())) - (elapsed.value - heat_time);
					DWIN_Draw_IntVariate(true,true,1,font8x16,White,Background_black, 2, 176, 212, remain_time/3600);
					DWIN_Draw_IntVariate(true,true,1,font8x16,White,Background_black, 2, 176+24, 212, (remain_time%3600)/60);
					next_remain_time_update = ms + DWIN_REMAIN_TIME_UPDATE_INTERVAL;
				}
			}
			else if(abort_flag && !HMI_flag.home_flag)	// Print Stop
			{
				abort_flag = 0;
				HMI_ValueStruct.print_speed = feedrate_percentage = 100;
				#if HAS_LEVELING
					zprobe_zoffset = probe_offset.z;
				#else
					zprobe_zoffset = 0;
				#endif

				planner.finish_and_disable();

				// if catd is move
				if(!DWIN_lcd_sd_status)
				{
					select_page = 0;
					Goto_ProcessFrame();
				}

				queue.enqueue_now_P(PSTR("G92 E0"));
			}
			#if ENABLED(POWER_LOSS_RECOVERY)
			else if(DWIN_lcd_sd_status && recovery.info.recovery_flag) // resume print before power off
			{
				if(yes_PLR_flag == 0)
				{
					yes_PLR_flag = 1;

					for (uint16_t i = 0; i < fileCnt ; i++) 
					{
						if(!strcmp(Cardbuf.Cardfilename[i], &recovery.info.sd_filename[1])) // Resume print before power failure while have the same file
						{
							Cardbuf.recovery_flag = 1;
							HMI_flag.select_flag = 1;
							Popup_Window_Resume();
							DWIN_Draw_Rectangle(0, Select_Color, 25, 306, 126, 345);
							DWIN_Draw_Rectangle(0, Select_Color, 24, 305, 127, 346);
							sprintf(filebuf,"%s\056gcode",Cardbuf.Cardshowfilename[i]);
							DWIN_Draw_String(false,true,font8x16, Font_window, Background_window, 136-8*strlen(filebuf)/2, 252, filebuf);
							DWIN_UpdateLCD();
							break;
						}
					}

					// if hasn't resumable G-code file
					if(!Cardbuf.recovery_flag) return;

					while(Cardbuf.recovery_flag)
					{
						ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
						if(encoder_diffState != ENCODER_DIFF_NO)
						{
							if(encoder_diffState == ENCODER_DIFF_CW)
							{
								HMI_flag.select_flag = 0;
								DWIN_Draw_Rectangle(0, Background_window, 25, 306, 126, 345);
								DWIN_Draw_Rectangle(0, Background_window, 24, 305, 127, 346);
								DWIN_Draw_Rectangle(0, Select_Color, 145, 306, 246, 345);
								DWIN_Draw_Rectangle(0, Select_Color, 144, 305, 247, 346);
							}
							else if(encoder_diffState == ENCODER_DIFF_CCW)
							{
								HMI_flag.select_flag = 1;
								DWIN_Draw_Rectangle(0, Background_window, 145, 306, 246, 345);
								DWIN_Draw_Rectangle(0, Background_window, 144, 305, 247, 346);
								DWIN_Draw_Rectangle(0, Select_Color, 25, 306, 126, 345);
								DWIN_Draw_Rectangle(0, Select_Color, 24, 305, 127, 346);
							}
							else if(encoder_diffState == ENCODER_DIFF_ENTER)
							{
								Cardbuf.recovery_flag = 0;
								HMI_ValueStruct.print_speed = feedrate_percentage = 100;
								if(HMI_flag.select_flag) 
								{
									break;
								}
								else 
								{
									#ifdef POWER_LOSS_RECOVERY
										card.removeJobRecoveryFile();
									#endif
									card.autostart_index = 0;
									HMI_StartFrame();
									return;
								}
							}
							DWIN_UpdateLCD();
						}
					}
			
					HMI_flag.heat_flag = 1;
					select_print = 0;
					HMI_ValueStruct.show_mode = 0;
					Goto_PrintProcess();

					DWIN_Draw_Rectangle(1, Background_black,	0,	360,	272,	479);

					DWIN_ICON_Show(ICON,ICON_HotendTemp, 13, 381);
					DWIN_ICON_Show(ICON,ICON_Celsius, 100, 381);
					DWIN_ICON_Show(ICON,ICON_BedTemp, 158, 381);
					DWIN_ICON_Show(ICON,ICON_Celsius, 245, 381);
					DWIN_ICON_Show(ICON,ICON_Speed, 13, 428);
					DWIN_ICON_Show(ICON,ICON_Zoffest, 158, 428);

					// show value
					DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 33, 382, thermalManager.temp_hotend[0].celsius);
					DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 71, 382, thermalManager.temp_hotend[0].target);	
					DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 178, 382, thermalManager.temp_bed.celsius);
					DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 216, 382, thermalManager.temp_bed.target);	
					DWIN_Draw_IntVariate(true,true,0,font8x16,White,Background_black, 3, 33, 429, feedrate_percentage);
					#if HAS_LEVELING
						show_plus_or_minus(font8x16, Background_black, 2, 2, 178+8, 429, probe_offset.z*100);
					#else
						show_plus_or_minus(font8x16, Background_black, 2, 2, 178+8, 429, zprobe_zoffset*100);
					#endif
					DWIN_Draw_String(false,false,font8x16,White,Background_black, 62, 383, (char*)"/");
					DWIN_Draw_String(false,false,font8x16,White,Background_black, 207, 383, (char*)"/");
					DWIN_Draw_String(false,false,font8x16,White,Background_black, 58, 429, (char*)"%");

					recovery.resume();

					return;
				}
			}
			else if(!yes_PLR_flag)	// no need to resume PLR
			{
				yes_PLR_flag = 1;
			}
			#endif
			DWIN_UpdateLCD();
		}
	}

  void DWIN_HandleDate(void)
	{
		switch(checkkey)
		{
			case ProcessFrame:
				HMI_ProcessFrame();
				break;

			case PrintFile:
				HMI_Printfile();
				break;

			case Perpare:
				HMI_Perpare();
				break;

			case Control:
				HMI_Control();
				break;

			case Leveing:
				
				break;

			case PrintProcess:
				HMI_Printing();
				break;

			case Print_window:
				HMI_PauseOrStop();
				break;

			case Filament_window:
				HMI_Filament();
				break;

			case AxisMove:
				HMI_AxisMove();
				break;

			case Temperation:
				HMI_Temperation();
				break;

			case Motion:
				HMI_Motion();
				break;

			case Info:
				HMI_Info();
				break;

			case Tune:
				HMI_Tune();
				break;

			case PLAPreheat:
				HMI_PLAPreheatSetting();
				break;

			case ABSPreheat:
				HMI_ABSPreheatSetting();
				break;

			case MaxSpeed:
				HMI_MaxSpeed();
				break;

			case MaxAcceleration:
				HMI_MaxAcceleration();
				break;

			case MaxCorner:
				HMI_MaxCorner();
				break;

			case Step:
				HMI_Step();
				break;

			case Move_X:
				HMI_Move_X();
				break;

			case Move_Y:
				HMI_Move_Y();
				break;
				
			case Move_Z:
				HMI_Move_Z();
				break;

			case Exturder:
				HMI_Extruder();
				break;

			case Homeoffset:
				HMI_Zoffset();
				break;

			case ETemp:
				HMI_ETemp();
				break;

			case BedTemp:
				HMI_BedTemp();
				break;
			
			case FanSpeed:
				HMI_FanSpeed();
				break;

			case PrintSpeed:
				HMI_PrintSpeed();
				break;

			case MaxSpeed_value:
				HMI_MaxFeedspeedXYZE();
				break;

			case MaxAcceleration_value:
				HMI_MaxAccelerationXYZE();
				break;

			case MaxCorner_value:
				HMI_MaxCornerXYZE();
				break;

			case Step_value:
				HMI_StepXYZE();
				break;

			default:
				break;
		}
	}


#endif

