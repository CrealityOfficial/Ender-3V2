/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../gcode.h"
#include "../../module/printcounter.h"
#include "../../lcd/marlinui.h"

#include "../../MarlinCore.h" // for startOrResumeJob
#include "../../lcd/dwin/e3v2/dwin.h"
#include "../../feature/powerloss.h"
/**D:\xuexi\Ender-3S_ONE\Marlin\src\gcode\stats\M75-M78.cpp
 * D:\xuexi\Ender-3S_ONE\Marlin\src\feature\powerloss.h
 * M75: Start print timer
 */
void GcodeSuite::M75() {
  startOrResumeJob();
}

/**
 * M76: Pause print timer
 */
void GcodeSuite::M76() {
  print_job_timer.pause();
}

/**
 * M77: Stop print timer
 */
void GcodeSuite::M77() {
  print_job_timer.stop();
}

#if ENABLED(PRINTCOUNTER)

/**
 * M78: Show print statistics
 */
void GcodeSuite::M78() {
  if (parser.intval('S') == 78) {  // "M78 S78" will reset the statistics
    print_job_timer.initStats();
    ui.reset_status();
    return;
  }

  #if HAS_SERVICE_INTERVALS
    if (parser.seenval('R')) {
      print_job_timer.resetServiceInterval(parser.value_int());
      ui.reset_status();
      return;
    }
  #endif

  print_job_timer.showStats();
}

#endif // PRINTCOUNTER


//  * M79: cloud print statistics
void GcodeSuite::M79()
{
  if (parser.seenval('S'))
  {
    const int16_t cloudvalue = parser.value_celsius();
    switch (cloudvalue)
    {
      case 0:
        // 0:cloud connect  预留
        #if ENABLED(DWIN_CREALITY_LCD)
          if(recovery.info.sd_printing_flag == false)
          {
            //rtscheck.RTS_SndData(1, WIFI_CONNECTED_DISPLAY_ICON_VP);
            // SERIAL_ECHOLN(" \r\n test_M79 S0 \r\n");
          }          
        #endif
        break;

      case 1:
        // 1:cloud print satrt 
        #if ENABLED(DWIN_CREALITY_LCD)
          if(recovery.info.sd_printing_flag == false)
          {
            // Update_Time_Value = 0;
             _remain_time=0; //rock_20210831  解决剩余时间不清零的问题。
             Cloud_Progress_Bar=0;
            print_job_timer.start();
            HMI_flag.cloud_printing_flag=true; //云打印开始标志位  
            process_subcommands_now_P(PSTR("M420 S1")); //Enable automatic compensation function rock_2021.10.29          
            // SERIAL_ECHOLN(" \r\n test_M79 S1 \r\n");
            // rtscheck.RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
            // change_page_font = 10;
          }          
        #endif
        break;

      case 2:
        // 2:cloud print pause
        #if ENABLED(DWIN_CREALITY_LCD)
          if(recovery.info.sd_printing_flag == false)
          {
            // Update_Time_Value = 0;
            print_job_timer.pause();
            if(HMI_flag.cloud_printing_flag && (checkkey = PrintProcess))
            {
              ICON_Continue(); //图标继续打印
            }
            // SERIAL_ECHOLN(" \r\n test_M79 S2 \r\n");
          }
        #endif
        break;

      case 3:
        // 3:cloud print resume
        #if ENABLED(DWIN_CREALITY_LCD)
          if(recovery.info.sd_printing_flag == false)
          {
            // Update_Time_Value = 0;
            print_job_timer.start();
            if(HMI_flag.cloud_printing_flag)
            {
              ICON_Pause();  //暂停界面
            }
          }
          else //云端控制SD打印暂停恢复
          {

          }          
        #endif
        break;

      case 4:
        // 4:cloud print stop
        #if ENABLED(DWIN_CREALITY_LCD)
          if(recovery.info.sd_printing_flag == false)
          {
            // Update_Time_Value = 0;
            print_job_timer.stop();
            HMI_flag.cloud_printing_flag=false; //云打印开始标志位
            Goto_MainMenu(); 
                _remain_time=0; //rock_20210831  解决剩余时间不清零的问题。
              Draw_Print_ProgressRemain();                   //增加停止打印的跳转页面
            //SERIAL_ECHOLN(" \r\n test_M79 S4 \r\n");
          }
          else 
          {
              //  abortFilePrintSoon(); //云端控制SD卡打印停止。 
            card.flag.abort_sd_printing = true; 
          }
        #endif
        break;

      case 5:
        // 5:cloud print finish
        #if ENABLED(DWIN_CREALITY_LCD)
          if(recovery.info.sd_printing_flag == false)
          {
            //Update_Time_Value = 0;
            print_job_timer.stop();

            HMI_flag.cloud_printing_flag=false; //云打印开始标志位
            Goto_MainMenu();                    //增加停止打印的跳转页面
            _remain_time=0; //rock_20210831  解决剩余时间不清零的问题。
            Draw_Print_ProgressRemain();
            //增加停止打印的跳转页面
            // SERIAL_ECHOLN(" M79 S5 \r\n");
          }
          else 
          {
            card.fileHasFinished();         // Handle end of file reached//云端控制SD卡打印完成。
          }
        #endif
        break;

      default:
        break;
    }
  }
  else if(parser.seenval('T'))
  {
    Cloud_Progress_Bar = parser.value_celsius();
    // SERIAL_ECHOLNPAIR("\r\n Cloud_Progress_Bar=: ", Cloud_Progress_Bar);
  }
}

////////////////////////////////////////////////////////////////////////////////