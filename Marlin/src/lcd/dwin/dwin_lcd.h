#ifndef __DWIN_LCD_H
#define __DWIN_LCD_H

#include "../../inc/MarlinConfig.h"
#include "../../Marlin.h"

#ifdef DWIN_LCDDISPLAY

	#define RECEIVED_NO_DATA         0x00
	#define RECEIVED_SHAKE_HAND_ACK  0x01

	#define SizeofDatabuf            26

	#define FHONE                    0xAA

	/*接收数据解析 返回值:ture,接收到数据;false,未接收到数据*/
	bool DWIN_ReceiveAnalyze(void);

	/*发送当前BUF中的数据以及包尾数据 len:整包数据长度*/
	void DWIN_Send_BufTail(unsigned char len);

	/*----------------------------------------------系统变量函数----------------------------------------------*/
	/*握手 1: 握手成功  2: 握手失败*/
	bool DWIN_ShakeHand(void);

	/*设定背光亮度 luminance:亮度(0x00~0xFF)*/
	void DWIN_Backlight_SetLuminance(unsigned char luminance);

	/*设定画面显示方向 dir:0,0°; 1,90°; 2,180°; 3,270°*/
	void DWIN_Frame_SetDir(unsigned char dir);

	/*更新显示*/
	void DWIN_UpdateLCD(void);

	/*----------------------------------------------绘图相关函数----------------------------------------------*/
	/*画面清屏 color:清屏颜色*/
	void DWIN_Frame_Clear(unsigned short int color);

	/*画面画线 color:线段颜色 xStart:X起始坐标 yStart:Y起始坐标 xEnd:X终止坐标 yEnd:Y终止坐标*/
	void DWIN_Draw_Line(unsigned short int color, unsigned short int xStart, unsigned short int yStart, unsigned short int xEnd, unsigned short int yEnd);

	/*画面画矩形 mode:0,外框;1,填充;2,异或填充 color:颜色 xStart/yStart:矩形左上坐标 xEnd/yEnd:矩形右下坐标*/
	void DWIN_Draw_Rectangle(unsigned char mode, unsigned short int color, \
													unsigned short int xStart, unsigned short int yStart, unsigned short int xEnd, unsigned short int yEnd);

	/*画面区域移动 mode:0,环移;1,平移 dir:0,向左移动;1,向右移动;2,向上移动;3,向下移动 dis:移动距离
									color:填充颜色 xStart/yStart:选定区域左上坐标 xEnd/yEnd:选定区域右下坐标*/
	void DWIN_Frame_AreaMove(unsigned char mode, unsigned char dir, unsigned short int dis, \
													unsigned short int color, unsigned short int xStart, unsigned short int yStart, unsigned short int xEnd, unsigned short int yEnd);

	/*----------------------------------------------文本相关函数----------------------------------------------*/
	/*画面显示字符串 widthAdjust:true,自调整字符宽度;false,不调整字符宽度 bShow:true,显示背景色;false,不显示背景色 size:字号大小
										color:字符颜色 bColor:背景颜色 x/y:字符串左上坐标 *string:字符串*/
	void DWIN_Draw_String(bool widthAdjust, bool bShow, unsigned char size, \
												unsigned short int color, unsigned short int bColor, unsigned short int x, unsigned short int y, char *string);

	/*画面显示正整数 bShow:true,显示背景色;false,不显示背景色 zeroFill:true,补零;false,不补零 zeroMode:1,无效0显示为0; 0,无效0显示为空格 size:字号大小
										color:字符颜色 bColor:背景颜色 iNum:位数 x/y:变量左上坐标 variate:整型变量*/
	void DWIN_Draw_IntVariate(unsigned char bShow, bool zeroFill, unsigned char zeroMode, unsigned char size, unsigned short int color, \
														unsigned short int bColor, unsigned char iNum, unsigned short int x, unsigned short int y, unsigned int variate);

	/*画面显示浮点数 bShow:true,显示背景色;false,不显示背景色 zeroFill:true,补零;false,不补零 zeroMode:1,无效0显示为0; 0,无效0显示为空格 size:字号大小
										color:字符颜色 bColor:背景颜色 iNum:整数位数 fNum:小数位数 x/y:变量左上坐标 variate:浮点数变量*/
	void DWIN_Draw_FloatVariate(unsigned char bShow, bool zeroFill, unsigned char zeroMode, unsigned char size, unsigned short int color, \
															unsigned short int bColor, unsigned char iNum, unsigned char fNum, unsigned short int x, unsigned short int y, long variate);

	/*----------------------------------------------图片相关函数----------------------------------------------*/
	/*jpg图片显示并缓存在#0虚拟显示区 id:图片ID*/
	void DWIN_JPG_ShowAndCache(unsigned char id);

	/*图标显示 libID:图标库ID picID:图标ID x/y:图标左上坐标*/
	void DWIN_ICON_Show(unsigned char libID, unsigned char picID, unsigned short int x, unsigned short int y);

	/*jpg图片解压到#1虚拟显示区 id:图片ID*/
	void DWIN_JPG_CacheTo1(unsigned char id);

	/*从虚拟显示区复制区域至当前画面 cacheID:虚拟区号 xStart/yStart:虚拟区左上坐标 xEnd/yEnd:虚拟区右下坐标 x/y:当前画面粘贴坐标*/
	void DWIN_Frame_AreaCopy(unsigned char cacheID, unsigned short int xStart, unsigned short int yStart, \
													unsigned short int xEnd, unsigned short int yEnd, unsigned short int x, unsigned short int y);

#endif

#endif

