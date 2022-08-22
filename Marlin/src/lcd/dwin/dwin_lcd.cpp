/**
  ******************************************************************************
  * @file     dwin_lcd.c
  * @author   LEO
	* @date     2019/07/18
	* @version  2.0.1
  * @brief    迪文屏控制操作函数
	******************************************************************************
**/

#include "dwin_lcd.h"
#include <string.h>

#ifdef DWIN_LCDDISPLAY

	unsigned char DWIN_SendBuf[50] = {0xAA};

	unsigned char DWIN_BufTail[4] = {0xCC, 0x33, 0xC3, 0x3C};

	unsigned char databuf[26] = {0};
	unsigned char receivedType;

	int recnum = 0;

	/*发送当前BUF中的数据以及包尾数据 len:整包数据长度*/
	void DWIN_Send_BufTail(unsigned char len)
	{
		for(int i = 0;i < (len-4);i++)
		{
			MYSERIAL1.write(DWIN_SendBuf[i]);
			delayMicroseconds(1);
		}
		for(int i = 0;i < 4;i++)
		{
			MYSERIAL1.write(DWIN_BufTail[i]);
			delayMicroseconds(1);
		}
	}

	// void DWIN_Send_BufTail(unsigned char len)
	// {
	//   usart_tx(MYSERIAL1.c_dev(), DWIN_SendBuf, len-4);
	//   MYSERIAL1.flush();
	//   usart_tx(MYSERIAL1.c_dev(), DWIN_BufTail, 4);
	//   MYSERIAL1.flush();
	// }

	/*----------------------------------------------系统变量函数----------------------------------------------*/
	/*握手 1: 握手成功  2: 握手失败*/
	bool DWIN_ShakeHand(void)
	{
		DWIN_SendBuf[1] = 0x00;
		DWIN_Send_BufTail(6);

		while(MYSERIAL1.available() > 0 && (recnum < SizeofDatabuf))
		{
			databuf[recnum] = MYSERIAL1.read();
			// ignore the invalid data
			if(databuf[0] != FHONE)
			{
				// prevent the program from running.
				if(recnum > 0)
				{
					memset(databuf,0,sizeof(databuf));
					recnum = 0;
				}
				continue;
			}
			delay(10);
			recnum++;
		}

		// receive nothing  	
		if(recnum < 1)
		{
			return -1;
		}
		else if(databuf[0] == FHONE && databuf[1] == 0x00 && recnum > 2)
		{
			if(databuf[2] == 0x4F && databuf[3] == 0x4B) return true;
		}
		return false;
	}

	/*设定背光亮度 luminance:亮度(0x00~0xFF)*/
	void DWIN_Backlight_SetLuminance(unsigned char luminance)
	{
		if(luminance < 0x1F) luminance = 0x1F;
		DWIN_SendBuf[1] = 0x30;
		DWIN_SendBuf[2] = luminance;
		DWIN_Send_BufTail(7);
	}

	/*设定画面显示方向 dir:0,0°; 1,90°; 2,180°; 3,270°*/
	void DWIN_Frame_SetDir(unsigned char dir)
	{
		DWIN_SendBuf[1] = 0x34;
		DWIN_SendBuf[2] = 0x5A;
		DWIN_SendBuf[3] = 0xA5;
		DWIN_SendBuf[4] = dir;
		DWIN_Send_BufTail(9);
	}

	/*更新显示*/
	void DWIN_UpdateLCD(void)
	{
		DWIN_SendBuf[1] = 0x3D;
		DWIN_Send_BufTail(6);
	}

	/*----------------------------------------------绘图相关函数----------------------------------------------*/
	/*画面清屏 color:清屏颜色*/
	void DWIN_Frame_Clear(unsigned short int color)
	{
		DWIN_SendBuf[1] = 0x01;
		DWIN_SendBuf[2] = (color >> 8);
		DWIN_SendBuf[3] = color;
		DWIN_Send_BufTail(8);
	}

	/*画面画线 color:线段颜色 xStart:X起始坐标 yStart:Y起始坐标 xEnd:X终止坐标 yEnd:Y终止坐标*/
	void DWIN_Draw_Line(unsigned short int color, unsigned short int xStart, unsigned short int yStart, unsigned short int xEnd, unsigned short int yEnd)
	{
		DWIN_SendBuf[1] = 0x03;
		DWIN_SendBuf[2] = (color >> 8);
		DWIN_SendBuf[3] = color;
		DWIN_SendBuf[4] = (xStart >> 8);
		DWIN_SendBuf[5] = xStart;
		DWIN_SendBuf[6] = (yStart >> 8);
		DWIN_SendBuf[7] = yStart;
		DWIN_SendBuf[8] = (xEnd >> 8);
		DWIN_SendBuf[9] = xEnd;
		DWIN_SendBuf[10] = (yEnd >> 8);
		DWIN_SendBuf[11] = yEnd;
		DWIN_Send_BufTail(16);
	}

	/*画面画矩形 mode:0,外框;1,填充;2,异或填充 color:颜色 xStart/yStart:矩形左上坐标 xEnd/yEnd:矩形右下坐标*/
	void DWIN_Draw_Rectangle(unsigned char mode, unsigned short int color, \
													unsigned short int xStart, unsigned short int yStart, unsigned short int xEnd, unsigned short int yEnd)
	{
		DWIN_SendBuf[1] = 0x05;
		DWIN_SendBuf[2] = mode;
		DWIN_SendBuf[3] = (color >> 8);
		DWIN_SendBuf[4] = color;
		DWIN_SendBuf[5] = (xStart >> 8);
		DWIN_SendBuf[6] = xStart;
		DWIN_SendBuf[7] = (yStart >> 8);
		DWIN_SendBuf[8] = yStart;
		DWIN_SendBuf[9] = (xEnd >> 8);
		DWIN_SendBuf[10] = xEnd;
		DWIN_SendBuf[11] = (yEnd >> 8);
		DWIN_SendBuf[12] = yEnd;
		DWIN_Send_BufTail(17);
	}

	/*画面区域移动 mode:0,环移;1,平移 dir:0,向左移动;1,向右移动;2,向上移动;3,向下移动 dis:移动距离
									color:填充颜色 xStart/yStart:选定区域左上坐标 xEnd/yEnd:选定区域右下坐标*/
	void DWIN_Frame_AreaMove(unsigned char mode, unsigned char dir, unsigned short int dis, \
													unsigned short int color, unsigned short int xStart, unsigned short int yStart, unsigned short int xEnd, unsigned short int yEnd)
	{
		DWIN_SendBuf[1] = 0x09;
		DWIN_SendBuf[2] = (mode<<7) | dir;
		DWIN_SendBuf[3] = (dis >> 8);
		DWIN_SendBuf[4] = dis;
		DWIN_SendBuf[5] = (color >> 8);
		DWIN_SendBuf[6] = color;
		DWIN_SendBuf[7] = (xStart >> 8);
		DWIN_SendBuf[8] = xStart;
		DWIN_SendBuf[9] = (yStart >> 8);
		DWIN_SendBuf[10] = yStart;
		DWIN_SendBuf[11] = (xEnd >> 8);
		DWIN_SendBuf[12] = xEnd;
		DWIN_SendBuf[13] = (yEnd >> 8);
		DWIN_SendBuf[14] = yEnd;
		DWIN_Send_BufTail(19);
	}

	/*----------------------------------------------文本相关函数----------------------------------------------*/
	/*画面显示字符串 widthAdjust:true,自调整字符宽度;false,不调整字符宽度 bShow:true,显示背景色;false,不显示背景色 size:字号大小
										color:字符颜色 bColor:背景颜色 x/y:字符串左上坐标 *string:字符串*/
	void DWIN_Draw_String(bool widthAdjust, bool bShow, unsigned char size, \
												unsigned short int color, unsigned short int bColor, unsigned short int x, unsigned short int y, char *string)
	{
		unsigned char len = strlen(string);	
		DWIN_SendBuf[1] = 0x11;
		DWIN_SendBuf[2] = (widthAdjust? 0x80:0x00) | (bShow? 0x40:0x00) | size;
		DWIN_SendBuf[3] = (color >> 8);
		DWIN_SendBuf[4] = color;
		DWIN_SendBuf[5] = (bColor >> 8);
		DWIN_SendBuf[6] = bColor;
		DWIN_SendBuf[7] = (x >> 8);
		DWIN_SendBuf[8] = x;
		DWIN_SendBuf[9] = (y >> 8);
		DWIN_SendBuf[10] = y;
		memcpy(&DWIN_SendBuf[11], string, len);
		DWIN_Send_BufTail(15 + len);
	}

	/*画面显示正整数 bShow:true,显示背景色;false,不显示背景色 zeroFill:true,补零;false,不补零 zeroMode:1,无效0显示为0; 0,无效0显示为空格 size:字号大小
										color:字符颜色 bColor:背景颜色 iNum:位数 x/y:变量左上坐标 variate:整型变量*/
	void DWIN_Draw_IntVariate(unsigned char bShow, bool zeroFill, unsigned char zeroMode, unsigned char size, unsigned short int color, \
														unsigned short int bColor, unsigned char iNum, unsigned short int x, unsigned short int y, unsigned int variate)
	{
		char count;
		DWIN_SendBuf[1] = 0x14;
		DWIN_SendBuf[2] = (bShow? 0x80:0x00) | (zeroFill? 0x20:0x00) | (zeroMode? 0x10:0x00) | size;
		DWIN_SendBuf[3] = (color >> 8);
		DWIN_SendBuf[4] = color;
		DWIN_SendBuf[5] = (bColor >> 8);
		DWIN_SendBuf[6] = bColor;
		DWIN_SendBuf[7] = iNum;
		DWIN_SendBuf[8] = 0;
		DWIN_SendBuf[9] = (x >> 8);
		DWIN_SendBuf[10] = x;
		DWIN_SendBuf[11] = (y >> 8);
		DWIN_SendBuf[12] = y;
		/*
		for(count=0; count<8; count++)
		{
			DWIN_SendBuf[13 + count] = variate;
			variate >>= 8;
			if((variate&0xFF) == 0x00) break;
		}
		*/
		for(count=7; count>=0; count--)
		{
			DWIN_SendBuf[13 + count] = variate;
			variate >>= 8;
		}
		
		DWIN_Send_BufTail(18 + 7);
	}

	/*画面显示浮点数 bShow:true,显示背景色;false,不显示背景色 zeroFill:true,补零;false,不补零 zeroMode:1,无效0显示为0; 0,无效0显示为空格 size:字号大小
										color:字符颜色 bColor:背景颜色 iNum:整数位数 fNum:小数位数 x/y:变量左上坐标 variate:浮点数变量*/
	void DWIN_Draw_FloatVariate(unsigned char bShow, bool zeroFill, unsigned char zeroMode, unsigned char size, unsigned short int color, \
															unsigned short int bColor, unsigned char iNum, unsigned char fNum, unsigned short int x, unsigned short int y, long variate)
	{
		//unsigned char *fvalue = (unsigned char*)&variate;
		DWIN_SendBuf[1] = 0x14;
		DWIN_SendBuf[2] = (bShow? 0x80:0x00) | (zeroFill? 0x20:0x00) | (zeroMode? 0x10:0x00) | size;
		DWIN_SendBuf[3] = (color >> 8);
		DWIN_SendBuf[4] = color;
		DWIN_SendBuf[5] = (bColor >> 8);
		DWIN_SendBuf[6] = bColor;
		DWIN_SendBuf[7] = iNum;
		DWIN_SendBuf[8] = fNum;
		DWIN_SendBuf[9] = (x >> 8);
		DWIN_SendBuf[10] = x;
		DWIN_SendBuf[11] = (y >> 8);
		DWIN_SendBuf[12] = y;
		DWIN_SendBuf[13] = variate >> 24;
		DWIN_SendBuf[14] = variate >> 16;
		DWIN_SendBuf[15] = variate >> 8;
		DWIN_SendBuf[16] = variate;
		/*
		DWIN_SendBuf[13] = fvalue[3];
		DWIN_SendBuf[14] = fvalue[2];
		DWIN_SendBuf[15] = fvalue[1];
		DWIN_SendBuf[16] = fvalue[0];
		*/
		DWIN_Send_BufTail(21);
	}

	/*----------------------------------------------图片相关函数----------------------------------------------*/
	/*jpg图片显示并缓存在#0虚拟显示区 id:图片ID*/
	void DWIN_JPG_ShowAndCache(unsigned char id)
	{
		DWIN_SendBuf[1] = 0x22;
		DWIN_SendBuf[2] = 0x00;
		DWIN_SendBuf[3] = id;
		DWIN_Send_BufTail(8); 				//AA 23 00 00 00 00 08 00 01 02 03 CC 33 C3 3C
	}

	/*图标显示 libID:图标库ID picID:图标ID x/y:图标左上坐标*/
	void DWIN_ICON_Show(unsigned char libID, unsigned char picID, unsigned short int x, unsigned short int y)
	{
		if(x > 272) x = 272;
		if(y > 480) y = 480; // -- ozy
		DWIN_SendBuf[1] = 0x23;
		DWIN_SendBuf[2] = (x >> 8);
		DWIN_SendBuf[3] = x;
		DWIN_SendBuf[4] = (y >> 8);
		DWIN_SendBuf[5] = y;
		DWIN_SendBuf[6] = 0x80 | libID;
		DWIN_SendBuf[7] = picID;
		DWIN_Send_BufTail(12);
	}

	/*jpg图片解压到#1虚拟显示区 id:图片ID*/
	void DWIN_JPG_CacheTo1(unsigned char id)
	{
		DWIN_SendBuf[1] = 0x25;
		DWIN_SendBuf[2] = 0x01;
		DWIN_SendBuf[3] = id;
		DWIN_Send_BufTail(8);
	}

	/*从虚拟显示区复制区域至当前画面 cacheID:虚拟区号 xStart/yStart:虚拟区左上坐标 xEnd/yEnd:虚拟区右下坐标 x/y:当前画面粘贴坐标*/
	void DWIN_Frame_AreaCopy(unsigned char cacheID, unsigned short int xStart, unsigned short int yStart, \
													unsigned short int xEnd, unsigned short int yEnd, unsigned short int x, unsigned short int y)
	{
		DWIN_SendBuf[1] = 0x27;
		DWIN_SendBuf[2] = 0x80 | cacheID;
		DWIN_SendBuf[3] = (xStart >> 8);
		DWIN_SendBuf[4] = xStart;
		DWIN_SendBuf[5] = (yStart >> 8);
		DWIN_SendBuf[6] = yStart;
		DWIN_SendBuf[7] = (xEnd >> 8);
		DWIN_SendBuf[8] = xEnd;
		DWIN_SendBuf[9] = (yEnd >> 8);
		DWIN_SendBuf[10] = yEnd;
		DWIN_SendBuf[11] = (x >> 8);
		DWIN_SendBuf[12] = x;
		DWIN_SendBuf[13] = (y >> 8);
		DWIN_SendBuf[14] = y;
		DWIN_Send_BufTail(19);
	}

#endif


