/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>

#include "../mxsfb.h"

static struct fb_videomode ili9225g_lcd_modedb[] = {
	{
	 "ILI9225G-QCIF", 60, 176, 220, 200000,
	 0, 0,
	 0, 0,
	 0, 0,
	 0,
	 FB_VMODE_NONINTERLACED,
	 0,
	},
};

static struct mpu_lcd_config lcd_config = {
	.bus_mode = MPU_BUS_8080,
	.interface_width = 8,
	.panel_bpp = 16,
};
void mpu_ili9225g_get_lcd_videomode(struct fb_videomode **mode, int *size,
		struct mpu_lcd_config **data)
{
	*mode = &ili9225g_lcd_modedb[0];
	*size = ARRAY_SIZE(ili9225g_lcd_modedb);
	*data = &lcd_config;
}

void Init_LCM(struct mxsfb_info* mxsfb);
void address(struct mxsfb_info* mxsfb);
void LCD_Exit_Standby_ILI9225B(struct mxsfb_info* mxsfb) ;


void ILI9225_CMO20_Initial(struct mxsfb_info* mxsfb);

#if 0
unsigned int ili9225g_read(struct mxsfb_info* mxsfb, unsigned int reg)
{
	unsigned int value = 0;
	mxsfb_mpu_access(mxsfb, MPU_CMD, MPU_WRITE, reg);
	value = mxsfb_mpu_access(mxsfb, MPU_DATA, MPU_READ, 0x00);
	return value;
}
#endif

void ili9225g_write(struct mxsfb_info* mxsfb, unsigned int reg, unsigned int value)
{

	mxsfb_mpu_access(mxsfb, MPU_CMD, MPU_WRITE, (reg>>8)&0xFF);
	mxsfb_mpu_access(mxsfb, MPU_CMD, MPU_WRITE, (reg>>0)&0xFF);
	mxsfb_mpu_access(mxsfb, MPU_DATA, MPU_WRITE, (value>>8)&0xFF);
	mxsfb_mpu_access(mxsfb, MPU_DATA, MPU_WRITE, (value>>0)&0xFF);
}

int mpu_ili9225g_lcd_setup(struct mxsfb_info * mxsfb)
{
	if (mxsfb == NULL)
		return -1;
	ILI9225_CMO20_Initial(mxsfb);

	return 0;
}

int mpu_ili9225g_lcd_poweroff(struct mxsfb_info * mxsfb)
{
#if 0
	/* Display off */
	mxsfb_mpu_access(mxsfb, MPU_CMD, MPU_WRITE, 0x28);
	/* Sleep in */
	mxsfb_mpu_access(mxsfb, MPU_CMD, MPU_WRITE, 0x10);
#endif
	return 0;
}


void LCD_CtrlWrite_ILI9225(struct mxsfb_info* mxsfb, unsigned int reg, unsigned int value)
{
	mxsfb_mpu_access(mxsfb, MPU_CMD, MPU_WRITE, (reg >> 8) & 0xFF);
	mxsfb_mpu_access(mxsfb, MPU_CMD, MPU_WRITE, (reg >> 0) & 0xFF);
	mxsfb_mpu_access(mxsfb, MPU_DATA, MPU_WRITE, (value >> 8) & 0xFF);
	mxsfb_mpu_access(mxsfb, MPU_DATA, MPU_WRITE, (value >> 0) & 0xFF);
}

void ILI9225_CMO20_Initial(struct mxsfb_info* mxsfb)
{

	//************* Start Initial Sequence **********//
	LCD_CtrlWrite_ILI9225(mxsfb, 0x0001,0x011C); // set SS and NL bit 	
	LCD_CtrlWrite_ILI9225(mxsfb, 0x0002,0x0100); // set 1 line inversion 	
	LCD_CtrlWrite_ILI9225(mxsfb, 0x0003,0x1030); // set GRAM write direction and BGR=1. 
	LCD_CtrlWrite_ILI9225(mxsfb, 0x0008,0x0808); // set BP and FP 	
	LCD_CtrlWrite_ILI9225(mxsfb, 0x000C,0x0000); // RGB interface setting R0Ch=0x0110 for  
	LCD_CtrlWrite_ILI9225(mxsfb, 0x000F,0x0A01); // Set frame rate 	
	LCD_CtrlWrite_ILI9225(mxsfb, 0x0020,0x0000); // Set GRAM Address 	
	LCD_CtrlWrite_ILI9225(mxsfb, 0x0021,0x0000); // Set GRAM Address 		//*************Power On sequence ***************	
	msleep(50); // Delay 50ms 	
	LCD_CtrlWrite_ILI9225(mxsfb, 0x0010,0x0A00); // Set SAP,DSTB,STB 	
	LCD_CtrlWrite_ILI9225(mxsfb, 0x0011,0x103B); // Set APON,PON,AON,VCI1EN,VC 		Delay(50); // Delay 50ms 	
	LCD_CtrlWrite_ILI9225(mxsfb, 0x0012,0x6121); // Internal reference voltage= Vci; 	
	LCD_CtrlWrite_ILI9225(mxsfb, 0x0013,0x006F); // Set GVDD 	
	LCD_CtrlWrite_ILI9225(mxsfb, 0x0014,0x5152); // 0x4552 Set VCOMH/VCOML voltage	
	LCD_CtrlWrite_ILI9225(mxsfb, 0x0030,0x0000); 	
	LCD_CtrlWrite_ILI9225(mxsfb, 0x0031,0x00DB); 	
	LCD_CtrlWrite_ILI9225(mxsfb, 0x0032,0x0000); 	
	LCD_CtrlWrite_ILI9225(mxsfb, 0x0033,0x0000); 		
	LCD_CtrlWrite_ILI9225(mxsfb, 0x0034,0x00DB); 	
	LCD_CtrlWrite_ILI9225(mxsfb, 0x0035,0x0000); 	

	LCD_CtrlWrite_ILI9225(mxsfb, 0x0050,0x0000); 	
	LCD_CtrlWrite_ILI9225(mxsfb, 0x0051,0x0207); 	
	LCD_CtrlWrite_ILI9225(mxsfb, 0x0052,0x0704); 	
	LCD_CtrlWrite_ILI9225(mxsfb, 0x0053,0x000B); 	
	LCD_CtrlWrite_ILI9225(mxsfb, 0x0054,0x0407); 	
	LCD_CtrlWrite_ILI9225(mxsfb, 0x0055,0x0702); 	
	LCD_CtrlWrite_ILI9225(mxsfb, 0x0056,0x0000); 	
	LCD_CtrlWrite_ILI9225(mxsfb, 0x0057,0x0B00); 	
	LCD_CtrlWrite_ILI9225(mxsfb, 0x0058,0x1102); 	
	LCD_CtrlWrite_ILI9225(mxsfb, 0x0059,0x0211); 	
	msleep(50); // Delay 50ms 	
	LCD_CtrlWrite_ILI9225(mxsfb, 0x0007,0x1017); 



	LCD_CtrlWrite_ILI9225(mxsfb, 0x0036, 176);      // Set GRAM Address     
	LCD_CtrlWrite_ILI9225(mxsfb, 0x0037, 0);      // Set GRAM Address	
	LCD_CtrlWrite_ILI9225(mxsfb, 0x0038, 220);      // Set GRAM Address     
	LCD_CtrlWrite_ILI9225(mxsfb, 0x0039, 0);      // Set GRAM Address	

	LCD_CtrlWrite_ILI9225(mxsfb, 0x0020, 0);      // Set GRAM Address     
	LCD_CtrlWrite_ILI9225(mxsfb, 0x0021, 0);      // Set GRAM Address	

	mxsfb_mpu_access(mxsfb, MPU_CMD, MPU_WRITE, 0x00);
	mxsfb_mpu_access(mxsfb, MPU_CMD, MPU_WRITE, 0x22);
}
