/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	drivers/video/broadcom/displays/lcd_ili9486.h
*
* Unless you and Broadcom execute a separate DISPCTRL_WRitten software license agreement
* governing use of this software, this software is licensed to you under the
* terms of the GNU General Public License version 2, available at
* http://www.gnu.org/copyleft/gpl.html (the "GPL").
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a license
* other than the GPL, without Broadcom's express prior DISPCTRL_WRitten consent.
*******************************************************************************/

/****************************************************************************
*
*  lcd_ili9486.h
*
*  PURPOSE:
*    This is the LCD-specific code for a ili9486 module.
*
*****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */
#ifndef __BCM_LCD_ILI9486_H
#define __BCM_LCD_ILI9486_H

//  LCD command definitions
#define PIXEL_FORMAT_RGB565  0x55   // for MPU & RGB interface
#define PIXEL_FORMAT_RGB666  0x66   // for MPU & RGB interface
#define PIXEL_FORMAT_RGB888  0x77   // for MPU & RGB interface

#define RESET_SEQ 	{DISPCTRL_WR_CMND, 0x2A,0},\
	{DISPCTRL_WR_DATA, 0, (dev->col_start) >> 8},\
	{DISPCTRL_WR_DATA, 0, dev->col_start & 0xFF},\
	{DISPCTRL_WR_DATA, 0, (dev->col_end) >> 8},\
	{DISPCTRL_WR_DATA, 0, dev->col_end & 0xFF},\
	{DISPCTRL_WR_CMND, 0x2B,0},\
	{DISPCTRL_WR_DATA, 0, (dev->row_start) >> 8},\
	{DISPCTRL_WR_DATA, 0, dev->row_start & 0xFF},\
	{DISPCTRL_WR_DATA, 0, (dev->row_end) >> 8},\
	{DISPCTRL_WR_DATA, 0, dev->row_end & 0xFF},\
	{DISPCTRL_WR_CMND, 0x2C,0}

#define PANEL_BOE	0x5BBCD0

const char *LCD_panel_name = "ILI9486V BOE";

// DISP DRV API - Display Info
static DISPDRV_INFO_T Disp_Info = {
	DISPLAY_TYPE_LCD_STD,	// DISPLAY_TYPE_T          type;          
	320,			// UInt32                  width;         
	480,			// UInt32                  height;        
	DISPDRV_FB_FORMAT_RGB888_U, // DISPDRV_FB_FORMAT_T     input_format; //@HW
	//DISPDRV_FB_FORMAT_RGB565,
	DISPLAY_BUS_SMI,	// DISPLAY_BUS_T           bus_type;
	4,			// Bpp; : !!! init may overwrite
};


DISPCTRL_REC_T power_on_seq_ili9486_BOE[] =
{
       {DISPCTRL_SLEEP_MS, 0, 100},   /* delay after Reset 100 + 100 = 200 */

        /* + Power Setting Sequence */
       {DISPCTRL_WR_CMND, 0xC0,0}, 
	{DISPCTRL_WR_DATA, 0, 0x0B},
	{DISPCTRL_WR_DATA, 0, 0x0B},	

       {DISPCTRL_WR_CMND, 0xC1,0}, 
	{DISPCTRL_WR_DATA, 0, 0x45},

       {DISPCTRL_WR_CMND, 0xC2,0}, 
	 {DISPCTRL_WR_DATA, 0, 0x22},
        /* - Power Setting Sequence */

       {DISPCTRL_SLEEP_MS, 0, 150}, /* wait 150ms */
 
        /* + Display Parameter Setting */
       {DISPCTRL_WR_CMND, 0x2A,0}, 
	{DISPCTRL_WR_DATA, 0, 0x00},
       {DISPCTRL_WR_DATA, 0, 0x00},
       {DISPCTRL_WR_DATA, 0, 0x01},
       {DISPCTRL_WR_DATA, 0, 0x3F},

        {DISPCTRL_WR_CMND, 0x2B,0}, 
	 {DISPCTRL_WR_DATA, 0, 0x00},
       {DISPCTRL_WR_DATA, 0, 0x00},
       {DISPCTRL_WR_DATA, 0, 0x01},
       {DISPCTRL_WR_DATA, 0, 0xDF},

       {DISPCTRL_WR_CMND, 0xB1,0}, 
	{DISPCTRL_WR_DATA, 0, 0xB0},   /* C0 = 81Hz, B0 = 67Hz */
	{DISPCTRL_WR_DATA, 0, 0x12},

       {DISPCTRL_WR_CMND, 0xB4,0}, 
	{DISPCTRL_WR_DATA, 0, 0x02},

       {DISPCTRL_WR_CMND, 0xB5,0}, 
	{DISPCTRL_WR_DATA, 0, 0x08},
	{DISPCTRL_WR_DATA, 0, 0x0C},	
       {DISPCTRL_WR_DATA, 0, 0x10},
	{DISPCTRL_WR_DATA, 0, 0x0A},	
       
       {DISPCTRL_WR_CMND, 0xB6,0}, 
	{DISPCTRL_WR_DATA, 0, 0x02},
	{DISPCTRL_WR_DATA, 0, 0x22},	
       {DISPCTRL_WR_DATA, 0, 0x3B},

       {DISPCTRL_WR_CMND, 0xB7,0}, 
	 {DISPCTRL_WR_DATA, 0, 0xC6},

       {DISPCTRL_WR_CMND, 0xF2,0}, 
	{DISPCTRL_WR_DATA, 0, 0x18},
	{DISPCTRL_WR_DATA, 0, 0xA3},	
       {DISPCTRL_WR_DATA, 0, 0x12},
	{DISPCTRL_WR_DATA, 0, 0x02},
	{DISPCTRL_WR_DATA, 0, 0xB2},	
       {DISPCTRL_WR_DATA, 0, 0x12},
	{DISPCTRL_WR_DATA, 0, 0xFF},
	{DISPCTRL_WR_DATA, 0, 0x10},	
       {DISPCTRL_WR_DATA, 0, 0x00},       

       {DISPCTRL_WR_CMND, 0xF4,0}, 
	{DISPCTRL_WR_DATA, 0, 0x00},
	{DISPCTRL_WR_DATA, 0, 0x00},	
       {DISPCTRL_WR_DATA, 0, 0x08},
 	{DISPCTRL_WR_DATA, 0, 0x91},	
       {DISPCTRL_WR_DATA, 0, 0x04},
       
       {DISPCTRL_WR_CMND, 0x3A,0}, 
	{DISPCTRL_WR_DATA, 0, 0x06},

       {DISPCTRL_WR_CMND, 0x35,0}, 

       {DISPCTRL_WR_CMND, 0x36,0}, 
	{DISPCTRL_WR_DATA, 0, 0xD8},
	
       {DISPCTRL_WR_CMND, 0x44,0}, 
	{DISPCTRL_WR_DATA, 0, 0x01},	
	{DISPCTRL_WR_DATA, 0, 0xD5},	
	
       {DISPCTRL_WR_CMND, 0xF8,0}, 
	{DISPCTRL_WR_DATA, 0, 0x21},
	{DISPCTRL_WR_DATA, 0, 0x06},	
        /* - Display Parameter Setting */

        /* + Gamma Setting */
       {DISPCTRL_WR_CMND, 0xE0,0}, 
	{DISPCTRL_WR_DATA, 0, 0x0F},
	{DISPCTRL_WR_DATA, 0, 0x18},	
	{DISPCTRL_WR_DATA, 0, 0x13},	
	{DISPCTRL_WR_DATA, 0, 0x08},	
       {DISPCTRL_WR_DATA, 0, 0x0B},
	{DISPCTRL_WR_DATA, 0, 0x07},	
       {DISPCTRL_WR_DATA, 0, 0x4A},
	{DISPCTRL_WR_DATA, 0, 0xA7},	
       {DISPCTRL_WR_DATA, 0, 0x3A},
	{DISPCTRL_WR_DATA, 0, 0x0C},	
       {DISPCTRL_WR_DATA, 0, 0x16},
       {DISPCTRL_WR_DATA, 0, 0x07},	
	{DISPCTRL_WR_DATA, 0, 0x09},	
       {DISPCTRL_WR_DATA, 0, 0x06},
	{DISPCTRL_WR_DATA, 0, 0x00},	
       
       {DISPCTRL_WR_CMND, 0xE1,0}, 
	{DISPCTRL_WR_DATA, 0, 0x0F},
	{DISPCTRL_WR_DATA, 0, 0x34},	
       {DISPCTRL_WR_DATA, 0, 0x31},
	{DISPCTRL_WR_DATA, 0, 0x09},	
       {DISPCTRL_WR_DATA, 0, 0x0B},
	{DISPCTRL_WR_DATA, 0, 0x02},	
       {DISPCTRL_WR_DATA, 0, 0x41},
	{DISPCTRL_WR_DATA, 0, 0x53},	
       {DISPCTRL_WR_DATA, 0, 0x30},
	{DISPCTRL_WR_DATA, 0, 0x04},	
       {DISPCTRL_WR_DATA, 0, 0x0F},
	{DISPCTRL_WR_DATA, 0, 0x02},	
       {DISPCTRL_WR_DATA, 0, 0x17},	
       {DISPCTRL_WR_DATA, 0, 0x14},
	{DISPCTRL_WR_DATA, 0, 0x00},

       {DISPCTRL_WR_CMND, 0xE2,0}, 
	{DISPCTRL_WR_DATA, 0, 0x19},	
       {DISPCTRL_WR_DATA, 0, 0x19},
	{DISPCTRL_WR_DATA, 0, 0x19},	
       {DISPCTRL_WR_DATA, 0, 0x19},
	{DISPCTRL_WR_DATA, 0, 0x19},	
       {DISPCTRL_WR_DATA, 0, 0x19},
       {DISPCTRL_WR_DATA, 0, 0x1A},
	{DISPCTRL_WR_DATA, 0, 0x1A},	
        {DISPCTRL_WR_DATA, 0, 0x1A},
	{DISPCTRL_WR_DATA, 0, 0x1A},	
	{DISPCTRL_WR_DATA, 0, 0x1A},	
       {DISPCTRL_WR_DATA, 0, 0x1A},	
       {DISPCTRL_WR_DATA, 0, 0x19},
	{DISPCTRL_WR_DATA, 0, 0x19},
	{DISPCTRL_WR_DATA, 0, 0x09},	
	{DISPCTRL_WR_DATA, 0, 0x09},

       {DISPCTRL_WR_CMND, 0xE3,0}, 
	{DISPCTRL_WR_DATA, 0, 0x04},
	{DISPCTRL_WR_DATA, 0, 0x04},	
       {DISPCTRL_WR_DATA, 0, 0x04},
	{DISPCTRL_WR_DATA, 0, 0x04},	
       {DISPCTRL_WR_DATA, 0, 0x04},
	{DISPCTRL_WR_DATA, 0, 0x04},
	{DISPCTRL_WR_DATA, 0, 0x04},	
       {DISPCTRL_WR_DATA, 0, 0x04},
	{DISPCTRL_WR_DATA, 0, 0x04},	
       {DISPCTRL_WR_DATA, 0, 0x04},
	{DISPCTRL_WR_DATA, 0, 0x24},
	{DISPCTRL_WR_DATA, 0, 0x24},	
       {DISPCTRL_WR_DATA, 0, 0x24},
	{DISPCTRL_WR_DATA, 0, 0x24},	
       {DISPCTRL_WR_DATA, 0, 0x24},
	{DISPCTRL_WR_DATA, 0, 0x24},
	{DISPCTRL_WR_DATA, 0, 0x24},	
       {DISPCTRL_WR_DATA, 0, 0x24},
	{DISPCTRL_WR_DATA, 0, 0x24},	
       {DISPCTRL_WR_DATA, 0, 0x24},
	{DISPCTRL_WR_DATA, 0, 0x24},
	{DISPCTRL_WR_DATA, 0, 0x24},	
       {DISPCTRL_WR_DATA, 0, 0x24},
	{DISPCTRL_WR_DATA, 0, 0x25},	
       {DISPCTRL_WR_DATA, 0, 0x3D},
	{DISPCTRL_WR_DATA, 0, 0x4D},
	{DISPCTRL_WR_DATA, 0, 0x4D},	
       {DISPCTRL_WR_DATA, 0, 0x4D},
	{DISPCTRL_WR_DATA, 0, 0x4C},	
       {DISPCTRL_WR_DATA, 0, 0x7C},
	{DISPCTRL_WR_DATA, 0, 0x6D},
	{DISPCTRL_WR_DATA, 0, 0x6D},	
       {DISPCTRL_WR_DATA, 0, 0x7D},
	{DISPCTRL_WR_DATA, 0, 0x6D},	
       {DISPCTRL_WR_DATA, 0, 0x6E},
	{DISPCTRL_WR_DATA, 0, 0x6D},
	{DISPCTRL_WR_DATA, 0, 0x6D},	
       {DISPCTRL_WR_DATA, 0, 0x6D},
	{DISPCTRL_WR_DATA, 0, 0x6D},	
       {DISPCTRL_WR_DATA, 0, 0x6D},
	{DISPCTRL_WR_DATA, 0, 0x5C},
	{DISPCTRL_WR_DATA, 0, 0x5C},	
       {DISPCTRL_WR_DATA, 0, 0x5C},
	{DISPCTRL_WR_DATA, 0, 0x6B},	
       {DISPCTRL_WR_DATA, 0, 0x6B},
	{DISPCTRL_WR_DATA, 0, 0x6A},
	{DISPCTRL_WR_DATA, 0, 0x5B},	
	{DISPCTRL_WR_DATA, 0, 0x5B},	
	{DISPCTRL_WR_DATA, 0, 0x53},	
       {DISPCTRL_WR_DATA, 0, 0x53},
	{DISPCTRL_WR_DATA, 0, 0x53},
	{DISPCTRL_WR_DATA, 0, 0x53},	
	{DISPCTRL_WR_DATA, 0, 0x53},		
	{DISPCTRL_WR_DATA, 0, 0x53},	
       {DISPCTRL_WR_DATA, 0, 0x43},
	{DISPCTRL_WR_DATA, 0, 0x33},
	{DISPCTRL_WR_DATA, 0, 0xB4},	
       {DISPCTRL_WR_DATA, 0, 0x94},
	{DISPCTRL_WR_DATA, 0, 0x74},	
       {DISPCTRL_WR_DATA, 0, 0x64},	
	{DISPCTRL_WR_DATA, 0, 0x64},
	{DISPCTRL_WR_DATA, 0, 0x43},		
	{DISPCTRL_WR_DATA, 0, 0x13},	
	{DISPCTRL_WR_DATA, 0, 0x24},	
        /* - Gamma Setting */

      /* + TE ON */
       {DISPCTRL_WR_CMND, 0x35,0}, 
       {DISPCTRL_WR_DATA, 0, 0x00},
      /* - TE ON */

	/* + Display On */
	{DISPCTRL_WR_CMND, 0x11,0},

	{DISPCTRL_SLEEP_MS, 0, 120},

	{DISPCTRL_WR_CMND, 0x2C,0}, 	   

	{DISPCTRL_WR_CMND, 0x29,0}, 

     /*  {DISPCTRL_SLEEP_MS, 0, 200},*/
       {DISPCTRL_SLEEP_MS, 0, 20},

       {DISPCTRL_LIST_END, 0, 0}
       /* - Display On */
};


DISPCTRL_REC_T power_off_seq_ili9486_BOE[] =
{
	{DISPCTRL_WR_CMND, 0x28,0},
	{DISPCTRL_SLEEP_MS, 0, 150},
	{DISPCTRL_WR_CMND, 0x10,0}, 
	{DISPCTRL_SLEEP_MS, 0, 120},	
      {DISPCTRL_LIST_END, 0, 0}
};

#endif /* __BCM_LCD_ILI9486_H */

