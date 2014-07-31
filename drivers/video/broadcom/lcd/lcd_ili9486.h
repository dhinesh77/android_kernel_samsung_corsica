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

#define PANEL_DTC	0x6bc010
#define PANEL_AUO	0x6b4c10
#define PANEL_SHARP	0x6b1c10

const char *LCD_panel_name = "ILI9486 LCD";

// DISP DRV API - Display Info
static DISPDRV_INFO_T Disp_Info = {
	DISPLAY_TYPE_LCD_STD,	// DISPLAY_TYPE_T          type;          
	320,			// UInt32                  width;         
	480,			// UInt32                  height;        
	DISPDRV_FB_FORMAT_RGB888_U, // DISPDRV_FB_FORMAT_T     input_format; //@HW
	//DISPDRV_FB_FORMAT_RGB565,
	DISPLAY_BUS_DSI,	// DISPLAY_BUS_T           bus_type;
	4,			// Bpp; : !!! init may overwrite
};


DISPCTRL_REC_T power_on_seq_ili9486_amazing_BOE[] =
{
       {DISPCTRL_SLEEP_MS, 0, 100},   /* delay after Reset 100 + 100 = 200 */

       /* + Hidden Register Setting */
	{DISPCTRL_WR_CMND, 0xF9,0}, 
	{DISPCTRL_WR_DATA, 0, 0x00},
	{DISPCTRL_WR_DATA, 0, 0x08},	

       {DISPCTRL_WR_CMND, 0xF2,0}, 
	{DISPCTRL_WR_DATA, 0, 0x18},
	{DISPCTRL_WR_DATA, 0, 0xA3},	
       {DISPCTRL_WR_DATA, 0, 0x12},
	{DISPCTRL_WR_DATA, 0, 0x02},	
       {DISPCTRL_WR_DATA, 0, 0x82},
	 {DISPCTRL_WR_DATA, 0, 0x32},
       {DISPCTRL_WR_DATA, 0, 0xFF},
	 {DISPCTRL_WR_DATA, 0, 0x13},
       {DISPCTRL_WR_DATA, 0, 0x00},

        {DISPCTRL_WR_CMND, 0xF1,0}, 
	{DISPCTRL_WR_DATA, 0, 0x36},
	{DISPCTRL_WR_DATA, 0, 0x04},	
       {DISPCTRL_WR_DATA, 0, 0x00},
	{DISPCTRL_WR_DATA, 0, 0x3C},	
       {DISPCTRL_WR_DATA, 0, 0x0F},
	 {DISPCTRL_WR_DATA, 0, 0x8F},
       
       /* - Hidden Register Setting */

        /* + Power Setting Sequence */
       {DISPCTRL_WR_CMND, 0xC0,0}, 
	{DISPCTRL_WR_DATA, 0, 0x07},
	{DISPCTRL_WR_DATA, 0, 0x07},	

       {DISPCTRL_WR_CMND, 0xC1,0}, 
	{DISPCTRL_WR_DATA, 0, 0x45},

       {DISPCTRL_WR_CMND, 0xC2,0}, 
	 {DISPCTRL_WR_DATA, 0, 0x33},
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

       {DISPCTRL_WR_CMND, 0xB0,0}, 
	{DISPCTRL_WR_DATA, 0, 0x80},

       {DISPCTRL_WR_CMND, 0xB1,0}, 
	{DISPCTRL_WR_DATA, 0, 0xB0},   /* C0 = 81Hz, B0 = 67Hz */
	{DISPCTRL_WR_DATA, 0, 0x11},

       {DISPCTRL_WR_CMND, 0xB4,0}, 
	{DISPCTRL_WR_DATA, 0, 0x02},

       {DISPCTRL_WR_CMND, 0xB5,0}, 
	{DISPCTRL_WR_DATA, 0, 0x08},
	{DISPCTRL_WR_DATA, 0, 0x0C},	
       {DISPCTRL_WR_DATA, 0, 0x10},
	{DISPCTRL_WR_DATA, 0, 0x0A},	

       
       {DISPCTRL_WR_CMND, 0xB6,0}, 
	{DISPCTRL_WR_DATA, 0, 0x20},
	{DISPCTRL_WR_DATA, 0, 0x22},	
       {DISPCTRL_WR_DATA, 0, 0x3B},

       {DISPCTRL_WR_CMND, 0xB7,0}, 
	 {DISPCTRL_WR_DATA, 0, 0x07},

       {DISPCTRL_WR_CMND, 0x20,0}, 

       {DISPCTRL_WR_CMND, 0x3A,0}, 
	{DISPCTRL_WR_DATA, 0, 0x66},

       {DISPCTRL_WR_CMND, 0xF7,0}, 
	{DISPCTRL_WR_DATA, 0, 0xA9},
	{DISPCTRL_WR_DATA, 0, 0x91},	
       {DISPCTRL_WR_DATA, 0, 0x2D},
	{DISPCTRL_WR_DATA, 0, 0x8A},	
        /* - Display Parameter Setting */

        /* + Gamma Setting */
       {DISPCTRL_WR_CMND, 0xF8,0}, 
	{DISPCTRL_WR_DATA, 0, 0x21},
	{DISPCTRL_WR_DATA, 0, 0x07},	
       {DISPCTRL_WR_DATA, 0, 0x02},

       {DISPCTRL_WR_CMND, 0xE0,0}, 
	{DISPCTRL_WR_DATA, 0, 0x01},
	{DISPCTRL_WR_DATA, 0, 0x1B},	
       {DISPCTRL_WR_DATA, 0, 0x17},
	{DISPCTRL_WR_DATA, 0, 0x08},	
       {DISPCTRL_WR_DATA, 0, 0x0E},
	{DISPCTRL_WR_DATA, 0, 0x05},	
       {DISPCTRL_WR_DATA, 0, 0x45},
	{DISPCTRL_WR_DATA, 0, 0x77},	
       {DISPCTRL_WR_DATA, 0, 0x3B},
	{DISPCTRL_WR_DATA, 0, 0x07},	
       {DISPCTRL_WR_DATA, 0, 0x12},
	{DISPCTRL_WR_DATA, 0, 0x06},	
       {DISPCTRL_WR_DATA, 0, 0x17},	
       {DISPCTRL_WR_DATA, 0, 0x11},
	{DISPCTRL_WR_DATA, 0, 0x0F},	

       
       {DISPCTRL_WR_CMND, 0xE1,0}, 
	{DISPCTRL_WR_DATA, 0, 0x0F},
	{DISPCTRL_WR_DATA, 0, 0x2A},	
       {DISPCTRL_WR_DATA, 0, 0x23},
	{DISPCTRL_WR_DATA, 0, 0x07},	
       {DISPCTRL_WR_DATA, 0, 0x0B},
	{DISPCTRL_WR_DATA, 0, 0x04},	
       {DISPCTRL_WR_DATA, 0, 0x42},
	{DISPCTRL_WR_DATA, 0, 0x42},	
       {DISPCTRL_WR_DATA, 0, 0x31},
	{DISPCTRL_WR_DATA, 0, 0x04},	
       {DISPCTRL_WR_DATA, 0, 0x10},
	{DISPCTRL_WR_DATA, 0, 0x04},	
       {DISPCTRL_WR_DATA, 0, 0x1F},	
       {DISPCTRL_WR_DATA, 0, 0x1C},
	{DISPCTRL_WR_DATA, 0, 0x01},


       {DISPCTRL_WR_CMND, 0xE2,0}, 
	{DISPCTRL_WR_DATA, 0, 0x09},
	{DISPCTRL_WR_DATA, 0, 0x09},	
       {DISPCTRL_WR_DATA, 0, 0x09},
	{DISPCTRL_WR_DATA, 0, 0x09},	
       {DISPCTRL_WR_DATA, 0, 0x09},
	{DISPCTRL_WR_DATA, 0, 0x09},	
       {DISPCTRL_WR_DATA, 0, 0x09},
	{DISPCTRL_WR_DATA, 0, 0x09},	
       {DISPCTRL_WR_DATA, 0, 0x09},
	{DISPCTRL_WR_DATA, 0, 0x09},	
       {DISPCTRL_WR_DATA, 0, 0x09},
	{DISPCTRL_WR_DATA, 0, 0x09},	
       {DISPCTRL_WR_DATA, 0, 0x09},	
       {DISPCTRL_WR_DATA, 0, 0x09},
	{DISPCTRL_WR_DATA, 0, 0x09},
       {DISPCTRL_WR_DATA, 0, 0x00},


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
        	{DISPCTRL_WR_DATA, 0, 0x04},
	{DISPCTRL_WR_DATA, 0, 0x04},	
       {DISPCTRL_WR_DATA, 0, 0x04},
	{DISPCTRL_WR_DATA, 0, 0x04},	 
        /* - Gamma Setting */

   
      /* + TE ON */
       {DISPCTRL_WR_CMND, 0x35,0}, 
       {DISPCTRL_WR_DATA, 0, 0x00},
      /* - TE ON */


	/* + Display On */

       {DISPCTRL_WR_CMND, 0x36,0}, 
       {DISPCTRL_WR_DATA, 0, 0xDC},

       {DISPCTRL_SLEEP_MS, 0, 10},

       {DISPCTRL_WR_CMND, 0xE9,0}, 
       {DISPCTRL_WR_DATA, 0, 0x18},
       {DISPCTRL_WR_DATA, 0, 0x00},
       {DISPCTRL_WR_DATA, 0, 0xD2},

       {DISPCTRL_SLEEP_MS, 0, 10},     
        
	{DISPCTRL_WR_CMND, 0x11,0},

	{DISPCTRL_SLEEP_MS, 0, 200},

	{DISPCTRL_WR_CMND, 0x29,0}, 

       {DISPCTRL_SLEEP_MS, 0, 100},

       //{DISPCTRL_WR_CMND, 0x2C,0},

       //{DISPCTRL_WR_CMND, 0x3C,0},

       {DISPCTRL_LIST_END, 0, 0}
       /* - Display On */
};


DISPCTRL_REC_T power_off_seq_BOE[] =
{
	{DISPCTRL_WR_CMND, 0x10,0},
	{DISPCTRL_SLEEP_MS, 0, 120},
	{DISPCTRL_WR_CMND, 0x28,0}, 
      {DISPCTRL_LIST_END, 0, 0}
};


DISPCTRL_REC_T power_on_seq_ili9486_sdi[] =
{
	{DISPCTRL_WR_CMND, 0x11,0}, // (SLPOUT)
	{DISPCTRL_LIST_END, 0, 0}
};

#endif /* __BCM_LCD_ILI9486_H */

