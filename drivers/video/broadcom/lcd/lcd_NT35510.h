/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	drivers/video/broadcom/displays/lcd_s6d05a1x01.h
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
*  lcd_NT35510.h
*
*  PURPOSE:
*    This is the LCD-specific code for a NT35510 module.
*
*****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */
#ifndef __BCM_LCD_NT35510_H
#define __BCM_LCD_NT35510_H

//  LCD command definitions
#define PIXEL_FORMAT_RGB565  0x05   // for 16 bits
#define PIXEL_FORMAT_RGB666  0x06   // for 18 bits
#define PIXEL_FORMAT_RGB888  0x07   // for 24 bits

const char *LCD_panel_name = "NT35510 LCD";

// DISP DRV API - Display Info
static DISPDRV_INFO_T Disp_Info =
{
    DISPLAY_TYPE_LCD_STD,         // DISPLAY_TYPE_T          type;          
    480,                          // UInt32                  width;         
    800,                          // UInt32                  height;        
    //DISPDRV_FB_FORMAT_RGB888_U,   // DISPDRV_FB_FORMAT_T     input_format; //@HW
    DISPDRV_FB_FORMAT_RGB565,
    DISPLAY_BUS_DSI,              // DISPLAY_BUS_T           bus_type;
    0,                            // UInt32                  interlaced;    
    DISPDRV_DITHER_NONE,          // DISPDRV_DITHER_T        output_dither; 
    0,                            // UInt32                  pixel_freq;    
    0,                            // UInt32                  line_rate;     
};



DISPCTRL_REC_T power_setting_seq_NT35510_DCS_Type[] =
{
  
       /* + User Set */
      {DISPCTRL_WR_CMND, 0xAA,0},  /* Test Commands */
      {DISPCTRL_WR_DATA, 0, 0x55},
      {DISPCTRL_WR_DATA, 0, 0x25},
      {DISPCTRL_WR_DATA, 0, 0x01},

      {DISPCTRL_WR_CMND, 0xF3,0},  /* Test Commands */
      {DISPCTRL_WR_DATA, 0, 0x00},
      {DISPCTRL_WR_DATA, 0, 0x32},
      {DISPCTRL_WR_DATA, 0, 0x00},
      {DISPCTRL_WR_DATA, 0, 0x38},
      {DISPCTRL_WR_DATA, 0, 0x31},
      {DISPCTRL_WR_DATA, 0, 0x08},
      {DISPCTRL_WR_DATA, 0, 0x11},
      {DISPCTRL_WR_DATA, 0, 0x00},
   
      {DISPCTRL_WR_CMND, 0xF0,0},  /* Manufacture Command Set Selection */
      {DISPCTRL_WR_DATA, 0, 0x55},
      {DISPCTRL_WR_DATA, 0, 0xAA},
      {DISPCTRL_WR_DATA, 0, 0x52},
      {DISPCTRL_WR_DATA, 0, 0x08},
      {DISPCTRL_WR_DATA, 0, 0x00},

      {DISPCTRL_WR_CMND, 0xB0,0},
      {DISPCTRL_WR_DATA, 0, 0x04}, /* DE low enable */
	{DISPCTRL_WR_DATA, 0, 0x0A}, /* VBP (Sony :  10 CLK) */
	{DISPCTRL_WR_DATA, 0, 0x0E}, /* VFP (Sony :  14 CLK) */
	{DISPCTRL_WR_DATA, 0, 0x09}, /* HBP (Sony : 10 CLK) / SEC : 9 CLK */
	{DISPCTRL_WR_DATA, 0, 0x04}, /* HFP (Sony : 10 CLK) / SEC : 4 CLK */

       {DISPCTRL_WR_CMND, 0xB1,0},
	{DISPCTRL_WR_DATA, 0, 0xCC}, /* Backward for Gate */
	{DISPCTRL_WR_DATA, 0, 0x04}, /* 04 : Backward / 00 : Forward */

       {DISPCTRL_WR_CMND, 0x36,0}, /* Data (00 : Forward / 02 : Backward) */
       {DISPCTRL_WR_DATA, 0, 0x02},

       {DISPCTRL_WR_CMND, 0x3A,0},
	{DISPCTRL_WR_DATA, 0, 0x55}, /* RGB565 16 bit*/


      {DISPCTRL_WR_CMND, 0xB3,0}, /* Display clock in RGB Interface */
	{DISPCTRL_WR_DATA, 0, 0x00},

       {DISPCTRL_WR_CMND, 0xB6,0}, /* Source Output Data Hold Time */
	{DISPCTRL_WR_DATA, 0, 0x03},

       {DISPCTRL_WR_CMND, 0xB7,0}, /* GATE EQ */
	{DISPCTRL_WR_DATA, 0, 0x70},
	{DISPCTRL_WR_DATA, 0, 0x70},

       {DISPCTRL_WR_CMND, 0xB8,0}, /* SOURCE EQ */
	{DISPCTRL_WR_DATA, 0, 0x00},
	{DISPCTRL_WR_DATA, 0, 0x06},
	{DISPCTRL_WR_DATA, 0, 0x06},
       {DISPCTRL_WR_DATA, 0, 0x06},


      {DISPCTRL_WR_CMND, 0xBC,0}, /* Inversion Type */
	{DISPCTRL_WR_DATA, 0, 0x00},
	{DISPCTRL_WR_DATA, 0, 0x00},
	{DISPCTRL_WR_DATA, 0, 0x00},

       {DISPCTRL_WR_CMND, 0xBD,0}, /* Display Timing Control */
	{DISPCTRL_WR_DATA, 0, 0x01},
	{DISPCTRL_WR_DATA, 0, 0x84},
	{DISPCTRL_WR_DATA, 0, 0x06},
      {DISPCTRL_WR_DATA, 0, 0x50},
       {DISPCTRL_WR_DATA, 0, 0x00},


       {DISPCTRL_WR_CMND, 0xCC,0}, /* aRD(Gateless) Setting */
	{DISPCTRL_WR_DATA, 0, 0x03},
	{DISPCTRL_WR_DATA, 0, 0x2A},
	{DISPCTRL_WR_DATA, 0, 0x06},
       /* - User Set */

       /* + Power Set */
       {DISPCTRL_WR_CMND, 0xF0,0}, /* Manufacture Command Set Selection */
	{DISPCTRL_WR_DATA, 0, 0x55},
	{DISPCTRL_WR_DATA, 0, 0xAA},
	{DISPCTRL_WR_DATA, 0, 0x52},
       {DISPCTRL_WR_DATA, 0, 0x08},
	{DISPCTRL_WR_DATA, 0, 0x01},

       {DISPCTRL_WR_CMND, 0xB0,0}, /* AVDD Setting */
	{DISPCTRL_WR_DATA, 0, 0x05},
	{DISPCTRL_WR_DATA, 0, 0x05},
	{DISPCTRL_WR_DATA, 0, 0x05},

       {DISPCTRL_WR_CMND, 0xB1,0}, /* AEE Setting */
	{DISPCTRL_WR_DATA, 0, 0x05},
	{DISPCTRL_WR_DATA, 0, 0x05},
	{DISPCTRL_WR_DATA, 0, 0x05},

       {DISPCTRL_WR_CMND, 0xB2,0}, /* VCL Setting for LVGL */
	{DISPCTRL_WR_DATA, 0, 0x03},
	{DISPCTRL_WR_DATA, 0, 0x03},
	{DISPCTRL_WR_DATA, 0, 0x03},

       {DISPCTRL_WR_CMND, 0xB8,0}, /* Control for VCL */
	{DISPCTRL_WR_DATA, 0, 0x24},
	{DISPCTRL_WR_DATA, 0, 0x24},
	{DISPCTRL_WR_DATA, 0, 0x24},

       {DISPCTRL_WR_CMND, 0xB3,0}, /* VGH Setting */
	{DISPCTRL_WR_DATA, 0, 0x0A},
	{DISPCTRL_WR_DATA, 0, 0x0A},
	{DISPCTRL_WR_DATA, 0, 0x0A},

       {DISPCTRL_WR_CMND, 0xB9,0}, /* VGH boosting tiems/frequency */
	{DISPCTRL_WR_DATA, 0, 0x24},
	{DISPCTRL_WR_DATA, 0, 0x24},
	{DISPCTRL_WR_DATA, 0, 0x24},

       {DISPCTRL_WR_CMND, 0xBF,0}, /* VGH output voltage control */
	{DISPCTRL_WR_DATA, 0, 0x01},

       {DISPCTRL_WR_CMND, 0xB5,0}, /* Set VGL_REG voltage (VGL) */
	{DISPCTRL_WR_DATA, 0, 0x08},
	{DISPCTRL_WR_DATA, 0, 0x08},
	{DISPCTRL_WR_DATA, 0, 0x08},

       {DISPCTRL_WR_CMND, 0xB4,0}, /* Set VRGH voltage (VBIAS) */
	{DISPCTRL_WR_DATA, 0, 0x2D},
	{DISPCTRL_WR_DATA, 0, 0x2D},
	{DISPCTRL_WR_DATA, 0, 0x2D},

       {DISPCTRL_WR_CMND, 0xBC,0}, /* VGMP/VGSP voltages */
	{DISPCTRL_WR_DATA, 0, 0x00},
	{DISPCTRL_WR_DATA, 0, 0x50},
	{DISPCTRL_WR_DATA, 0, 0x00},
       
       {DISPCTRL_WR_CMND, 0xBD,0}, /* VGMN/VGSN voltages */
	{DISPCTRL_WR_DATA, 0, 0x00},
	{DISPCTRL_WR_DATA, 0, 0x60},
	{DISPCTRL_WR_DATA, 0, 0x00},


       {DISPCTRL_WR_CMND, 0xCE,0}, /* PWM Commands */
	{DISPCTRL_WR_DATA, 0, 0x00},
	{DISPCTRL_WR_DATA, 0, 0x00},
	{DISPCTRL_WR_DATA, 0, 0x00},
       {DISPCTRL_WR_DATA, 0, 0x00},
	{DISPCTRL_WR_DATA, 0, 0x00},
	{DISPCTRL_WR_DATA, 0, 0x00},
       {DISPCTRL_WR_DATA, 0, 0x00},
       /* - Power Set */


       /* + Gamma Control */
       {DISPCTRL_WR_CMND, 0xD0,0}, /* Gradient Control for Gamma Voltage */
	{DISPCTRL_WR_DATA, 0, 0x0D},
	{DISPCTRL_WR_DATA, 0, 0x15},
	{DISPCTRL_WR_DATA, 0, 0x08},
      {DISPCTRL_WR_DATA, 0, 0x0C},

       {DISPCTRL_WR_CMND, 0xD1,0},  /* POSI (RED) */
	{DISPCTRL_WR_DATA, 0, 0x00},  /* + Gamma Set_1 */
	{DISPCTRL_WR_DATA, 0, 0x37},
	{DISPCTRL_WR_DATA, 0, 0x00},
      {DISPCTRL_WR_DATA, 0, 0x71},
      {DISPCTRL_WR_DATA, 0, 0x00},
	{DISPCTRL_WR_DATA, 0, 0xA2},
	{DISPCTRL_WR_DATA, 0, 0x00},
      {DISPCTRL_WR_DATA, 0, 0xC4},
      {DISPCTRL_WR_DATA, 0, 0x00},
      {DISPCTRL_WR_DATA, 0, 0xDB},

       {DISPCTRL_WR_DATA, 0, 0x01},
	{DISPCTRL_WR_DATA, 0, 0x01},
	{DISPCTRL_WR_DATA, 0, 0x01},
      {DISPCTRL_WR_DATA, 0, 0x40},
      {DISPCTRL_WR_DATA, 0, 0x01},
	{DISPCTRL_WR_DATA, 0, 0x84},
	{DISPCTRL_WR_DATA, 0, 0x01},
      {DISPCTRL_WR_DATA, 0, 0xA9},
      {DISPCTRL_WR_DATA, 0, 0x01},
      {DISPCTRL_WR_DATA, 0, 0xD8},

       {DISPCTRL_WR_DATA, 0, 0x02},
	{DISPCTRL_WR_DATA, 0, 0x0A},
	{DISPCTRL_WR_DATA, 0, 0x02},
      {DISPCTRL_WR_DATA, 0, 0x44},
      {DISPCTRL_WR_DATA, 0, 0x02},
	{DISPCTRL_WR_DATA, 0, 0x85},
	{DISPCTRL_WR_DATA, 0, 0x02},
      {DISPCTRL_WR_DATA, 0, 0x87},
      {DISPCTRL_WR_DATA, 0, 0x02},
      {DISPCTRL_WR_DATA, 0, 0xBF},

       {DISPCTRL_WR_DATA, 0, 0x02},
	{DISPCTRL_WR_DATA, 0, 0xE5},
	{DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0x0F},
      {DISPCTRL_WR_DATA, 0, 0x03},
	{DISPCTRL_WR_DATA, 0, 0x34},
	{DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0x4F},
      {DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0x73},

       {DISPCTRL_WR_DATA, 0, 0x03},
	{DISPCTRL_WR_DATA, 0, 0x77},
	{DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0x94},
      {DISPCTRL_WR_DATA, 0, 0x03},
	{DISPCTRL_WR_DATA, 0, 0x9E},
	{DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0xAC},
      {DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0xBD},

      {DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0xF1}, /* - Gamma Set_1 */


       {DISPCTRL_WR_CMND, 0xD2,0},  /* POSI (GREEN) */
	{DISPCTRL_WR_DATA, 0, 0x00},  /* + Gamma Set_1 */
	{DISPCTRL_WR_DATA, 0, 0x37},
	{DISPCTRL_WR_DATA, 0, 0x00},
      {DISPCTRL_WR_DATA, 0, 0x71},
      {DISPCTRL_WR_DATA, 0, 0x00},
	{DISPCTRL_WR_DATA, 0, 0xA2},
	{DISPCTRL_WR_DATA, 0, 0x00},
      {DISPCTRL_WR_DATA, 0, 0xC4},
      {DISPCTRL_WR_DATA, 0, 0x00},
      {DISPCTRL_WR_DATA, 0, 0xDB},

       {DISPCTRL_WR_DATA, 0, 0x01},
	{DISPCTRL_WR_DATA, 0, 0x01},
	{DISPCTRL_WR_DATA, 0, 0x01},
      {DISPCTRL_WR_DATA, 0, 0x40},
      {DISPCTRL_WR_DATA, 0, 0x01},
	{DISPCTRL_WR_DATA, 0, 0x84},
	{DISPCTRL_WR_DATA, 0, 0x01},
      {DISPCTRL_WR_DATA, 0, 0xA9},
      {DISPCTRL_WR_DATA, 0, 0x01},
      {DISPCTRL_WR_DATA, 0, 0xD8},

       {DISPCTRL_WR_DATA, 0, 0x02},
	{DISPCTRL_WR_DATA, 0, 0x0A},
	{DISPCTRL_WR_DATA, 0, 0x02},
      {DISPCTRL_WR_DATA, 0, 0x44},
      {DISPCTRL_WR_DATA, 0, 0x02},
	{DISPCTRL_WR_DATA, 0, 0x85},
	{DISPCTRL_WR_DATA, 0, 0x02},
      {DISPCTRL_WR_DATA, 0, 0x87},
      {DISPCTRL_WR_DATA, 0, 0x02},
      {DISPCTRL_WR_DATA, 0, 0xBF},

       {DISPCTRL_WR_DATA, 0, 0x02},
	{DISPCTRL_WR_DATA, 0, 0xE5},
	{DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0x0F},
      {DISPCTRL_WR_DATA, 0, 0x03},
	{DISPCTRL_WR_DATA, 0, 0x34},
	{DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0x4F},
      {DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0x73},

       {DISPCTRL_WR_DATA, 0, 0x03},
	{DISPCTRL_WR_DATA, 0, 0x77},
	{DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0x94},
      {DISPCTRL_WR_DATA, 0, 0x03},
	{DISPCTRL_WR_DATA, 0, 0x9E},
	{DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0xAC},
      {DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0xBD},

      {DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0xF1}, /* - Gamma Set_1 */

       {DISPCTRL_WR_CMND, 0xD3,0},  /* POSI (BLUE) */
	{DISPCTRL_WR_DATA, 0, 0x00},  /* + Gamma Set_1 */
	{DISPCTRL_WR_DATA, 0, 0x37},
	{DISPCTRL_WR_DATA, 0, 0x00},
      {DISPCTRL_WR_DATA, 0, 0x71},
      {DISPCTRL_WR_DATA, 0, 0x00},
	{DISPCTRL_WR_DATA, 0, 0xA2},
	{DISPCTRL_WR_DATA, 0, 0x00},
      {DISPCTRL_WR_DATA, 0, 0xC4},
      {DISPCTRL_WR_DATA, 0, 0x00},
      {DISPCTRL_WR_DATA, 0, 0xDB},

       {DISPCTRL_WR_DATA, 0, 0x01},
	{DISPCTRL_WR_DATA, 0, 0x01},
	{DISPCTRL_WR_DATA, 0, 0x01},
      {DISPCTRL_WR_DATA, 0, 0x40},
      {DISPCTRL_WR_DATA, 0, 0x01},
	{DISPCTRL_WR_DATA, 0, 0x84},
	{DISPCTRL_WR_DATA, 0, 0x01},
      {DISPCTRL_WR_DATA, 0, 0xA9},
      {DISPCTRL_WR_DATA, 0, 0x01},
      {DISPCTRL_WR_DATA, 0, 0xD8},

       {DISPCTRL_WR_DATA, 0, 0x02},
	{DISPCTRL_WR_DATA, 0, 0x0A},
	{DISPCTRL_WR_DATA, 0, 0x02},
      {DISPCTRL_WR_DATA, 0, 0x44},
      {DISPCTRL_WR_DATA, 0, 0x02},
	{DISPCTRL_WR_DATA, 0, 0x85},
	{DISPCTRL_WR_DATA, 0, 0x02},
      {DISPCTRL_WR_DATA, 0, 0x87},
      {DISPCTRL_WR_DATA, 0, 0x02},
      {DISPCTRL_WR_DATA, 0, 0xBF},

       {DISPCTRL_WR_DATA, 0, 0x02},
	{DISPCTRL_WR_DATA, 0, 0xE5},
	{DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0x0F},
      {DISPCTRL_WR_DATA, 0, 0x03},
	{DISPCTRL_WR_DATA, 0, 0x34},
	{DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0x4F},
      {DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0x73},

       {DISPCTRL_WR_DATA, 0, 0x03},
	{DISPCTRL_WR_DATA, 0, 0x77},
	{DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0x94},
      {DISPCTRL_WR_DATA, 0, 0x03},
	{DISPCTRL_WR_DATA, 0, 0x9E},
	{DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0xAC},
      {DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0xBD},

      {DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0xF1}, /* - Gamma Set_1 */


      {DISPCTRL_WR_CMND, 0xD4,0},  /* NEGA (RED) */
	{DISPCTRL_WR_DATA, 0, 0x00},  /* + Gamma Set_2 */
	{DISPCTRL_WR_DATA, 0, 0x37},
	{DISPCTRL_WR_DATA, 0, 0x00},
      {DISPCTRL_WR_DATA, 0, 0x46},
      {DISPCTRL_WR_DATA, 0, 0x00},
	{DISPCTRL_WR_DATA, 0, 0x7E},
	{DISPCTRL_WR_DATA, 0, 0x00},
      {DISPCTRL_WR_DATA, 0, 0x9E},
      {DISPCTRL_WR_DATA, 0, 0x00},
      {DISPCTRL_WR_DATA, 0, 0xC2},

       {DISPCTRL_WR_DATA, 0, 0x01},
	{DISPCTRL_WR_DATA, 0, 0x01},
	{DISPCTRL_WR_DATA, 0, 0x01},
      {DISPCTRL_WR_DATA, 0, 0x14},
      {DISPCTRL_WR_DATA, 0, 0x01},
	{DISPCTRL_WR_DATA, 0, 0x4A},
	{DISPCTRL_WR_DATA, 0, 0x01},
      {DISPCTRL_WR_DATA, 0, 0x73},
      {DISPCTRL_WR_DATA, 0, 0x01},
      {DISPCTRL_WR_DATA, 0, 0xB8},

       {DISPCTRL_WR_DATA, 0, 0x01},
	{DISPCTRL_WR_DATA, 0, 0xDF},
	{DISPCTRL_WR_DATA, 0, 0x02},
      {DISPCTRL_WR_DATA, 0, 0x2F},
      {DISPCTRL_WR_DATA, 0, 0x02},
	{DISPCTRL_WR_DATA, 0, 0x68},
	{DISPCTRL_WR_DATA, 0, 0x02},
      {DISPCTRL_WR_DATA, 0, 0x6A},
      {DISPCTRL_WR_DATA, 0, 0x02},
      {DISPCTRL_WR_DATA, 0, 0xA3},

       {DISPCTRL_WR_DATA, 0, 0x02},
	{DISPCTRL_WR_DATA, 0, 0xE0},
	{DISPCTRL_WR_DATA, 0, 0x02},
      {DISPCTRL_WR_DATA, 0, 0xF9},
      {DISPCTRL_WR_DATA, 0, 0x03},
	{DISPCTRL_WR_DATA, 0, 0x25},
	{DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0x43},
      {DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0x6E},

       {DISPCTRL_WR_DATA, 0, 0x03},
	{DISPCTRL_WR_DATA, 0, 0x77},
	{DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0x94},
      {DISPCTRL_WR_DATA, 0, 0x03},
	{DISPCTRL_WR_DATA, 0, 0x9E},
	{DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0xAC},
      {DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0xBD},

      {DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0xF1}, /* - Gamma Set_2 */
       

       {DISPCTRL_WR_CMND, 0xD5,0},  /* NEGA (GREEN) */
	{DISPCTRL_WR_DATA, 0, 0x00},  /* + Gamma Set_2 */
	{DISPCTRL_WR_DATA, 0, 0x37},
	{DISPCTRL_WR_DATA, 0, 0x00},
      {DISPCTRL_WR_DATA, 0, 0x46},
      {DISPCTRL_WR_DATA, 0, 0x00},
	{DISPCTRL_WR_DATA, 0, 0x7E},
	{DISPCTRL_WR_DATA, 0, 0x00},
      {DISPCTRL_WR_DATA, 0, 0x9E},
      {DISPCTRL_WR_DATA, 0, 0x00},
      {DISPCTRL_WR_DATA, 0, 0xC2},

       {DISPCTRL_WR_DATA, 0, 0x01},
	{DISPCTRL_WR_DATA, 0, 0x01},
	{DISPCTRL_WR_DATA, 0, 0x01},
      {DISPCTRL_WR_DATA, 0, 0x14},
      {DISPCTRL_WR_DATA, 0, 0x01},
	{DISPCTRL_WR_DATA, 0, 0x4A},
	{DISPCTRL_WR_DATA, 0, 0x01},
      {DISPCTRL_WR_DATA, 0, 0x73},
      {DISPCTRL_WR_DATA, 0, 0x01},
      {DISPCTRL_WR_DATA, 0, 0xB8},

       {DISPCTRL_WR_DATA, 0, 0x01},
	{DISPCTRL_WR_DATA, 0, 0xDF},
	{DISPCTRL_WR_DATA, 0, 0x02},
      {DISPCTRL_WR_DATA, 0, 0x2F},
      {DISPCTRL_WR_DATA, 0, 0x02},
	{DISPCTRL_WR_DATA, 0, 0x68},
	{DISPCTRL_WR_DATA, 0, 0x02},
      {DISPCTRL_WR_DATA, 0, 0x6A},
      {DISPCTRL_WR_DATA, 0, 0x02},
      {DISPCTRL_WR_DATA, 0, 0xA3},

       {DISPCTRL_WR_DATA, 0, 0x02},
	{DISPCTRL_WR_DATA, 0, 0xE0},
	{DISPCTRL_WR_DATA, 0, 0x02},
      {DISPCTRL_WR_DATA, 0, 0xF9},
      {DISPCTRL_WR_DATA, 0, 0x03},
	{DISPCTRL_WR_DATA, 0, 0x25},
	{DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0x43},
      {DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0x6E},

       {DISPCTRL_WR_DATA, 0, 0x03},
	{DISPCTRL_WR_DATA, 0, 0x77},
	{DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0x94},
      {DISPCTRL_WR_DATA, 0, 0x03},
	{DISPCTRL_WR_DATA, 0, 0x9E},
	{DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0xAC},
      {DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0xBD},

      {DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0xF1}, /* - Gamma Set_2 */
       

       {DISPCTRL_WR_CMND, 0xD6,0},  /* NEGA (BLUE) */
	{DISPCTRL_WR_DATA, 0, 0x00},  /* + Gamma Set_2 */
	{DISPCTRL_WR_DATA, 0, 0x37},
	{DISPCTRL_WR_DATA, 0, 0x00},
      {DISPCTRL_WR_DATA, 0, 0x46},
      {DISPCTRL_WR_DATA, 0, 0x00},
	{DISPCTRL_WR_DATA, 0, 0x7E},
	{DISPCTRL_WR_DATA, 0, 0x00},
      {DISPCTRL_WR_DATA, 0, 0x9E},
      {DISPCTRL_WR_DATA, 0, 0x00},
      {DISPCTRL_WR_DATA, 0, 0xC2},

       {DISPCTRL_WR_DATA, 0, 0x01},
	{DISPCTRL_WR_DATA, 0, 0x01},
	{DISPCTRL_WR_DATA, 0, 0x01},
      {DISPCTRL_WR_DATA, 0, 0x14},
      {DISPCTRL_WR_DATA, 0, 0x01},
	{DISPCTRL_WR_DATA, 0, 0x4A},
	{DISPCTRL_WR_DATA, 0, 0x01},
      {DISPCTRL_WR_DATA, 0, 0x73},
      {DISPCTRL_WR_DATA, 0, 0x01},
      {DISPCTRL_WR_DATA, 0, 0xB8},

       {DISPCTRL_WR_DATA, 0, 0x01},
	{DISPCTRL_WR_DATA, 0, 0xDF},
	{DISPCTRL_WR_DATA, 0, 0x02},
      {DISPCTRL_WR_DATA, 0, 0x2F},
      {DISPCTRL_WR_DATA, 0, 0x02},
	{DISPCTRL_WR_DATA, 0, 0x68},
	{DISPCTRL_WR_DATA, 0, 0x02},
      {DISPCTRL_WR_DATA, 0, 0x6A},
      {DISPCTRL_WR_DATA, 0, 0x02},
      {DISPCTRL_WR_DATA, 0, 0xA3},

       {DISPCTRL_WR_DATA, 0, 0x02},
	{DISPCTRL_WR_DATA, 0, 0xE0},
	{DISPCTRL_WR_DATA, 0, 0x02},
      {DISPCTRL_WR_DATA, 0, 0xF9},
      {DISPCTRL_WR_DATA, 0, 0x03},
	{DISPCTRL_WR_DATA, 0, 0x25},
	{DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0x43},
      {DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0x6E},

       {DISPCTRL_WR_DATA, 0, 0x03},
	{DISPCTRL_WR_DATA, 0, 0x77},
	{DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0x94},
      {DISPCTRL_WR_DATA, 0, 0x03},
	{DISPCTRL_WR_DATA, 0, 0x9E},
	{DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0xAC},
      {DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0xBD},

      {DISPCTRL_WR_DATA, 0, 0x03},
      {DISPCTRL_WR_DATA, 0, 0xF1}, /* - Gamma Set_2 */
       
      /* - Gamma Control */


       /* + Display On */

       /* + TE ON */
       {DISPCTRL_WR_CMND, 0x35,0}, 
       {DISPCTRL_WR_DATA, 0, 0x00},
      /* - TE ON */

       {DISPCTRL_WR_CMND, 0x11,0},  /* Sleep Out */
     
       {DISPCTRL_SLEEP_MS, 0, 120}, 	/* Wait 120ms */


       {DISPCTRL_WR_CMND, 0x51,0}, /* PWM */
	{DISPCTRL_WR_DATA, 0, 0x6C},  

       {DISPCTRL_WR_CMND, 0x53,0},
	{DISPCTRL_WR_DATA, 0, 0x2C},  
        
       {DISPCTRL_WR_CMND, 0x29,0}, /* Display On */

       /* - Display On */

      {DISPCTRL_LIST_END, 0, 0}
};



DISPCTRL_REC_T power_off_seq_NT35510_DCS_Type[] =
{
	{DISPCTRL_WR_CMND, 0x28,0}, 
       {DISPCTRL_LIST_END, 0, 0}
};



DISPCTRL_REC_T sleep_out_seq_DCS_Type[] =
{

	/* + Sleep Out*/
    {DISPCTRL_WR_CMND, 0x11,0},
	/* - Sleep Out*/

	/* Wait 120ms */
    {DISPCTRL_SLEEP_MS, 0, 120},

};

DISPCTRL_REC_T power_off_seq_DCS_Type[] =
{
	{DISPCTRL_WR_CMND, 0x28,0}, 
	{DISPCTRL_WR_CMND, 0x10,0}, 
	{DISPCTRL_SLEEP_MS, 0, 120},
	{DISPCTRL_LIST_END, 0, 0}	
};


DISPCTRL_REC_T power_on_seq_DCS_Type[] =
{
	{DISPCTRL_WR_CMND, 0x11,0}, // (SLPOUT)
	{DISPCTRL_SLEEP_MS, 0, 120},
	{DISPCTRL_WR_CMND, 0x29,0}, // Display on
	{DISPCTRL_LIST_END, 0, 0}
};


#endif /* __BCM_LCD_NT35510_H */

