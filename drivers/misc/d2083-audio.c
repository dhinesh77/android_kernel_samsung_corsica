/*
 * d2083-audio.c: Audio amplifier driver for d2083
 *   
 * Copyright(c) 2012 Dialog Semiconductor Ltd.
 *  
 * Author: Dialog Semiconductor Ltd. D. Chen, Roy Im
 *  
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/d2083/d2083_reg.h>
#include <linux/d2083/core.h>
#include <linux/d2083/audio.h>
#include <linux/delay.h>

#define DRIVER_NAME "d2083-audio"
#define LIMITED_SWAP_PRE 0x13
//#define NOISE_GATE_USE

static struct d2083 *d2083;
struct d2083_audio *dlg_audio = NULL;

static u8 dlg_audio_sleep = true;
static u8 dlg_set_ng1 = 0x00;
static u8 dlg_set_ng2 = 0x00;
static u8 is_playback_stop = false;
static u8 ihf_power_status = 0;
static bool hs_pw_status = 0;

static inline u8 audio_read(int reg)
{
	u8 val;

	d2083_reg_read(d2083, reg, &val);

	return val;
}

static inline int audio_write(int reg, u8 const val)
{
	//dlg_info("%s, Reg[0x%x] val[0x%x]\n",__func__,reg,val);     
	return d2083_reg_write(d2083, reg, val);
}

static inline int audio_read_block(int start_reg, int len, u8 *dest)
{
	return d2083_block_read(d2083, start_reg, len, dest);
}

static inline int audio_write_block(int start_reg, int len, u8 *src)
{
	return d2083_block_write(d2083, start_reg, len, src);
}

static inline u8 audio_set_bits(int reg, u8 mask)
{
	return d2083_set_bits(d2083, reg, mask);
}

static inline int audio_clear_bits(int reg, u8 mask)
{
	return d2083_clear_bits(d2083, reg, mask);
}


/*
 * API functions
 */

static inline int d2083_audio_poweron(bool on)
{
	int ret = 0;
	int (*set_bits)(struct d2083 * const d2083, u8 const reg, u8 const mask);

	if(on==true) {
		audio_write(D2083_LDO1_MCTL_REG, 0x54);
		audio_write(D2083_LDO_AUD_MCTL_REG, 0x54);
	}	
	set_bits = (on) ? d2083_set_bits : d2083_clear_bits;

	ret |= set_bits(d2083, D2083_PREAMP_A_CTRL1_REG, D2083_PREAMP_EN);
	ret |= set_bits(d2083, D2083_PREAMP_B_CTRL1_REG, D2083_PREAMP_EN);
	ret |= set_bits(d2083, D2083_MXHPR_CTRL_REG, D2083_MX_EN);
	ret |= set_bits(d2083, D2083_MXHPL_CTRL_REG, D2083_MX_EN);
	ret |= set_bits(d2083, D2083_CP_CTRL_REG, D2083_CP_EN);

	if(on==false){
		audio_write(D2083_CP_CTRL_REG,0xC9);
		audio_write(D2083_LDO1_MCTL_REG, 0x00);
		audio_write(D2083_LDO_AUD_MCTL_REG, 0x00);
	}

	return ret;
}


int d2083_audio_hs_poweron1(bool on)
{
    int ret = 0;
    u8 regval;

	//dlg_info("%s status:%d, cmd:%d\n",__func__,hs_pw_status, on);     
	
	if ( hs_pw_status == on){
		return ret;
	}
	hs_pw_status = on;
	
	dlg_info("%s HP=%d speaker_power=%d \n",__func__,on,dlg_audio->IHFenabled);     

    if(on) 
	{
		//if(dlg_audio->IHFenabled==false)
		//{
		    //audio_write(D2083_LDO_AUD_MCTL_REG, 0x44); //AUD_LDO on
		    //audio_write(D2083_LDO1_MCTL_REG, 0x54);
		    //mdelay(20);
		//}

		if (!dlg_audio_sleep) {
			ret |= audio_write(D2083_HP_L_CTRL_REG,0xF0);
			ret |= audio_write(D2083_HP_R_CTRL_REG,0xF0);

			ret = audio_write(D2083_HP_L_GAIN_REG,0x00);
			ret = audio_write(D2083_HP_R_GAIN_REG,0x00); 

			ret = audio_write(D2083_HP_NG1_REG,0x10);
			msleep(30); // will test for pop noise newly
		}

		//dlg_audio->hs_pga_gain -= (LIMITED_SWAP_PRE - dlg_audio->hs_pre_gain-1);
		//dlg_audio->hs_pre_gain += (LIMITED_SWAP_PRE - dlg_audio->hs_pre_gain);
#ifndef USE_AUDIO_DIFFERENTIAL
		regval = audio_read(D2083_PREAMP_B_CTRL1_REG) & ~D2083_PREAMP_VOL;
		regval |= ((dlg_audio->hs_pre_gain << D2083_PREAMP_VOL_SHIFT) & D2083_PREAMP_VOL);
		regval |= (D2083_PREAMP_EN | D2083_PREAMP_ZC_EN);
		regval &= ~D2083_PREAMP_MUTE;

		ret |= audio_write(D2083_PREAMP_B_CTRL1_REG,regval);
		ret |= audio_write(D2083_PREAMP_B_CTRL2_REG,0x00); 

		ret |= audio_write(D2083_MXHPR_CTRL_REG,0x11);
		ret |= audio_write(D2083_MXHPL_CTRL_REG,0x09);   

#else
		regval = audio_read(D2083_PREAMP_B_CTRL1_REG) & ~D2083_PREAMP_VOL;
		regval |= ((dlg_audio->hs_pre_gain << D2083_PREAMP_VOL_SHIFT) & D2083_PREAMP_VOL);
		regval |= (D2083_PREAMP_EN | D2083_PREAMP_ZC_EN);
		regval &= ~D2083_PREAMP_MUTE;

		ret |= audio_write(D2083_PREAMP_B_CTRL1_REG,regval);
		ret |= audio_write(D2083_PREAMP_B_CTRL2_REG,0x03); 

		    if(dlg_audio->IHFenabled == false) {
		regval = audio_read(D2083_PREAMP_A_CTRL1_REG) & ~D2083_PREAMP_VOL;
		regval |= ((dlg_audio->hs_pre_gain << D2083_PREAMP_VOL_SHIFT) & D2083_PREAMP_VOL);
		regval |= (D2083_PREAMP_EN | D2083_PREAMP_ZC_EN);
		regval &= ~D2083_PREAMP_MUTE;

		ret |= audio_write(D2083_PREAMP_A_CTRL1_REG,regval);
		ret |= audio_write(D2083_PREAMP_A_CTRL2_REG,0x03); 
		    }

		    // HP Mixer controll
		    if(dlg_audio->IHFenabled == false){
			    audio_write(D2083_MXHPL_CTRL_REG,0x07); 
		    } else {
			    audio_write(D2083_MXHPL_CTRL_REG,0x19); 
		    }
			    audio_write(D2083_MXHPR_CTRL_REG,0x19);
#endif

		regval = audio_read(D2083_HP_L_GAIN_REG) & ~D2042_HP_AMP_GAIN & ~D2083_HP_AMP_MUTE_EN; 
		ret |= audio_write(D2083_HP_L_GAIN_REG, regval | (dlg_audio->hs_pga_gain & D2042_HP_AMP_GAIN));

		regval = audio_read(D2083_HP_R_GAIN_REG) & ~D2042_HP_AMP_GAIN & ~D2083_HP_AMP_MUTE_EN;
		ret |= audio_write(D2083_HP_R_GAIN_REG, regval | (dlg_audio->hs_pga_gain & D2042_HP_AMP_GAIN));

		ret |= audio_write(D2083_CP_CTRL_REG,0xC9);
		ret |= audio_write(D2083_CP_DELAY_REG,0x85);

		ret |= audio_write(D2083_CP_DETECTOR_REG,0x00);
		ret |= audio_write(D2083_CP_VOL_THRESHOLD_REG,0x32);
		ret |= audio_write(D2083_BIAS_CTRL_REG,0x3F);

		msleep(65);
		
#ifndef USE_AUDIO_DIFFERENTIAL
		regval = audio_read(D2083_PREAMP_B_CTRL1_REG);
		regval &= ~D2083_PREAMP_ZC_EN;       
		ret |= audio_write(D2083_PREAMP_B_CTRL1_REG,regval);
#else
		regval = audio_read(D2083_PREAMP_B_CTRL1_REG);
		regval &= ~D2083_PREAMP_ZC_EN;       
		ret |= audio_write(D2083_PREAMP_B_CTRL1_REG,regval);
		
		if(dlg_audio->IHFenabled == false){
		regval = audio_read(D2083_PREAMP_A_CTRL1_REG);
		regval &= ~D2083_PREAMP_ZC_EN;       
		ret |= audio_write(D2083_PREAMP_A_CTRL1_REG,regval);
		}
#endif
		ret = audio_write(D2083_HP_NG1_REG, dlg_set_ng1);
		ret = audio_write(D2083_HP_NG2_REG, dlg_set_ng2);

		ret |= audio_write(D2083_HP_L_CTRL_REG,0xa0);
		ret |= audio_write(D2083_HP_R_CTRL_REG,0xa0);

		msleep(25);
		ret |= audio_write(D2083_CP_CTRL_REG,0xCD);

		msleep(40);
		dlg_audio->HSenabled = true;

	}
	else
	{

#ifndef USE_AUDIO_DIFFERENTIAL
		regval = audio_read(D2083_PREAMP_B_CTRL1_REG) & ~D2083_PREAMP_VOL;
		regval |= (D2083_PREAMP_MUTE);
		ret |= audio_write(D2083_PREAMP_B_CTRL1_REG,regval);
#else		
		regval = audio_read(D2083_PREAMP_B_CTRL1_REG) & ~D2083_PREAMP_VOL;
		regval |= (D2083_PREAMP_MUTE);
		ret |= audio_write(D2083_PREAMP_B_CTRL1_REG,regval);
		
		if(dlg_audio->IHFenabled != true) {
		regval = audio_read(D2083_PREAMP_A_CTRL1_REG) & ~D2083_PREAMP_VOL;
		regval |= (D2083_PREAMP_MUTE);
		ret |= audio_write(D2083_PREAMP_A_CTRL1_REG,regval);
		}
#endif
		ret = audio_write(D2083_HP_L_GAIN_REG, dlg_audio->hs_pga_gain -2);
		ret = audio_write(D2083_HP_R_GAIN_REG, dlg_audio->hs_pga_gain -2); 
		msleep(10);

		ret = audio_write(D2083_HP_L_GAIN_REG, dlg_audio->hs_pga_gain -5);
		ret = audio_write(D2083_HP_R_GAIN_REG, dlg_audio->hs_pga_gain -5); 
		msleep(10);

		ret = audio_write(D2083_HP_L_GAIN_REG, dlg_audio->hs_pga_gain -10);
		ret = audio_write(D2083_HP_R_GAIN_REG, dlg_audio->hs_pga_gain -10); 
		msleep(10);

		ret = audio_write(D2083_HP_NG1_REG,0x11); 
		ret = audio_write(D2083_HP_NG2_REG,0x07);    

		if(dlg_audio->IHFenabled != true)
		msleep(50);

		ret = audio_write(D2083_HP_L_GAIN_REG,0x00);
		ret = audio_write(D2083_HP_R_GAIN_REG,0x00); 
		msleep(10);

		audio_write(D2083_CP_CTRL_REG,0xC9);
			
		dlg_audio_sleep = false;
		dlg_audio->HSenabled = false;

		// if(dlg_audio->IHFenabled==false) {
		//     audio_write(D2083_LDO_AUD_MCTL_REG, 0x00); //AUD_LDO off
		//	 audio_write(D2083_LDO1_MCTL_REG, 0x54);
		// }

	}

	return ret;
}


int d2083_audio_hs_poweron(bool on)
{
	if(dlg_audio->AudioStart==0)
	{
		del_timer(&dlg_audio->timer); 
		mod_timer(&dlg_audio->timer, jiffies + msecs_to_jiffies(10));
		dlg_audio->AudioStart=1;
	}
	else
		d2083_audio_hs_poweron1(on);
		
	return 0;
}
EXPORT_SYMBOL(d2083_audio_hs_poweron);


int d2083_audio_hs_shortcircuit_enable(bool en)
{
	dlg_err("%s(%s): NOT SUPPORTED.\n", __func__, en ? "on" : "off");
	return -EINVAL;
}
EXPORT_SYMBOL(d2083_audio_hs_shortcircuit_enable);

int d2083_audio_hs_set_gain(enum d2083_audio_output_sel hs_path_sel, enum d2083_hp_vol_val hs_gain_val)
{
	int ret = 0;
	u8 regval;
	
	if(D2083_HPVOL_NEG_57DB > hs_gain_val || D2083_HPVOL_MUTE < hs_gain_val) {
		dlg_info("[%s]- Invalid gain, so set to -1dB \n", __func__);
		hs_gain_val = D2083_HPVOL_NEG_1DB;				
	}
	
	dlg_info("d2083_audio_hs_set_gain path=%d hs_pga_gain=0x%x \n", hs_path_sel, hs_gain_val);
	
	if(dlg_audio->HSenabled==true)
	{
		if (hs_path_sel & D2083_OUT_HPL) 
		{
			regval = audio_read(D2083_HP_L_GAIN_REG) & ~D2042_HP_AMP_GAIN;
			ret |= audio_write(D2083_HP_L_GAIN_REG, regval | (hs_gain_val & D2042_HP_AMP_GAIN));
			ret |= audio_clear_bits(D2083_HP_L_CTRL_REG, D2083_HP_AMP_MUTE_EN);
		}

		if (hs_path_sel & D2083_OUT_HPR) 
		{
			regval = audio_read(D2083_HP_R_GAIN_REG) & ~D2042_HP_AMP_GAIN;
			ret |= audio_write(D2083_HP_R_GAIN_REG, regval | (hs_gain_val & D2042_HP_AMP_GAIN));
			ret |= audio_clear_bits(D2083_HP_R_CTRL_REG, D2083_HP_AMP_MUTE_EN);
		}            

	}
	dlg_audio->hs_pga_gain=hs_gain_val;

	return ret;
}
EXPORT_SYMBOL(d2083_audio_hs_set_gain);

// speaker on/off
int d2083_audio_hs_ihf_poweron1(void)
{
	int ret = 0;
	u8 regval;

	if(ihf_power_status == true)
	 return ret;
	ihf_power_status = true;
	 
	dlg_info("%s HP =%d \n",__func__, dlg_audio->HSenabled);

	// if(dlg_audio->HSenabled==false)
	//     audio_write(D2083_LDO_AUD_MCTL_REG, 0x44); //AUD_LDO on

	audio_write(D2083_MXHPL_CTRL_REG,0x19); 

	regval = audio_read(D2083_PREAMP_A_CTRL1_REG);
	regval = (regval | D2083_PREAMP_EN) & ~D2083_PREAMP_MUTE;
	ret |= audio_write(D2083_PREAMP_A_CTRL1_REG,regval);
	ret |= audio_write(D2083_PREAMP_A_CTRL2_REG,0x03); //pre amp A fully differential

	regval = audio_read(D2083_SP_CTRL_REG);
	regval &= ~D2083_SP_MUTE;
	regval |= D2083_SP_EN;
	ret |= audio_write(D2083_SP_CTRL_REG,regval);

	ret |= audio_write(D2083_SP_CFG1_REG,0x00); 
	ret |= audio_write(D2083_SP_CFG2_REG,0x00); 
	//ret |= audio_write(D2083_SP_NG1_REG,0x0F); 
	//ret |= audio_write(D2083_SP_NG2_REG,0x07); 
	ret |= audio_write(D2083_SP_NON_CLIP_ZC_REG,0x00);
	ret |= audio_write(D2083_SP_NON_CLIP_REG,0x00); 
	ret |= audio_write(D2083_SP_PWR_REG,0x00);

	ret |= audio_write(D2083_BIAS_CTRL_REG,0x3F); 
	dlg_audio->IHFenabled = true;

	return ret;
}

int d2083_audio_hs_ihf_poweron(void)
{
	if(dlg_audio->AudioStart==0)
	{
		del_timer(&dlg_audio->timer); 
		mod_timer(&dlg_audio->timer, jiffies + msecs_to_jiffies(20));
		dlg_audio->AudioStart=2;
	}
	else
	{
		d2083_audio_hs_ihf_poweron1();
	}
	return 0;
}

EXPORT_SYMBOL(d2083_audio_hs_ihf_poweron);

int d2083_audio_hs_ihf_poweroff(void)
{
	int ret = 0;
	u8 regval;

	if(ihf_power_status ==0){
		return ret;
	}
	ihf_power_status = 0;
	dlg_info("%s, HP =%d, status:%d \n", __func__,dlg_audio->HSenabled,ihf_power_status);

	audio_write(D2083_SP_CTRL_REG,0x32);

#ifdef USE_AUDIO_DIFFERENTIAL
	if (dlg_audio->HSenabled == false) 
#endif
	{
		regval = audio_read(D2083_PREAMP_A_CTRL1_REG) & ~D2083_PREAMP_VOL;
		regval |= (D2083_PREAMP_MUTE);
		regval &= (~D2083_PREAMP_EN);
		ret |= audio_write(D2083_PREAMP_A_CTRL1_REG,regval);
	}

	audio_write(D2083_SP_PWR_REG, 0x00);
	audio_write(D2083_SP_NON_CLIP_ZC_REG, 0x00);
	audio_write(D2083_SP_NON_CLIP_REG, 0x00);
	audio_write(D2083_SP_NG1_REG, 0x00);
	audio_write(D2083_SP_NG2_REG, 0x00);
	
	dlg_audio->IHFenabled = false;
	//if(dlg_audio->HSenabled==false)
	//    audio_write(D2083_LDO_AUD_MCTL_REG, 0x00); //AUD_LDO off

	return ret;
}
EXPORT_SYMBOL(d2083_audio_hs_ihf_poweroff);

int d2083_audio_hs_ihf_enable_bypass(bool en)
{
	dlg_err("%s(%s): NOT SUPPORTED.\n", __func__, en ? "on" : "off");
	return -EINVAL;
}
EXPORT_SYMBOL(d2083_audio_hs_ihf_enable_bypass);

int d2083_audio_hs_ihf_set_gain(enum d2083_sp_vol_val ihfgain_val)
{
	u8 regval;

	dlg_info("[%s]-ihfgain_val[0x%x]\n", __func__, ihfgain_val);
	
	if(0 > ihfgain_val || D2083_SPVOL_MUTE < ihfgain_val) {
		dlg_info("[%s]- Invalid gain, so set to 4dB \n", __func__);
		ihfgain_val = D2083_SPVOL_0DB;				
	}

	regval = audio_read(D2083_SP_CTRL_REG);
	if (ihfgain_val == D2083_SPVOL_MUTE) 
	{
		regval |= D2083_SP_MUTE;
	} 
	else 
	{
		regval &= ~(D2083_SP_VOL | D2083_SP_MUTE);
		regval |= (ihfgain_val << D2083_SP_VOL_SHIFT) & D2083_SP_VOL;			
	}
	return  audio_write(D2083_SP_CTRL_REG,regval);
}
EXPORT_SYMBOL(d2083_audio_hs_ihf_set_gain);

int d2083_audio_set_mixer_input(enum d2083_audio_output_sel path_sel, enum d2083_audio_input_val input_val)
{
	int reg;
	u8 regval;

	if(path_sel == D2083_OUT_HPL)
		reg = D2083_MXHPL_CTRL_REG;
	else if(path_sel == D2083_OUT_HPR)
		reg = D2083_MXHPR_CTRL_REG;
	else if(path_sel == D2083_OUT_SPKR)
		reg = D2083_MXSP_CTRL_REG;
	else
		return -EINVAL;

	regval = audio_read(reg) & ~D2083_MX_SEL;
	regval |= (input_val << D2083_MX_SEL_SHIFT) & D2083_MX_SEL;

	return audio_write(reg,regval);
}
EXPORT_SYMBOL(d2083_audio_set_mixer_input);

int d2083_audio_set_input_mode(enum d2083_input_path_sel inpath_sel, enum d2083_input_mode_val mode_val)
{
	int reg;
	u8 regval;

	if (inpath_sel == D2083_INPUTA)
		reg = D2083_PREAMP_A_CTRL2_REG;
	else if (inpath_sel == D2083_INPUTB)
		reg = D2083_PREAMP_B_CTRL2_REG;
	else
		return -EINVAL;

	regval = audio_read(reg) & ~D2083_PREAMP_CFG;
	regval |= mode_val & D2083_PREAMP_CFG;

	return audio_write(reg,regval);
}
EXPORT_SYMBOL(d2083_audio_set_input_mode);

int d2083_audio_set_input_preamp_gain(enum d2083_input_path_sel inpath_sel, enum d2083_preamp_gain_val pagain_val)
{
	int reg;
	u8 regval;

	if (inpath_sel == D2083_INPUTA)
		reg = D2083_PREAMP_A_CTRL1_REG;
	else if (inpath_sel == D2083_INPUTB)
		reg = D2083_PREAMP_B_CTRL1_REG;
	else
		return -EINVAL;

	regval = audio_read(reg) & ~D2083_PREAMP_VOL;
	regval |= (pagain_val << D2083_PREAMP_VOL_SHIFT) & D2083_PREAMP_VOL;

	dlg_info("[%s]-addr[0x%x] ihfgain_val[0x%x]\n", __func__, reg,regval);

	return audio_write(reg,regval);
}
EXPORT_SYMBOL(d2083_audio_set_input_preamp_gain);

int d2083_audio_hs_preamp_gain(enum d2083_preamp_gain_val hsgain_val)
{
	u8 regval = 0;
	int ret = 0;

	dlg_info("d2083_audio_hs_preamp_gain hs_pre_gain=0x%x \n",hsgain_val);
	//if(0 <  hsgain_val && D2083_PREAMP_GAIN_MUTE > hsgain_val ) 
		//dlg_audio->hs_pre_gain = hsgain_val;

	if(0 >  hsgain_val || D2083_PREAMP_GAIN_MUTE < hsgain_val ) {
		dlg_info("[%s]- Invalid preamp gain, so set to 0dB \n", __func__);
		hsgain_val = D2083_PREAMP_GAIN_0DB;				
	}

	if(dlg_audio->HSenabled==true)
	{
#ifdef USE_AUDIO_DIFFERENTIAL
		if(!dlg_audio->IHFenabled) 
		{
			regval = audio_read(D2083_PREAMP_A_CTRL1_REG) & ~D2083_PREAMP_VOL;
			regval |= (hsgain_val << D2083_PREAMP_VOL_SHIFT) & D2083_PREAMP_VOL;
			ret = audio_write(D2083_PREAMP_A_CTRL1_REG,regval);
		}
#endif		
		regval = audio_read(D2083_PREAMP_B_CTRL1_REG) & ~D2083_PREAMP_VOL;
		regval |= (hsgain_val << D2083_PREAMP_VOL_SHIFT) & D2083_PREAMP_VOL;
		ret = audio_write(D2083_PREAMP_B_CTRL1_REG,regval);
	}
	dlg_audio->hs_pre_gain = hsgain_val;

	return ret;
}
EXPORT_SYMBOL(d2083_audio_hs_preamp_gain);

int d2083_audio_ihf_preamp_gain(enum d2083_preamp_gain_val ihfgain_val)
{
	u8 regval;

	dlg_info("d2083_audio_ihf_preamp_gain gain=%d \n", ihfgain_val);

	if(0 >  ihfgain_val || D2083_PREAMP_GAIN_MUTE < ihfgain_val ) {
		dlg_info("[%s]- Invalid preamp gain, so set to 0dB \n", __func__);
		ihfgain_val = D2083_PREAMP_GAIN_0DB;				
	}

	regval = audio_read(D2083_PREAMP_A_CTRL1_REG) & ~D2083_PREAMP_VOL;
	regval |= (ihfgain_val << D2083_PREAMP_VOL_SHIFT) & D2083_PREAMP_VOL;

	return audio_write(D2083_PREAMP_A_CTRL1_REG,regval);

}
EXPORT_SYMBOL(d2083_audio_ihf_preamp_gain);

void extern_pre_start_stop_playback(u8 startstop)
{
	is_playback_stop = startstop;
}
EXPORT_SYMBOL(extern_pre_start_stop_playback);

int d2083_audio_multicast(u8 flag)
{
	u8 regval;
	int ret = 0;
	
	dlg_info("%s = %d\n",__func__, flag);
	
	switch(flag)
	{
		case DLG_REMOVE_IHF:
		if(!is_playback_stop) {
			ret = audio_write(D2083_MXHPL_CTRL_REG,0x07);
			regval = audio_read(D2083_PREAMP_B_CTRL1_REG);
			ret = audio_write(D2083_PREAMP_A_CTRL1_REG,regval);
		}
		ret |= d2083_audio_hs_ihf_poweroff();
		break;

		case DLG_REMOVE_HS:
		ret = d2083_audio_hs_poweron1(0);
		break;
		
		case DLG_ADD_IHF:
		ret = audio_write(D2083_MXHPL_CTRL_REG,0x19);
		ret |= d2083_audio_hs_ihf_poweron1();
		break;
		
		case DLG_ADD_HS: 
		ret = d2083_audio_hs_poweron1(1);
		break;

		default:
		break;
	}
	
	return ret;
}

EXPORT_SYMBOL(d2083_audio_multicast);

int d2083_audio_enable_zcd(int enable)
{
	int ret = 0;

	dlg_info("d2083_audio_enable_zcd =%d \n",enable);

	ret |= audio_set_bits(D2083_PREAMP_A_CTRL1_REG, D2083_PREAMP_ZC_EN);
	ret |= audio_set_bits(D2083_PREAMP_B_CTRL1_REG, D2083_PREAMP_ZC_EN);
	ret |= audio_set_bits(D2083_HP_L_CTRL_REG, D2083_HP_AMP_ZC_EN);
	ret |= audio_set_bits(D2083_HP_R_CTRL_REG, D2083_HP_AMP_ZC_EN);
	ret |= audio_set_bits(D2083_SP_NON_CLIP_ZC_REG, D2083_SP_ZC_EN);

	return ret;
}
EXPORT_SYMBOL(d2083_audio_enable_zcd);

int d2083_audio_enable_vol_slew(int enable)
{
	dlg_err("%s(%s): NOT SUPPORTED.\n", __func__, enable ? "on" : "off");
	return -EINVAL;
}
EXPORT_SYMBOL(d2083_audio_enable_vol_slew);

int d2083_set_hs_noise_gate(u16 regval)		
{
	int ret = 0;
#ifdef NOISE_GATE_USE
	dlg_info("d2083_set_hs_noise_gate regval=%d \n",regval);
	if(dlg_audio->HSenabled==true) {

		ret |= audio_write(D2083_HP_NG1_REG, (u8)(regval & 0x00FF));
		ret |= audio_write(D2083_HP_NG2_REG, (u8)((regval >> 8) & 0x00FF));
	}

	dlg_set_ng1 = (u8)(regval & 0x00FF);
	dlg_set_ng2 = (u8)((regval >> 8) & 0x00FF);
#endif
	return ret;
}
EXPORT_SYMBOL(d2083_set_hs_noise_gate);

int d2083_set_ihf_noise_gate(u16 regval)
{
	int ret = 0;
#ifdef NOISE_GATE_USE
	dlg_info("d2083_set_ihf_noise_gate regval=%d \n",regval);

	ret |= audio_write(D2083_SP_NG1_REG, (u8)(regval & 0x00FF));
	ret |= audio_write(D2083_SP_NG2_REG, (u8)((regval >> 8) & 0x00FF));
#endif
	return ret;
}
EXPORT_SYMBOL(d2083_set_ihf_noise_gate);

int d2083_set_ihf_none_clip(u16 regval)
{
	int ret = 0;

	dlg_info("d2083_set_ihf_none_clip regval=%d \n",regval);

	ret |= audio_write(D2083_SP_NON_CLIP_ZC_REG, (u8)(regval & 0x00FF));
	ret |= audio_write(D2083_SP_NON_CLIP_REG, (u8)((regval >> 8) & 0x00FF));

	return ret;
}
EXPORT_SYMBOL(d2083_set_ihf_none_clip);

int d2083_set_ihf_pwr(u8 regval)
{
	return audio_write(D2083_SP_PWR_REG, regval);
}
EXPORT_SYMBOL(d2083_set_ihf_pwr);

int d2083_sp_set_hi_impedance(u8 set_last_bit)
{
	#define cfg2_mask_bit	0x01
	
	return audio_set_bits(D2083_SP_CFG2_REG, cfg2_mask_bit&set_last_bit);
}
EXPORT_SYMBOL(d2083_sp_set_hi_impedance);

static void d2083_audio_timer(unsigned long data)
{
	schedule_work(&dlg_audio->work);
}

static void d2083_audio_worker(struct work_struct *work)
{
	if(dlg_audio->AudioStart==1) //hs
	{
		d2083_audio_hs_poweron(1);
	}
	else if(dlg_audio->AudioStart==2) //ihf
	{
		d2083_audio_hs_ihf_poweron1();
	}

	dlg_audio->AudioStart=3;
	
	return; 
}
static int d2083_audio_probe(struct platform_device *pdev)
{
	int ret = 0;

	d2083 = platform_get_drvdata(pdev);
	if(!d2083)
		return -EINVAL;

	dev_info(d2083->dev, "Starting audio\n");
	dlg_audio = &d2083->audio;

	dlg_audio->HSenabled = false;
	dlg_audio->IHFenabled = false;
	dlg_audio->AudioStart= 3;

	// Audio LDO & LDO1 mcontrol
	audio_write(D2083_LDO1_MCTL_REG, 0x54);
	audio_write(D2083_LDO_AUD_MCTL_REG, 0x54); 

	audio_write(D2083_PREAMP_A_CTRL1_REG, 0x35);
	audio_write(D2083_PREAMP_B_CTRL1_REG, 0x35);
	audio_write(D2083_PREAMP_A_CTRL2_REG,0x03); 
	audio_write(D2083_PREAMP_B_CTRL2_REG,0x03); 
	
	audio_write(D2083_CP_CTRL_REG, 0xC9);

	// HP Mixer controll
	audio_write(D2083_MXHPL_CTRL_REG,0x07);
	audio_write(D2083_MXHPR_CTRL_REG,0x19); 

	// SPK Mixer controll, A
	audio_write(D2083_MXSP_CTRL_REG,0x07);
	
	//d2083_sp_set_hi_impedance(1);

	INIT_WORK(&dlg_audio->work,d2083_audio_worker);
	init_timer(&dlg_audio->timer);
	dlg_audio->timer.function = d2083_audio_timer;
	dlg_audio->timer.data = (unsigned long) d2083;

	dev_info(d2083->dev, "\nAudio started.\n");

	return ret;
}

static int __devexit d2083_audio_remove(struct platform_device *pdev)
{
	struct d2083_audio *dlg_audio = NULL;

	d2083 = platform_get_drvdata(pdev);
	if(!d2083)
		return -EINVAL;
	dlg_audio = &d2083->audio;

	d2083 = NULL;

	return 0;
}

#ifdef CONFIG_PM
static int d2083_audio_suspend(struct device *dev)
{
	//u8 regval = 0;

	if( dlg_audio->HSenabled ==true || dlg_audio->IHFenabled == true)
		return 0;

	dev_info(d2083->dev,"d2083_audio_suspend \n");
	audio_write(D2083_LDO1_MCTL_REG, 0x00);
	audio_write(D2083_LDO_AUD_MCTL_REG, 0x00); 

	audio_write(D2083_HP_L_CTRL_REG,0x20); 
	audio_write(D2083_HP_R_CTRL_REG,0x20); 
	dlg_audio_sleep = true;
	return 0;
}

static int d2083_audio_resume(struct device *dev)
{
	
	if( dlg_audio->HSenabled ==true || dlg_audio->IHFenabled == true)
		return 0;
		
	dev_info(d2083->dev,"d2083_audio_resume \n");
	
	audio_write(D2083_PREAMP_A_CTRL1_REG,0x35);
	audio_write(D2083_PREAMP_B_CTRL1_REG,0x35);
	audio_write(D2083_CP_CTRL_REG,0xC9);
	audio_write(D2083_HP_L_CTRL_REG,0x70); 
	audio_write(D2083_HP_R_CTRL_REG,0x70); 
	
	audio_write(D2083_LDO1_MCTL_REG, 0x54);
	audio_write(D2083_LDO_AUD_MCTL_REG, 0x54); 
	msleep(30);
	return 0;
}
#else
#define d2083_audio_suspend NULL
#define d2083_audio_resume NULL
#endif

static struct dev_pm_ops d2083_audio_pm_ops = {
	.suspend = d2083_audio_suspend,
	.resume = d2083_audio_resume,
};


static struct platform_driver d2083_audio_driver = {
	.probe = d2083_audio_probe,
	.remove = __devexit_p(d2083_audio_remove),
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.pm = &d2083_audio_pm_ops,
	},
};


static int __init d2083_audio_init(void)
{
	return platform_driver_register(&d2083_audio_driver);
}
late_initcall(d2083_audio_init);

static void __exit d2083_audio_exit(void)
{
	platform_driver_unregister(&d2083_audio_driver);
}
module_exit(d2083_audio_exit);

MODULE_DESCRIPTION("D2083 Audio amplifier driver");
MODULE_AUTHOR("Roy Im <Roy.Im@diasemi.com>");
MODULE_LICENSE("GPL v2");

