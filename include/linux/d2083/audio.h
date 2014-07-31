/*
 * audio.h: Audio amplifier driver for D2083
 *
 * Copyright(c) 2012 Dialog Semiconductor Ltd.
 * Author: Mariusz Wojtasik <mariusz.wojtasik@diasemi.com>
 *
 * This program is free software; you can redistribute  it and/or modify
 * it under  the terms of  the GNU General  Public License as published by
 * the Free Software Foundation;  either version 2 of the  License, or
 * (at your option) any later version.
 *
 */

#ifndef __LINUX_D2083_AUDIO_H
#define __LINUX_D2083_AUDIO_H

#include <linux/platform_device.h>

#define D2083_MAX_GAIN_TABLE 24
#define USE_AUDIO_DIFFERENTIAL

enum d2083_audio_input_val {
	D2083_IN_NONE   = 0x00,
	D2083_IN_A1     = 0x01,
	D2083_IN_A2     = 0x02,
	D2083_IN_B1     = 0x04,
	D2083_IN_B2     = 0x08
};

enum d2083_input_mode_val {
	D2083_IM_TWO_SINGLE_ENDED   = 0x00,
	D2083_IM_ONE_SINGLE_ENDED_1 = 0x01,
	D2083_IM_ONE_SINGLE_ENDED_2 = 0x02,
	D2083_IM_FULLY_DIFFERENTIAL = 0x03
};
enum d2083_input_path_sel {
	D2083_INPUTA,
	D2083_INPUTB
};

enum d2083_preamp_gain_val {
	D2083_PREAMP_GAIN_NEG_6DB   = 0x07,
	D2083_PREAMP_GAIN_NEG_5DB   = 0x08,
	D2083_PREAMP_GAIN_NEG_4DB   = 0x09,
	D2083_PREAMP_GAIN_NEG_3DB   = 0x0a,
	D2083_PREAMP_GAIN_NEG_2DB   = 0x0b,
	D2083_PREAMP_GAIN_NEG_1DB   = 0x0c,
	D2083_PREAMP_GAIN_0DB       = 0x0d,
	D2083_PREAMP_GAIN_1DB       = 0x0e,
	D2083_PREAMP_GAIN_2DB       = 0x0f,
	D2083_PREAMP_GAIN_3DB       = 0x10,
	D2083_PREAMP_GAIN_4DB       = 0x11,
	D2083_PREAMP_GAIN_5DB       = 0x12,
	D2083_PREAMP_GAIN_6DB       = 0x13,
	D2083_PREAMP_GAIN_7DB       = 0x14,
	D2083_PREAMP_GAIN_8DB       = 0x15,
	D2083_PREAMP_GAIN_9DB       = 0x16,
	D2083_PREAMP_GAIN_10DB      = 0x17,
	D2083_PREAMP_GAIN_11DB      = 0x18,
	D2083_PREAMP_GAIN_12DB      = 0x19,
	D2083_PREAMP_GAIN_13DB      = 0x1a,
	D2083_PREAMP_GAIN_14DB      = 0x1b,
	D2083_PREAMP_GAIN_15DB      = 0x1c,
	D2083_PREAMP_GAIN_16DB      = 0x1d,
	D2083_PREAMP_GAIN_17DB      = 0x1e,
	D2083_PREAMP_GAIN_18DB      = 0x1f,
	D2083_PREAMP_GAIN_MUTE
};

static inline int d2083_pagain_to_reg(enum d2083_preamp_gain_val gain_val)
{
	return (int)gain_val;
}

static inline enum d2083_preamp_gain_val d2083_reg_to_pagain(int reg)
{
	return ((reg < D2083_PREAMP_GAIN_NEG_6DB) ? D2083_PREAMP_GAIN_NEG_6DB :
			(enum d2083_preamp_gain_val)reg);
}

static inline int d2083_pagain_to_db(enum d2083_preamp_gain_val gain_val)
{
	return (int)gain_val - (int)D2083_PREAMP_GAIN_0DB;
}

static inline enum d2083_preamp_gain_val d2083_db_to_pagain(int db)
{
	return (enum d2083_preamp_gain_val)(db + (int)D2083_PREAMP_GAIN_0DB);
}

enum d2083_mixer_selector_val {
	D2083_MSEL_A1   = 0x01, /* if A is fully diff. then selects A2 as the inverting input */
	D2083_MSEL_B1   = 0x02, /* if B is fully diff. then selects B2 as the inverting input */
	D2083_MSEL_A2   = 0x04, /* if A is fully diff. then selects A1 as the non-inverting input */
	D2083_MSEL_B2   = 0x08  /* if B is fully diff. then selects B1 as the non-inverting input */
};

enum d2083_mixer_attenuation_val {
	D2083_MIX_ATT_0DB       = 0x00, /* attenuation = 0dB */
	D2083_MIX_ATT_NEG_6DB   = 0x01, /* attenuation = -6dB */
	D2083_MIX_ATT_NEG_9DB5  = 0x02, /* attenuation = -9.5dB */
	D2083_MIX_ATT_NEG_12DB  = 0x03  /* attenuation = -12dB */
};

enum d2083_hp_vol_val {
	D2083_HPVOL_NEG_57DB,   D2083_HPVOL_NEG_56DB,   D2083_HPVOL_NEG_55DB,   D2083_HPVOL_NEG_54DB,
	D2083_HPVOL_NEG_53DB,   D2083_HPVOL_NEG_52DB,   D2083_HPVOL_NEG_51DB,   D2083_HPVOL_NEG_50DB,
	D2083_HPVOL_NEG_49DB,   D2083_HPVOL_NEG_48DB,   D2083_HPVOL_NEG_47DB,   D2083_HPVOL_NEG_46DB,
	D2083_HPVOL_NEG_45DB,   D2083_HPVOL_NEG_44DB,   D2083_HPVOL_NEG_43DB,   D2083_HPVOL_NEG_42DB,
	D2083_HPVOL_NEG_41DB,   D2083_HPVOL_NEG_40DB,   D2083_HPVOL_NEG_39DB,   D2083_HPVOL_NEG_38DB,
	D2083_HPVOL_NEG_37DB,   D2083_HPVOL_NEG_36DB,   D2083_HPVOL_NEG_35DB,   D2083_HPVOL_NEG_34DB,
	D2083_HPVOL_NEG_33DB,   D2083_HPVOL_NEG_32DB,   D2083_HPVOL_NEG_31DB,   D2083_HPVOL_NEG_30DB,
	D2083_HPVOL_NEG_29DB,   D2083_HPVOL_NEG_28DB,   D2083_HPVOL_NEG_27DB,   D2083_HPVOL_NEG_26DB,
	D2083_HPVOL_NEG_25DB,   D2083_HPVOL_NEG_24DB,   D2083_HPVOL_NEG_23DB,   D2083_HPVOL_NEG_22DB,
	D2083_HPVOL_NEG_21DB,   D2083_HPVOL_NEG_20DB,   D2083_HPVOL_NEG_19DB,   D2083_HPVOL_NEG_18DB,
	D2083_HPVOL_NEG_17DB,   D2083_HPVOL_NEG_16DB,   D2083_HPVOL_NEG_15DB,   D2083_HPVOL_NEG_14DB,
	D2083_HPVOL_NEG_13DB,   D2083_HPVOL_NEG_12DB,   D2083_HPVOL_NEG_11DB,   D2083_HPVOL_NEG_10DB,
	D2083_HPVOL_NEG_9DB,    D2083_HPVOL_NEG_8DB,    D2083_HPVOL_NEG_7DB,    D2083_HPVOL_NEG_6DB,
	D2083_HPVOL_NEG_5DB,    D2083_HPVOL_NEG_4DB,    D2083_HPVOL_NEG_3DB,    D2083_HPVOL_NEG_2DB,
	D2083_HPVOL_NEG_1DB,    D2083_HPVOL_0DB,        D2083_HPVOL_1DB,        D2083_HPVOL_2DB,
	D2083_HPVOL_3DB,        D2083_HPVOL_4DB,        D2083_HPVOL_5DB,        D2083_HPVOL_6DB,
	D2083_HPVOL_MUTE
};


#define PMU_HSGAIN_NUM 	D2083_HPVOL_MUTE
#define PMU_IHFGAIN_NUM	D2083_SPVOL_MUTE


static inline int d2083_hpvol_to_reg(enum d2083_hp_vol_val vol_val)
{
	return (int)vol_val;
}

static inline enum d2083_hp_vol_val d2083_reg_to_hpvol(int reg)
{
	return (enum d2083_hp_vol_val)reg;
}

static inline int d2083_hpvol_to_db(enum d2083_hp_vol_val vol_val)
{
	return (int)vol_val - (int)D2083_HPVOL_0DB;
}

static inline enum d2083_hp_vol_val d2083_db_to_hpvol(int db)
{
	return (enum d2083_hp_vol_val)(db + (int)D2083_HPVOL_0DB);
}

enum d2083_sp_vol_val {
	D2083_SPVOL_NEG_24DB    = 0x1b, D2083_SPVOL_NEG_23DB = 0x1c,
	D2083_SPVOL_NEG_22DB    = 0x1d, D2083_SPVOL_NEG_21DB = 0x1e,
	D2083_SPVOL_NEG_20DB    = 0x1f, D2083_SPVOL_NEG_19DB = 0x20,
	D2083_SPVOL_NEG_18DB    = 0x21, D2083_SPVOL_NEG_17DB = 0x22,
	D2083_SPVOL_NEG_16DB    = 0x23, D2083_SPVOL_NEG_15DB = 0x24,
	D2083_SPVOL_NEG_14DB    = 0x25, D2083_SPVOL_NEG_13DB = 0x26,
	D2083_SPVOL_NEG_12DB    = 0x27, D2083_SPVOL_NEG_11DB = 0x28,
	D2083_SPVOL_NEG_10DB    = 0x29, D2083_SPVOL_NEG_9DB = 0x2a,
	D2083_SPVOL_NEG_8DB     = 0x2b, D2083_SPVOL_NEG_7DB = 0x2c,
	D2083_SPVOL_NEG_6DB     = 0x2d, D2083_SPVOL_NEG_5DB = 0x2e,
	D2083_SPVOL_NEG_4DB     = 0x2f, D2083_SPVOL_NEG_3DB = 0x30,
	D2083_SPVOL_NEG_2DB     = 0x31, D2083_SPVOL_NEG_1DB = 0x32,
	D2083_SPVOL_0DB         = 0x33,
	D2083_SPVOL_1DB         = 0x34, D2083_SPVOL_2DB = 0x35,
	D2083_SPVOL_3DB         = 0x36, D2083_SPVOL_4DB = 0x37,
	D2083_SPVOL_5DB         = 0x38, D2083_SPVOL_6DB = 0x39,
	D2083_SPVOL_7DB         = 0x3a, D2083_SPVOL_8DB = 0x3b,
	D2083_SPVOL_9DB         = 0x3c, D2083_SPVOL_10DB = 0x3d,
	D2083_SPVOL_11DB        = 0x3e, D2083_SPVOL_12DB = 0x3f,
	D2083_SPVOL_MUTE
};

enum d2083_multicast_type {
	DLG_NORMAL_PATH	= 0,
	DLG_REMOVE_IHF	= 1,
	DLG_REMOVE_HS 	= 2,
	DLG_ADD_IHF 	= 3,
	DLG_ADD_HS		= 4
};

static inline int d2083_spvol_to_reg(enum d2083_sp_vol_val vol_val)
{
	return (int)vol_val;
}

static inline enum d2083_sp_vol_val d2083_reg_to_spvol(int reg)
{
	return ((reg < D2083_SPVOL_NEG_24DB) ? D2083_SPVOL_NEG_24DB :
			(enum d2083_sp_vol_val)reg);
}

static inline int d2083_spvol_to_db(enum d2083_sp_vol_val vol_val)
{
	return (int)vol_val - (int)D2083_SPVOL_0DB;
}

static inline enum d2083_sp_vol_val d2083_db_to_spvol(int db)
{
	return (enum d2083_sp_vol_val)(db + (int)D2083_SPVOL_0DB);
}


enum d2083_audio_output_sel {
	D2083_OUT_NONE  = 0x00,
	D2083_OUT_HPL   = 0x01,
	D2083_OUT_HPR   = 0x02,
	D2083_OUT_HPLR  = D2083_OUT_HPL | D2083_OUT_HPR,
	D2083_OUT_SPKR  = 0x04,
};

struct d2083_audio_platform_data
{
	u8 ina_def_mode;
	u8 inb_def_mode;
	u8 ina_def_preampgain;
	u8 inb_def_preampgain;

	u8 lhs_def_mixer_in;
	u8 rhs_def_mixer_in;
	u8 ihf_def_mixer_in;

	int hs_input_path;
	int ihf_input_path;
};

struct d2083_audio {
	struct platform_device  *pdev;
	bool IHFenabled;
	bool HSenabled;
	u8 hs_pga_gain;
	u8 hs_pre_gain;
	struct timer_list timer;
	struct work_struct work;
	u8 AudioStart; //0=not started, 1=hs starting, 2=ihf starting, 3=started
};

int d2083_audio_hs_poweron(bool on);
int d2083_audio_hs_shortcircuit_enable(bool en);
int d2083_audio_hs_set_gain(enum d2083_audio_output_sel hs_path_sel, enum d2083_hp_vol_val hs_gain_val);
int d2083_audio_hs_ihf_poweron(void);
int d2083_audio_hs_ihf_poweroff(void);
int d2083_audio_hs_ihf_enable_bypass(bool en);
int d2083_audio_hs_ihf_set_gain(enum d2083_sp_vol_val ihfgain_val);
int d2083_audio_set_mixer_input(enum d2083_audio_output_sel path_sel, enum d2083_audio_input_val input_val);
int d2083_audio_set_input_mode(enum d2083_input_path_sel inpath_sel, enum d2083_input_mode_val mode_val);
int d2083_audio_set_input_preamp_gain(enum d2083_input_path_sel inpath_sel, enum d2083_preamp_gain_val pagain_val);
int d2083_audio_hs_preamp_gain(enum d2083_preamp_gain_val hsgain_val);
int d2083_audio_ihf_preamp_gain(enum d2083_preamp_gain_val ihfgain_val);
int d2083_audio_enable_zcd(int enable);
int d2083_audio_enable_vol_slew(int enable);
int d2083_set_hs_noise_gate(u16 regval);		
int d2083_set_ihf_noise_gate(u16 regval);
int d2083_set_ihf_none_clip(u16 regval);
int d2083_set_ihf_pwr(u8 regval);
int d2083_sp_set_hi_impedance(u8 set_last_bit);
int d2083_audio_multicast(u8 flag);
void extern_pre_start_stop_playback(u8 startstop);

#endif /* __LINUX_D2083_AUDIO_H */
