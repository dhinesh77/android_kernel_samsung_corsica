/*
 * Driver for AuthenTec fingerprint sensor
 *
 * Copyright (C) 2011 AuthenTec, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __AUTH_FP_DEFS_H
#define __AUTH_FP_DEFS_H


/*****************
 * TrOperationId *
 *****************
 *
 * This enumerated type defines the operation performed by the IOCTLs. There is
 * not necessarily a 1-to-1 mapping between an item in this list and existing
 * IOCTLs.
 */
enum tr_operation_id {
	TROPID_CONNECT = 1,
	TROPID_DISCONNECT,
	TROPID_RESET,
	TROPID_CHECK_PROTOCOL,
	TROPID_GET_DEVICE_INFO,
	TROPID_GET_DEVICE_POWER,
	TROPID_SET_DEVICE_POWER,
	TROPID_SETUP_TRANSFER,
	TROPID_CANCEL_TRANSFER,
	TROPID_STOP_TRANSFER,
	TROPID_SET_READ_TIMEOUT,
	TROPID_SEND_MOUSE_EVENT,
	TROPID_SEND_KEY_EVENT
};

/* Interface Types */
#define TR_IFACE_UNDEF 0x00000000 /* Undefined   */
#define TR_IFACE_SPI_S 0x00000001 /* SPI Slave   */
#define TR_IFACE_SPI_M 0x00000002 /* SPI Master  */
#define TR_IFACE_MCBSP 0x00000004 /* McBSP       */
#define TR_IFACE_I2C   0x00000008 /* I2C         */
#define TR_IFACE_USB   0x00000010 /* USB         */



/****************************************************************************
*                                                                           *
*                            INTERFACE V2                                   *
*                                                                           *
*****************************************************************************/


#define MAX_ABORT_SEQ_LEN_V1 32
#define MAX_ABORT_SEQ_LEN_V2 64

struct tr_init_params_v1 {
	__u32 version;
	__u32 struct_size;
	__u32 interrupt_polarity;
	__u32 max_data_len;
	__u32 abort_sequence_len;
	__u8  abort_sequence[MAX_ABORT_SEQ_LEN_V1];
};

struct tr_init_params_v2 {
	__u32 version;
	__u32 struct_size;
	__u32 interrupt_polarity;
	__u32 max_data_len;
	__u32 abort_sequence_len;
	__u8  abort_sequence[MAX_ABORT_SEQ_LEN_V2];
};

struct driver_info_v1 {
	__u32 version;
	__u32 struct_size;
	__u32 driver_version;
	__u32 interface_type;
	__u32 interface_speed;
	__u32 support_sensor_poweroff;
};

/* Values of TrMouseEvent flags*/
#define TR_MOUSE_MOVE       0x00000001
#define TR_MOUSE_LEFTDOWN   0x00000002
#define TR_MOUSE_LEFTUP     0x00000004
#define TR_MOUSE_RIGHTDOWN  0x00000008
#define TR_MOUSE_RIGHTUP    0x00000010

/* Values of TrKeyEvent key events */
#define TR_KEY_EVENT_DOWN   0x01
#define TR_KEY_EVENT_UP     0x02
#define TR_KEY_EVENT_PRESS  0x03

/* Values of TrKeyEvent key codes */
#define TR_KEY_CODE_ENTER   0x1c
#define TR_KEY_CODE_LEFT    0x69
#define TR_KEY_CODE_RIGHT   0x6a
#define TR_KEY_CODE_UP      0x67
#define TR_KEY_CODE_DOWN    0x6c

struct tr_mouse_event {
	__u32  flags;         /* TR_MOUSE_* values should be put here */
	__s32  dx;
	__s32  dy;
};

struct tr_key_event {
	__u32 key_event;      /* TR_KEY_EVENT_* values should be put here */
	__u32 key_code;       /* TR_KEY_CODE_*  values should be put here */
} ;


/* Read Policy values. */
#define RP_DO_NOT_READ                  0
#define RP_READ_ON_INT                  1
#define RP_READ_ON_REQ                  2

/* Interrupt control flags */
#define TR_FLAG_RETURN_INTERRUPT_PACKET 0x1

/* Power Modes. */
#define POWER_MODE_OFF                  0
#define POWER_MODE_SLEEP                1
#define POWER_MODE_STANDBY              2
#define POWER_MODE_ACTIVE               3



/******************************************************************************
 * Notes:
 *
 * IN:    Parameter used to send information from the transport bridge to the
 *        transport driver.
 *
 * OUT:   Parameter used to receive information from the transport driver.
 *
 * INOUT: Parameter used to send information to the transport driver an to
 *        receive a response back from the transport driver.
 *****************************************************************************/

/******* Header for all structures **********/

struct tr_ioctl_header {
	/* INOUT: Total size of the structure including header.
	* On input it contains the size of the input buffer.
	* On output it contains the size of the output buffer.
	*/
	__u32 size;

	/*IN: ID of the desired operation. Needed for TransportIOCL
	*or on systems where everything is tunneled through a
	*single IOCTL. See TrOperationId.
	*/
	__u32 operation_id;

	/*OUT: Return code of the IOCTL.*/
	__s32  ret_code;
} ;


/******* Transport Connect/Disconnect **********************/

struct tr_connect_ioctl_buffer_v1 {
	struct tr_ioctl_header  header;  /* INOUT:   Header.*/
	__u32  protocol_id;        /* IN:   Selected protocol ID.*/
	struct tr_init_params_v1 params;  /*IN: Initialization parameters.*/
} ;

struct tr_connect_ioctl_buffer_v2 {
	struct tr_ioctl_header  header;  /* INOUT:   Header.*/
	__u32  protocol_id;        /* IN:   Selected protocol ID.*/
	struct tr_init_params_v2 params;  /*IN: Initialization parameters.*/
} ;

/******* Transport Reset *********************/

struct tr_reset_ioctl_buffer {
	struct tr_ioctl_header header;  /* INOUT: header.    */
};

/******* Transport GetSupportedDrivers *********************/

struct tr_check_protocol_ioctl_buffer {
	struct tr_ioctl_header header;   /* INOUT: header.*/

	/* INOUT: On input : Protocol to be checked for support.
	* On output: Same protocol ID if supported, 0 if not supported.
	*/
	__u32 protocol_id;
} ;


/******* Transport GetDriverInfo *********************/

struct tr_device_info_ioctl_buf {
	/* INOUT: Header.*/
	struct tr_ioctl_header header;

	/*IN: requested driver info structure version.*/
	__u32         requested_version;

	/*INOUT: size of the following buffer on input.*/
	__u32         buffer_size;

	/*size of the returned structure or expected buffer size on output
	*OUT: buffer to receive the structure.
	*(size = uiBufferSize)
	*/
	__u8          buffer[1];

} ;


/******* Transport SetDriverPower/GetDriverPower *********************/

struct tr_device_power_ioctl_buffer {
	/* INOUT: Header.*/
	struct tr_ioctl_header header;

	/* INOUT: Input power mode for SetDriverPower.*/
	/* Output power mode for GetDriver Power. */
	__u32          power_mode;

} ;

/******* Transport SetupTransfer *********************/

struct tr_setup_transfer_ioctl_buffer {
	struct tr_ioctl_header header;            /* INOUT: Header.        */
	__u32          packet_len;         /*    IN: Packet length. */
	__u32          nshots;             /*    IN: NShots.        */
	__u32          keep_on;            /*    IN: Keep on bit.   */
	__u32          read_policy;        /*    IN: Read policy.   */
	__u32          option_flags;       /*    IN: Option flags.  */
} ;

/******* Transport CancelTransfer *********************/

struct tr_cancel_ioctl_buffer {
	struct tr_ioctl_header header;                /* INOUT: Header.    */
};

/******* Transport SetTimeout *********************/

struct tr_timeout_ioctl_buffer {
	struct tr_ioctl_header header;    /* INOUT: Header.         */
	__u32 timeout;                         /*    IN: Timeout value.  */
} ;

/******* Transport Ioctl *********************/

struct tr_ext_ioctl_buffer {
	/* INOUT: Header.*/
	struct tr_ioctl_header header;

	/* IN: Timeout value.*/
	__u32 timeout;

	/* IN: Buffer length. Should be the maximum between */
	__u32 buf_len;

	/* the input and output buffer lengths.*/
	/* INOUT: As input:  Input data length.*/
	/* As output: Received bytes.*/
	__u32 data_len;

	/* INOUT: As input:  Contains the input data.*/
	/* As output: Contains the output data.*/
	__u8  buffer[1];

} ;


#define FP_IOCTL_BASE ('x')

#define FP_IOCTL_TRANSPORT_CONNECT     \
	_IOWR(FP_IOCTL_BASE,  0, void *)

#define FP_IOCTL_TRANSPORT_DISCONNECT    \
	_IOWR(FP_IOCTL_BASE,  1, void *)

#define FP_IOCTL_TRANSPORT_RESET     \
	_IOWR(FP_IOCTL_BASE,  2, void *)

#define FP_IOCTL_CHECK_PROTOCOL      \
	_IOWR(FP_IOCTL_BASE,  3, void *)

#define FP_IOCTL_GET_DEVINFO             \
	_IOWR(FP_IOCTL_BASE,  4, void *)

#define FP_IOCTL_GET_DEVICE_POWER   \
	_IOWR(FP_IOCTL_BASE,  5, void *)

#define FP_IOCTL_SET_DEVICE_POWER    \
	_IOWR(FP_IOCTL_BASE,  6, void *)

#define FP_IOCTL_SETUP_TRANSFER         \
	_IOWR(FP_IOCTL_BASE,  7, void *)

#define FP_IOCTL_CANCEL_TRANSFER       \
	_IOWR(FP_IOCTL_BASE,  8, void *)

#define FP_IOCTL_SET_TIMEOUT              \
	_IOWR(FP_IOCTL_BASE, 10, void *)

#define FP_IOCTL_TRANSPORT_EXT          \
	_IOWR(FP_IOCTL_BASE, 11, void *)


#endif
