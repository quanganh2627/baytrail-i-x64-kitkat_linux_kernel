#ifndef __LPAUDIO_H__
#define __LPAUDIO_H__

#if defined(LPVM)
#include "lpvm.h"

#define LPAUDIO_PROMPT		"lpaudio_vm"

void lpaudio_init(TX_BYTE_POOL *pool);

#elif defined(LPAUDIO_LIB)
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

typedef unsigned int	u32;
typedef unsigned short	u16;
typedef unsigned char	u8;

#define LPAUDIO_PROMPT		"lpaudio_lib"

#elif defined(__KERNEL__)
#define LPAUDIO_PROMPT		"lpaudio_drv"
#define lpvm_print		pr_info

extern int lpaudio_enabled;
extern struct dsp_audio_device *p_dsp_audio_dev;
extern int dsp_start_audio_hwafe(void);
extern int dsp_stop_audio_hwafe(void);
extern int get_dsp_pcm_rate(unsigned int rate);
extern void setup_pcm_play_path(void);
#endif

#define OFFSET_SM_AUDIO_BUFFER_1_DL	594
#define OFFSET_SM_AUDIO_BUFFER_SIZE_UL	1074
#define OFFSET_SM_AUDIO_BUFFER_UL	114

#define DMA_PADDING_SIZE		(8*4096)
#define DMA_BURST_SIZE			256
#define DMA_BLOCK_SIZE			(960)
#define DMA_BLOCK_NUM			(220)
#define DMA_TOTAL_SIZE			\
			(DMA_BLOCK_SIZE * DMA_BLOCK_NUM + DMA_PADDING_SIZE)
#define DMA_CTRL_ADDR			0xe0100000

#define LPAUDIO_MMAP_DMAC       0
#define LPAUDIO_MMAP_DMA_MEM    1
#define LPAUDIO_MMAP_WORK_BUF   2

#define LPAUDIO_IOCTRL_DMAC		_IOWR('A', 1, u32)
#define LPAUDIO_IOCTRL_DMAMEM		_IOWR('A', 2, u32)
#define LPAUDIO_IOCTRL_WORKBUF		_IOWR('A', 3, u32)
#define LPAUDIO_IOCTRL_DMASIZE		_IOWR('A', 4, u32)
#define LPAUDIO_IOCTRL_DMAPADDING	_IOWR('A', 5, u32)
#define LPAUDIO_IOCTRL_WBUFSIZE		_IOWR('A', 6, u32)
#define LPAUDIO_IOCTRL_DSP_DL		_IOWR('A', 7, u32)
#define LPAUDIO_IOCTRL_DSP_UL		_IOWR('A', 8, u32)
#define LPAUDIO_IOCTRL_DSP_PLAY		_IOWR('A', 9, u32)
#define LPAUDIO_IOCTRL_START		_IOWR('A', 21, u32)
#define LPAUDIO_IOCTRL_STOP		_IOWR('A', 22, u32)

#define WORK_BUF_SIZE			(128 * 4096)
#define LPAUDIO_LPVM_WBUF_SIZE		(400 * 1024)
#define LPAUDIO_INPUT_PADDING_SIZE	(4096)
#define LPAUDIO_INPUT_BUF_SIZE		(32*1024)
#define LPAUDIO_INPUT_BLOCK_MAX		4096
#define LPAUDIO_INPUT_TOTAL_SIZE	\
		((LPAUDIO_INPUT_BUF_SIZE + LPAUDIO_INPUT_PADDING_SIZE) * 2)
#define LPAUDIO_WORKING_BUF_SIZE	(512*1024)

#define LPAUDIO_CMD_DEBUG		0x00
#define LPAUDIO_CMD_INIT		0x01
#define LPAUDIO_CMD_START		0x02
#define LPAUDIO_CMD_SETUP		0x03
#define LPAUDIO_CMD_PAUSE		0x04
#define LPAUDIO_CMD_STOP		0x05
#define LPAUDIO_CMD_DATA		0x06
#define LPAUDIO_CMD_PLAY		0x07

#define LPAUDIO_CMD_ACK			0x81

#define LPAUDIO_FMT_PCM_16_48000	0x0
#define LPAUDIO_FMT_MP3			0x1

#define LPAUDIO_TYPE_WAV		0
#define LPAUDIO_TYPE_MP3		1

#define lpaudio_err(x...)	lpvm_print("["LPAUDIO_PROMPT"] Err: " x)
#define lpaudio_info(x...)	lpvm_print("["LPAUDIO_PROMPT"] Inf: " x)
#define lpaudio_dbg(x...)						\
	do {								\
		if (lpaudio_debug_enable)				\
			lpvm_print("["LPAUDIO_PROMPT"] Dbg: " x);	\
	} while (0)

typedef int(*LPAUDIO_PLAYER)(u8 *buf, int len);

struct lpaudio_ipc_head {
	u8			cmd;
	u8			id;
	u16			payload_len;
};

struct lpaudio_setup_param {
	/* output buf setup */
	u32	*dma_ptr;
	u8	*output_start;
	u8	*output_end;
	u32	output_len;
	u32	output_padding_len;
	/* input buf setup */
	u8	*input_buf[2];
	u32	input_padding_len;
	/* player setup */
	u8	*work_buf;
	u32	work_buf_len;
	u32	type;
	u32	rate;
};

struct lpaudio_play_param {
	u32	play_seq;
	u32	len;
};

struct lpaudio_ack_param {
	u8	id;
	u8	is_log;
	union {
		int	ret;
		u8	log[80];
	};
};

struct lpaudio_ipc_t {
#define	LPAUDIO_IPC_MAXSIZE		4096
#define	LPAUDIO_PAYLOAD_SIZE		(4096 - sizeof(struct lpaudio_ipc_head))
	struct lpaudio_ipc_head head;
	union {
		struct lpaudio_setup_param setup;
		struct lpaudio_play_param play;
		u8 payload[LPAUDIO_PAYLOAD_SIZE];
		int return_value;
	};
};

extern int lpaudio_debug_enable;
extern u32 lpaudio_rate;

#endif /*__LPAUDIO_H__*/
