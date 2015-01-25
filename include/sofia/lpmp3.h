#ifndef __LPMP3_H__
#define __LPMP3_H__

#ifdef __KERNEL__
#define LPMP3_PROMPT		"lpmp3_drv"
#define lpmp3_print		pr_info

int xgold_lpmp3_mode(void);

extern struct dma_async_tx_descriptor *
	       (*lpmp3_dma_setup)(struct dma_chan *dmach);
extern void (*lpmp3_dma_release)(struct dma_chan *dmach);
extern void (*lpmp3_trigger)(int start);
#endif

extern int lpmp3_debug_enable;
#define lpmp3_err(x...)		lpmp3_print("["LPMP3_PROMPT"] Err: " x)
#define lpmp3_info(x...)	lpmp3_print("["LPMP3_PROMPT"] Inf: " x)
#define lpmp3_dbg(x...) \
	do { \
		if (lpmp3_debug_enable) \
			lpmp3_print("["LPMP3_PROMPT"] Dbg: " x); \
	} while (0)

#ifndef PAGE_SIZE
#define PAGE_SIZE			(4096)
#endif
#define LPMP3_OUTPUT_MAX		(8*PAGE_SIZE)
#define LPMP3_DATA_BLOCK		(8*PAGE_SIZE)
#define LPMP3_DATA_SIZE			(2048*PAGE_SIZE)
#define LPMP3_DATA_PADDING		PAGE_SIZE

#define LPMP3_DMA_PADDING_SIZE		LPMP3_OUTPUT_MAX
#define LPMP3_DMA_BURST_SIZE		256
#define LPMP3_DMA_BLOCK_SIZE		960
#define LPMP3_DMA_BLOCK_NUM		220
#define LPMP3_DMA_TOTAL_SIZE		\
			(LPMP3_DMA_BLOCK_SIZE * LPMP3_DMA_BLOCK_NUM \
			 + LPMP3_DMA_PADDING_SIZE)

#define LPMP3_MMAP_DMAC			0
#define LPMP3_MMAP_DMA_MEM		1
#define LPMP3_MMAP_CTRL			2
#define LPMP3_MMAP_DATA			3

#define LPMP3_IOCTRL_START		_IOWR('A', 10, u32)
#define LPMP3_IOCTRL_STOP		_IOWR('A', 11, u32)
#define LPMP3_IOCTRL_ENABLE		_IOWR('A', 12, u32)
#define LPMP3_IOCTRL_DISABLE		_IOWR('A', 13, u32)

#define LPMP3_CMD_DEBUG			0x0
#define LPMP3_CMD_OPEN			0x1
#define LPMP3_CMD_CLOSE			0x2
#define LPMP3_CMD_VOL			0x3
#define LPMP3_CMD_WRITE			0x4
#define LPMP3_CMD_PAUSE			0x5
#define LPMP3_CMD_RESUME		0x6
#define LPMP3_CMD_DRAIN			0x7
#define LPMP3_CMD_FLUSH			0x8

#define LPMP3_ACK_CMD			0x70
#define LPMP3_ACK_CMD_WRITE_READY	0x71
#define LPMP3_ACK_CMD_DRAIN_READY	0x72
#define LPMP3_ACK_CMD_CLOSE_READY	0x73

#define LPMP3_FMT_MP3			0x1
#define LPMP3_FMT_AAC			0x2

struct lpmp3_ctrl_t {
#define LPMP3_CTRL_STOP			0
#define LPMP3_CTRL_PLAY			1
#define LPMP3_CTRL_PAUSE		2
	/* update by app */
	int			play_ctrl;
	u32			vol;
	u32			rate;
	u32			format;
	u32			data_size;

	/* update by vm */
	int			play_status;
	int			frame;
	int			done_seq;
	int			play_seq;

	/* update by driver */
	u32			ctrl_addr;
	int			open;
	u32			*dma_ptr;
	u32			data_pages[LPMP3_DATA_SIZE/PAGE_SIZE];
	int			data_page_num;
	u32			output_addr;
	u32			output_len;
	u32			output_padding_len;
};

struct lpmp3_ipc_t {
	int			cmd;
	union {
		int		ret;
		u32		data;
	};
};

#endif /*__LPMP3_H__*/
