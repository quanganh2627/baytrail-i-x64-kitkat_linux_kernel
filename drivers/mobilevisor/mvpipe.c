/* ----------------------------------------------------------------------------
 *  Copyright (C) 2014 Intel Mobile Communications GmbH

 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License Version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *  You should have received a copy of the GNU General Public License Version 2
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.

  ---------------------------------------------------------------------------*/

#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/semaphore.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/debugfs.h>

#include <sofia/mv_gal.h>
#include <sofia/mv_ipc.h>
#include <sofia/mv_hypercalls.h>

#define DEBUG_LEVEL 1
#define DEBUG_LEVEL_ERROR 1
#define DEBUG_LEVEL_INFO  2
#define MODULE_NAME "mvpipe"

enum mbox_status {
	MBOX_DISCONNECTED,
	MBOX_CONNECTED
};

enum mvpipe_status {
	MVPIPE_OPEN,
	MVPIPE_CLOSE,
	MVPIPE_OPENING,
	MVPIPE_CLOSING
};

enum mvpipe_event_id {
	PIPE_EVENT = 0,
	RING0_EVENT,
	RING1_EVENT
};

/* writer_status */
#define WRITE_BUF_FULL 1
#define WRITE_BUF_IDLE 0

/* reader_status */
#define READ_BUF_EMPTY 1
#define READ_BUF_IDLE  0

struct mvpipe_ring {
	volatile uint32_t reader_index;
	volatile uint32_t reader_status;

	volatile uint32_t writer_index;
	volatile uint32_t writer_status;

	volatile uint32_t ring_event;
	volatile uint32_t size;

	uint32_t reserved[2];

	uint8_t buf[0];
};

inline uint32_t get_write_buf_size(struct mvpipe_ring *r)
{
	uint32_t reader_index = r->reader_index;
	if (r->writer_index < reader_index)
		return reader_index - r->writer_index - 1;

	return r->size - r->writer_index - (reader_index == 0);
}

inline uint32_t get_read_buf_size(struct mvpipe_ring *r)
{
	uint32_t writer_index = r->writer_index;
	if (r->reader_index <= writer_index)
		return writer_index - r->reader_index;

	return r->size - r->reader_index;
}

#define advance_index(index, size, count) \
	do { \
		index += count; \
		if (index == size) \
			index = 0; \
		if (index >= size) \
			BUG(); \
	} while (0)

#define is_buffer_full(r) (r->writer_index + 1 == r->reader_index || \
			   r->writer_index + 1 == r->reader_index + r->size)

#define is_buffer_empty(r) (r->writer_index  == r->reader_index)

#define set_pipe_status(dev, s) (dev->p_event->mvpipe_status[dev->writer_id] \
					= s)
#define get_pipe_status(dev)    (dev->p_event->mvpipe_status[dev->writer_id])
#define get_peer_status(dev)    (dev->p_event->mvpipe_status[!dev->writer_id])

struct mvpipe_event {
	volatile uint32_t handshake;
	volatile enum mvpipe_status mvpipe_status[2];
	uint32_t reserved;
};

enum debug_log_type {
    NONE_MIN_TYPE = 0,
	DUMP_TRACE_TYPE,
	DUMP_FTRACE_TYPE,
    NONE_MAX_TYPE
};

struct mvpipe_instance {
	char name[MAX_MBOX_INSTANCE_NAME_SIZE];
	uint32_t token;

	uint32_t open_count;
	uint32_t writer_id;

	struct semaphore open_sem;
	struct semaphore read_sem;
	struct semaphore write_sem;

	wait_queue_head_t open_wait;
	wait_queue_head_t read_wait;
	wait_queue_head_t write_wait;
	wait_queue_head_t close_wait;

	char dev_name[MAX_MBOX_INSTANCE_NAME_SIZE + 8];
	struct cdev cdev;

	enum mbox_status mbox_status;

	uint32_t ring_size;
	uint32_t pipe_event;

	/* Share mem struct */
	struct mvpipe_event *p_event;
	struct mvpipe_ring *ring[2];

    enum debug_log_type dump_type;
    uint32_t dump_maxlen;
    uint32_t is_dbg;

	int ready_to_release;
};

#define is_ring0_writer(dev) (dev->writer_id == 0)

struct mvpipe_instance *mvpipes = NULL;
uint32_t mvpipe_count;
struct class *cl_ipc;

static struct dentry *mvpipe_debug_root;
static uint32_t debug_print_lev = 0;
static uint32_t debug_print_type = 0;

static int mvpipe_pr_info(char *fname,int line,struct mvpipe_instance *dev,uint32_t type,const char *fmt, ...)
{
        struct va_format vaf;
        va_list args;
        int r;

        va_start(args, fmt);

        vaf.fmt = fmt;
        vaf.va = &args;
        if(type == 1)
        r = pr_info("[%s:%d][%s]%pV",fname,line,dev->name,&vaf);
        else if(type == 2)
        r = __trace_printk(_THIS_IP_,"[%s:%d][%s]%pV",fname,line,dev->name,&vaf);

        va_end(args);

        return r;
}

#define mvpipe_info(fmt, arg...) \
	do { \
		if ((debug_print_lev >= DEBUG_LEVEL_INFO) && (debug_print_type > 0 && debug_print_type <= 2)) \
            mvpipe_pr_info(__func__,__LINE__,dev,debug_print_type,fmt, ##arg);  \
	} while (0)

#define mvpipe_error(fmt, arg...) \
            do { \
                mvpipe_pr_info(__func__,__LINE__,dev,1,fmt, ##arg);  \
            } while (0)

static void ftrace_dump_data(char *dev_name,enum debug_log_type dump_type,u8 is_out, const void *buf,size_t len,size_t maxlen)
{
	const u8 *ptr = buf;
	int i, linelen, remaining = 0;
	unsigned char linebuf[32 * 3 + 2 + 32 + 1];
	int rowsize = 16;
	int groupsize = 1;
	bool ascii = true;
	int orig_len = len;

	if(maxlen)
	len = min(len,maxlen);

	remaining = len;
	for (i = 0; i < len; i += rowsize) {
		linelen = min(remaining, rowsize);
		remaining -= rowsize;

		hex_dump_to_buffer(ptr + i, linelen, rowsize, groupsize,
				   linebuf, sizeof(linebuf), ascii);

		if(dump_type == DUMP_FTRACE_TYPE)
		trace_printk("[%s-%d-%s] %.4x: %s\n",dev_name,orig_len,is_out == 1 ? "-->" : "<--", i, linebuf);
		else if(dump_type == DUMP_TRACE_TYPE)
		pr_info("[%s-%d-%s] %.4x: %s\n",dev_name,orig_len,is_out == 1 ? "-->" : "<--", i, linebuf);
	}

}


void mvpipe_close(struct mvpipe_instance *dev)
{
	dev->open_count = 0;

	/* reset ring */
	dev->ring[dev->writer_id]->writer_index = 0;
	dev->ring[dev->writer_id]->writer_status = WRITE_BUF_IDLE;

	dev->ring[!dev->writer_id]->reader_index = 0;
	dev->ring[!dev->writer_id]->reader_status = READ_BUF_IDLE;

	/* wake up read and write */
	wake_up_interruptible(&dev->write_wait);
	wake_up_interruptible(&dev->read_wait);
}

int mvpipe_dev_open(struct inode *inode, struct file *filp)
{
	struct mvpipe_instance *dev;
	dev = container_of(inode->i_cdev, struct mvpipe_instance, cdev);
	filp->private_data = dev;

	mvpipe_info("Open mvpipe: %s\n", dev->dev_name);

	if (down_interruptible(&dev->open_sem)) {
		mvpipe_error("down interruptible fail!\n");
		return -ERESTARTSYS;
	}

	if (dev->open_count) {
		up(&dev->open_sem);
        if(dev->is_dbg) {
            dev->open_count++;
            return 0;
        }
        else{
            mvpipe_error("Multiple open, open_count = %d!\n",
                     dev->open_count);
            return -ERESTARTSYS;
        }
	}

	dev->open_count++;
	set_pipe_status(dev, MVPIPE_OPENING);

	mvpipe_info("Wait until connected!\n");
	wait_event_interruptible(dev->open_wait,
				 dev->mbox_status == MBOX_CONNECTED);

	mvpipe_info("Wait until peer close properly!\n");
	wait_event_interruptible(dev->open_wait,
				 get_peer_status(dev) != MVPIPE_CLOSING);

	mvpipe_info("Open success!\n");
	set_pipe_status(dev, MVPIPE_OPEN);

	dev->ready_to_release = 0;

	up(&dev->open_sem);

	return 0;
}

int mvpipe_dev_release(struct inode *inode, struct file *filp)
{
	struct mvpipe_instance *dev = filp->private_data;
	mvpipe_info("Release mvpipe: %s\n", dev->dev_name);

	if (down_interruptible(&dev->open_sem))
		return -ERESTARTSYS;

    if(dev->is_dbg){
        if (dev->open_count > 1){
            dev->open_count--;
            up(&dev->open_sem);
            return 0;
        }
    }

	if (get_pipe_status(dev) == MVPIPE_CLOSE) {
		up(&dev->open_sem);
		return -ERESTARTSYS;
	}

	/* set pipe status to CLOSING */
	set_pipe_status(dev, MVPIPE_CLOSING);

	/* Close */
	mvpipe_close(dev);

	/* inform peer, if peer is OPEN */
	if (get_peer_status(dev) == MVPIPE_OPEN &&
	    dev->mbox_status == MBOX_CONNECTED)
		mv_ipc_mbox_post(dev->token, dev->pipe_event);

	/* wait until peer status is not OPEN */
	wait_event_interruptible(dev->close_wait,
				dev->ready_to_release);

	mvpipe_info("Release mvpipe successful!\n");
	up(&dev->open_sem);

	return 0;
}

ssize_t mvpipe_dev_read(struct file *filp, char __user *buf, size_t count,
			loff_t *f_pos)
{
	ssize_t retval = 0;
	struct mvpipe_instance *dev = filp->private_data;
	uint32_t buffer_size, read_size;
	struct mvpipe_ring *ring;

	if (get_pipe_status(dev) != MVPIPE_OPEN ||
	    dev->mbox_status != MBOX_CONNECTED) {
		mvpipe_error("Read before Open!");
		return -EINTR;
	}

	if (down_interruptible(&dev->read_sem))
		return -ERESTARTSYS;

	ring = dev->ring[!dev->writer_id];

	mvpipe_info("mvpipe going to read %d bytes\n", count);
	while (count != 0) {

		buffer_size = get_read_buf_size(ring);

		if (buffer_size > 0) {
			if (count <= buffer_size)
				read_size = count;
			else
				read_size = buffer_size;

			copy_to_user(buf + retval,
				     &ring->buf[ring->reader_index], read_size);

			retval += read_size;
			count -= read_size;

			advance_index(ring->reader_index, ring->size,
				      read_size);
		} else {
			if (retval)
				break;
			ring->reader_status = READ_BUF_EMPTY;
			buffer_size = get_read_buf_size(ring);
			if (buffer_size == 0) {
				if (ring->writer_status) {
					mv_ipc_mbox_post(dev->token,
							 ring->ring_event);
				}
				mvpipe_info("read wait...\n");
				if (wait_event_interruptible(dev->read_wait,
							     !is_buffer_empty
							     (ring)
							     ||
							     get_pipe_status
							     (dev)
							     != MVPIPE_OPEN)) {
					up(&dev->read_sem);
					return -ERESTARTSYS;
				}
				mvpipe_info("read awake...\n");
				if (get_pipe_status(dev) != MVPIPE_OPEN) {
					mvpipe_info
					("leave due to pipe closed\n");
					up(&dev->read_sem);
					return -EINTR;
				}
			}
			ring->reader_status = READ_BUF_IDLE;
		}
	}
	if (ring->writer_status)
		mv_ipc_mbox_post(dev->token, ring->ring_event);

	mvpipe_info("mvpipe read out %d bytes\n", retval);
	up(&dev->read_sem);

	if((dev->dump_type > NONE_MIN_TYPE) && (dev->dump_type < NONE_MAX_TYPE) && (retval > 0))
		ftrace_dump_data(dev->name,dev->dump_type,0, buf,retval,dev->dump_maxlen);

	return retval;
}

ssize_t mvpipe_dev_write(struct file *filp, const char __user *buf,
			 size_t count, loff_t *f_pos)
{
	ssize_t retval = 0;
	struct mvpipe_instance *dev = filp->private_data;
	uint32_t buffer_size, write_size;
	struct mvpipe_ring *ring;

	if (get_pipe_status(dev) != MVPIPE_OPEN ||
	    dev->mbox_status != MBOX_CONNECTED) {
		mvpipe_error("Write before Open!");
		return -EINTR;
	}

	if (down_interruptible(&dev->write_sem))
		return -ERESTARTSYS;

	ring = dev->ring[dev->writer_id];

	while (count != 0) {

		buffer_size = get_write_buf_size(ring);

		if (buffer_size > 0) {
			if (count <= buffer_size)
				write_size = count;
			else
				write_size = buffer_size;

			mvpipe_info("writing %d\n", write_size);
			copy_from_user(&ring->buf[ring->writer_index],
				       buf + retval, write_size);
			mvpipe_info("wrote %d\n", write_size);

			retval += write_size;
			count -= write_size;
			mvpipe_info("retval=%d, count=%d\n", retval, count);
			advance_index(ring->writer_index, ring->size,
				      write_size);
		} else {
			ring->writer_status = WRITE_BUF_FULL;
			buffer_size = get_write_buf_size(ring);
			if (buffer_size == 0) {
				if (ring->reader_status) {
					mv_ipc_mbox_post(dev->token,
							 ring->ring_event);
				}

				mvpipe_info("write wait...\n");

				if (wait_event_interruptible(dev->write_wait,
							     !is_buffer_full
							     (ring)
							     ||
							     get_pipe_status
							     (dev)
							     != MVPIPE_OPEN)) {
					up(&dev->write_sem);
					return retval;
				}
				mvpipe_info("write awake...\n");
				if (get_pipe_status(dev) != MVPIPE_OPEN) {
					mvpipe_info
					("leave due to pipe closed\n");
					up(&dev->write_sem);
					return -EINTR;
				}
			}
			ring->writer_status = WRITE_BUF_IDLE;

		}
	}

	if (ring->reader_status) {
		mvpipe_info("post event %d\n", ring->ring_event);
		mv_ipc_mbox_post(dev->token, ring->ring_event);
	}

	up(&dev->write_sem);
	mvpipe_info("returning %d\n", retval);

	if((dev->dump_type > NONE_MIN_TYPE) && (dev->dump_type < NONE_MAX_TYPE) && (retval > 0))
		ftrace_dump_data(dev->name,dev->dump_type,1, buf,retval,dev->dump_maxlen);

	return retval;
}

static unsigned int mvpipe_dev_poll(struct file *filp, poll_table *wait)
{
	struct mvpipe_instance *dev = filp->private_data;
	unsigned int result = 0;

	struct mvpipe_ring *read_ring = dev->ring[!dev->writer_id];
	struct mvpipe_ring *write_ring = dev->ring[dev->writer_id];

	poll_wait(filp, &dev->write_wait, wait);
	poll_wait(filp, &dev->read_wait, wait);
	poll_wait(filp, &dev->open_wait, wait);
	poll_wait(filp, &dev->close_wait, wait);

	if (get_pipe_status(dev) != MVPIPE_OPEN)
		return POLLHUP;

	if (!is_buffer_empty(read_ring))
		result |= (POLLIN | POLLRDNORM);

	if (!is_buffer_full(write_ring))
		result |= (POLLOUT | POLLWRNORM);

	return result;
}

const struct file_operations mvpipe_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = mvpipe_dev_read,
	.write = mvpipe_dev_write,
	.open = mvpipe_dev_open,
	.release = mvpipe_dev_release,
	.poll = mvpipe_dev_poll
};

static void mvpipe_on_connect(uint32_t token, void *cookie)
{
	struct mvpipe_instance *dev = (struct mvpipe_instance *)cookie;

	mvpipe_info("on connect event!\n");

	if (dev->mbox_status == MBOX_CONNECTED) {
		mvpipe_error
		("Received on connect irq, while dev is still connected!\n");
		return;
	}

	/* Setup rings according to handshake */
	mvpipe_info("Setup rings\n");

	if (dev->p_event->handshake == mv_gal_os_id()) {
		dev->writer_id = 0;
		dev->ring[dev->writer_id]->ring_event = RING0_EVENT;
		dev->ring[dev->writer_id]->size = dev->ring_size;
	} else {
		dev->writer_id = 1;
		dev->ring[dev->writer_id]->ring_event = RING1_EVENT;
		dev->ring[dev->writer_id]->size = dev->ring_size;
	}

	dev->ring[dev->writer_id]->writer_index = 0;
	dev->ring[dev->writer_id]->writer_status = WRITE_BUF_IDLE;

	dev->ring[!dev->writer_id]->reader_index = 0;
	dev->ring[!dev->writer_id]->reader_status = READ_BUF_IDLE;

	set_pipe_status(dev, MVPIPE_CLOSE);

	dev->mbox_status = MBOX_CONNECTED;

	/* wake up open wait */
	wake_up_interruptible(&dev->open_wait);
}

static void mvpipe_on_disconnect(uint32_t token, void *cookie)
{
	struct mvpipe_instance *dev = (struct mvpipe_instance *)cookie;
	mvpipe_info("on disconnect event!\n");

	if (dev->mbox_status == MBOX_DISCONNECTED) {
		mvpipe_error("on disconnect irq, but dev is disconnected!\n");
		return;
	}

	dev->mbox_status = MBOX_DISCONNECTED;

	set_pipe_status(dev, MVPIPE_CLOSE);

	mvpipe_close(dev);

	/* unblock close wait */
	dev->ready_to_release = 1;
	wake_up_interruptible(&dev->close_wait);
}

static void mvpipe_on_event(uint32_t token, uint32_t event_id, void *cookie)
{
	struct mvpipe_instance *dev = (struct mvpipe_instance *)cookie;
	switch (event_id) {
	case RING0_EVENT:
		mvpipe_info("on ring0 event!\n");
		if (is_ring0_writer(dev))
			wake_up_interruptible(&dev->write_wait);
		else
			wake_up_interruptible(&dev->read_wait);
		break;
	case RING1_EVENT:
		mvpipe_info("on ring1 event!\n");
		if (is_ring0_writer(dev))
			wake_up_interruptible(&dev->read_wait);
		else
			wake_up_interruptible(&dev->write_wait);
		break;
	case PIPE_EVENT:
		mvpipe_info("on pipe event!\n");
		switch (get_peer_status(dev)) {
		case MVPIPE_CLOSING:
			mvpipe_info("peer is closing!\n");
			if (get_pipe_status(dev) == MVPIPE_OPEN) {
				mvpipe_close(dev);
				set_pipe_status(dev, MVPIPE_CLOSE);
			}
			/* send ACK */
			mv_ipc_mbox_post(dev->token, dev->pipe_event);
			break;
		case MVPIPE_OPENING:
			if (get_pipe_status(dev) == MVPIPE_CLOSING) {
				set_pipe_status(dev, MVPIPE_CLOSE);
				dev->ready_to_release = 1;
				wake_up_interruptible(&dev->close_wait);
			}
			/* send ACK */
			mv_ipc_mbox_post(dev->token, dev->pipe_event);
			break;
		case MVPIPE_CLOSE:
			if (get_pipe_status(dev) == MVPIPE_CLOSING) {
				set_pipe_status(dev, MVPIPE_CLOSE);
				dev->ready_to_release = 1;
				wake_up_interruptible(&dev->close_wait);
			}
			break;
		case MVPIPE_OPEN:
		default:
			break;
		}
		if (get_pipe_status(dev) == MVPIPE_OPENING)
			wake_up_interruptible(&dev->open_wait);
		break;
	default:
		break;
	}
}

static struct mbox_ops mvpipe_mbox_ops = {
	.on_connect = mvpipe_on_connect,
	.on_disconnect = mvpipe_on_disconnect,
	.on_event = mvpipe_on_event
};

void on_mvpipe_instance(char *instance_name, uint32_t instance_index,
			uint32_t instance_count)
{
	unsigned char *shared_mem_start;
	unsigned int shared_mem_size;
	char *cmdline;
	struct mvpipe_instance *mvpipe;
	uint32_t token;
	uint32_t p_share_mem;
	char tmp_buf[64] = {0x00};

	dev_t mvpipe_dev;

	if (!mvpipes) {
		mvpipes =
			kmalloc(sizeof(struct mvpipe_instance) * instance_count,
				GFP_KERNEL);
		mvpipe_count = instance_count;
		cl_ipc = class_create(THIS_MODULE, "mvipc");
		if (cl_ipc == NULL) {
			pr_info("failed to create ipc class\n");
			return;
		}
	}

	mvpipe = &mvpipes[instance_index];
	strncpy(mvpipe->name, instance_name, sizeof(mvpipe->name));
	snprintf(mvpipe->dev_name, sizeof(mvpipe->dev_name), "mvpipe-%s",
		 instance_name);

	mvpipe->pipe_event = PIPE_EVENT;
	mvpipe->mbox_status = MBOX_DISCONNECTED;
	mvpipe->writer_id = 0;
	mvpipe->open_count = 0;
	mvpipe->ready_to_release = 0;

	if (alloc_chrdev_region(&mvpipe_dev, 0, 1, mvpipe->dev_name) < 0)
		return;

	if (device_create(cl_ipc, NULL, mvpipe_dev, NULL, mvpipe->dev_name) ==
	    NULL) {
		unregister_chrdev_region(mvpipe_dev, 1);
		return;
	}

	cdev_init(&mvpipe->cdev, &mvpipe_fops);

	if (cdev_add(&mvpipe->cdev, mvpipe_dev, 1) == -1) {
		device_destroy(cl_ipc, mvpipe_dev);
		unregister_chrdev_region(mvpipe_dev, 1);
		return;
	}

	pr_info("mvpipe enumerate instance name %s\n", instance_name);
	token = mv_ipc_mbox_get_info("mvpipe", instance_name, &mvpipe_mbox_ops,
				     &shared_mem_start, &shared_mem_size,
				     &cmdline, mvpipe);
	if (token != MBOX_INIT_ERR) {
		mvpipe->token = token;
		/*
		 * |------------------| <- share mem start / mvpipe_event start
		 * | mvpipe event     |
		 * |------------------| <- ring 0 start
		 * | ring 0 structure |
		 * |------------------| <- ring 0 buf start
		 * | ring 0 buf       |  <-- mvpipe->ring_size
		 * |------------------| <- ring 1 start
		 * | ring 1 structure |
		 * |------------------| <- ring 1 buf start
		 * | ring 1 buf       |  <-- mvpipe->ring_size
		 * |------------------| <- share mem end
		 */

		/* p_share_mem = share mem start */
		p_share_mem = (uint32_t) shared_mem_start;
		mvpipe->p_event = (struct mvpipe_event *)p_share_mem;

		/* p_share_mem = ring 0 start */
		p_share_mem += sizeof(struct mvpipe_event);
		mvpipe->ring[0] = (struct mvpipe_ring *)p_share_mem;

		/* p_share_mem = ring 0 buf */
		p_share_mem += sizeof(struct mvpipe_ring);

		/*
		 * 2 x Ring Size = Total Size - mvpipe event size
		 *                                 - ring0 structure size
		 *                                 - ring1 structure size
		 */
		mvpipe->ring_size = (shared_mem_size -
				     sizeof(struct mvpipe_event) -
				     sizeof(struct mvpipe_ring) * 2) / 2;

		mvpipe->ring_size &= ~0xF;

		/* p_share_mem = ring 1 start */
		p_share_mem += mvpipe->ring_size;
		mvpipe->ring[1] = (struct mvpipe_ring *)p_share_mem;

		sema_init(&mvpipe->open_sem, 1);
		sema_init(&mvpipe->read_sem, 1);
		sema_init(&mvpipe->write_sem, 1);

		init_waitqueue_head(&mvpipe->open_wait);
		init_waitqueue_head(&mvpipe->read_wait);
		init_waitqueue_head(&mvpipe->write_wait);
		init_waitqueue_head(&mvpipe->close_wait);

		mvpipe->p_event->handshake = mv_gal_os_id();

		mv_mbox_set_online(mvpipe->token);

        snprintf(tmp_buf, sizeof(tmp_buf), "%s_dump", instance_name);
		debugfs_create_u32(tmp_buf, 0644, mvpipe_debug_root, &mvpipe->dump_type);

        snprintf(tmp_buf, sizeof(tmp_buf), "%s_dump_len", instance_name);
        debugfs_create_u32(tmp_buf, 0644, mvpipe_debug_root, &mvpipe->dump_maxlen);

        mvpipe->dump_type = 0;
        mvpipe->dump_maxlen = 0;
        
#ifdef CONFIG_DEBUG_AT_PORT
        if(strstr(cmdline,"dbg"))
            mvpipe->is_dbg = 1;
        else
            mvpipe->is_dbg = 0;
#endif
        
	} else
		panic("failed to init instance\n");
}

static int mvpipe_module_init(void)
{
    mvpipe_debug_root = debugfs_create_dir("mvpipe",NULL);
    debugfs_create_u32("debug_print_lev", 0644, mvpipe_debug_root, &debug_print_lev);
    debugfs_create_u32("debug_print_type", 0644, mvpipe_debug_root, &debug_print_type);

	mv_ipc_mbox_for_all_instances("mvpipe", on_mvpipe_instance);
	return 0;
}

static void mvpipe_module_exit(void)
{
	debugfs_remove(mvpipe_debug_root);
	mvpipe_debug_root = NULL;
}

module_init(mvpipe_module_init);
module_exit(mvpipe_module_exit);
