/*
 *  stk8baxx.h - Definitions for sensortek stk8baxx accelerometer
 *
 *  Copyright (C) 2012~2014 Lex Hsieh / sensortek <lex_hsieh@sensortek.com.tw>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */   
 
#ifndef __STK8BAXX__
#define __STK8BAXX__

#include <linux/ioctl.h>
#include <linux/types.h>

//#define IS_STK8BA22
#define IS_STK8BA50

#ifdef IS_STK8BA22
	#define LSB_1G	64	
#elif defined IS_STK8BA50
	#define LSB_1G	256	
#endif
/*	direction settings	*/
static const int coordinate_trans[8][3][3] = 
{
	/* x_after, y_after, z_after */
	{{0,-1,0}, {1,0,0}, {0,0,1}},
	{{1,0,0}, {0,1,0}, {0,0,1}},
	{{0,1,0}, {-1,0,0}, {0,0,1}},
	{{-1,0,0}, {0,-1,0}, {0,0,1}},
	{{0,1,0}, {1,0,0}, {0,0,-1}},
	{{-1,0,0}, {0,1,0}, {0,0,-1}},
	{{0,-1,0}, {-1,0,0}, {0,0,-1}},
	{{1,0,0}, {0,-1,0}, {0,0,-1}},

};

/* IOCTLs*/	
#define STKDIR						0x3E
#define STK_IOCTL_WRITE				_IOW(STKDIR, 0x01, char[8])
#define STK_IOCTL_READ				_IOWR(STKDIR, 0x02, char[8])
#define STK_IOCTL_SET_ENABLE			_IOW(STKDIR, 0x03, char)
#define STK_IOCTL_GET_ENABLE			_IOR(STKDIR, 0x04, char)
#define STK_IOCTL_SET_DELAY			_IOW(STKDIR, 0x05, char)
#define STK_IOCTL_GET_DELAY			_IOR(STKDIR, 0x06, char)
#define STK_IOCTL_SET_OFFSET			_IOW(STKDIR, 0x07, int[3])
#define STK_IOCTL_GET_OFFSET			_IOR(STKDIR, 0x08, int[3])
#define STK_IOCTL_GET_ACCELERATION	_IOR(STKDIR, 0x09, int[3])
#define STK_IOCTL_SET_RANGE			_IOW(STKDIR, 0x10, char)
#define STK_IOCTL_GET_RANGE			_IOR(STKDIR, 0x11, char)
#define STK_IOCTL_SET_CALI			_IOW(STKDIR, 0x12, char)

struct stk8baxx_platform_data
{
	unsigned char direction;
	int interrupt_pin;	
};



#endif	/* #ifndef __STK8BAXX__ */