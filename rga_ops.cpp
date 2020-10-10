/*
 * =====================================================================================
 *
 *       Filename:  rga_ops.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2020年05月16日 17时28分39秒
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  zouxf (), 
 *   Organization:  Firefly
 *
 * =====================================================================================
 */
#include <stdlib.h>
#include "rga_ops.h"

void set_rect_format(rga_rect_t *rect, int format)
{
	rect->format = format;
}

void set_rect_size(rga_rect_t *rect, int w, int h)
{
	rect->wstride = w;
	rect->hstride = h;
}

void set_rect_crop(rga_rect_t *rect, int x, int y, int w, int h)
{
	rect->xoffset = x;
	rect->yoffset = y;
	rect->width = w;
	rect->height = h;
}

int V4l2ToRgaFormat(__u32 v4l2Format) {
    switch(v4l2Format) {
		case V4L2_PIX_FMT_ARGB32:
			return RK_FORMAT_RGBA_8888;
		case V4L2_PIX_FMT_RGB24:
			return RK_FORMAT_RGB_888;
		case V4L2_PIX_FMT_BGR24:
			return RK_FORMAT_BGR_888;
		case V4L2_PIX_FMT_ABGR32:
			return RK_FORMAT_BGRA_8888;
		case V4L2_PIX_FMT_RGB565:
			return RK_FORMAT_RGB_565;
		case V4L2_PIX_FMT_NV12:
			return RK_FORMAT_YCbCr_420_SP;  // Todo: why is Ycrcb NOT Ycbcr
		case V4L2_PIX_FMT_YUV420:
			return RK_FORMAT_YCbCr_420_P;
		case V4L2_PIX_FMT_NV16:
			return RK_FORMAT_YCrCb_422_SP;
		default:
			return RK_FORMAT_UNKNOWN;
    }    
}
