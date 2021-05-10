/*
 * =====================================================================================
 *
 *       Filename:  rga_ops.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2020年05月16日 17时30分07秒
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  zouxf (), 
 *   Organization:  Firefly
 *
 * =====================================================================================
 */
#include <stdio.h>
#include <unistd.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include <rga/RockchipRga.h>

enum yuv2RgbMode{
    RGB_TO_RGB = 0,
    YUV_TO_YUV = 0,
    YUV_TO_RGB = 0x1 << 0,
    RGB_TO_YUV = 0x2 << 4,
};

void set_rect_format(rga_rect_t *rect, int format);
void set_rect_size(rga_rect_t *rect, int w, int h);
void set_rect_crop(rga_rect_t *rect, int x, int y, int w, int h);
int V4l2ToRgaFormat(__u32 v4l2Format);


