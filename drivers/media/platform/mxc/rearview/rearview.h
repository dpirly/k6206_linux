/*
 * Copyright 2017 NXP.
 *
 * This code is based on:
 * Author: Ivan liu <xiaowen.liu@nxp.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (Version 2) as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _NXP_REARVIEW_H_
#define _NXP_REARVIEW_H_

#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/kref.h>
#include <linux/mutex.h>
#include <linux/kernel.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/videodev2.h>
#include <linux/kthread.h>
#include <linux/fb.h>
#include <media/v4l2-dev.h>
#include <media/videobuf-core.h>

#define MAX_BUFFERS 3

struct rear_v4l2 {
    struct inode inode;
    struct dentry dentry;
    struct file file;
    const struct file_operations* fops;
};

struct rear_fb {
    struct inode inode;
    struct dentry dentry;
    struct file file;
    const struct file_operations* fops;
    /* framebuffer */
    struct fb_fix_screeninfo fb_fix;
    struct fb_var_screeninfo fb_var;
};

struct rear_input {
    struct inode inode;
    struct dentry dentry;
    struct file file;
    const struct file_operations* fops;
};

enum rear_state {
    REAL_STATE_IDLE = 10,
    REAL_STATE_START,
    REAL_STATE_RUN,
    REAL_STATE_STOP,
    REAL_STATE_EXIT
};

struct rear_buffer {
    int fd;
    int phys;
};

struct rear_context {
    struct miscdevice device;
    struct mutex lock;

    // camera.
    struct rear_v4l2 camera;
    int fps;
    int camera_opened;
    int camera_on;
    int crop_x;
    int crop_y;
    int crop_w;
    int crop_h;
    int camera_subdev;

    // overlay.
    struct rear_fb overlay;
    int overlay_opened;
    int overlay_on;
    int screen_x;
    int screen_y;
    int screen_w;
    int screen_h;
    int overlay_subdev;

    // buffers.
    int format;
    int width;
    int height;
    int size;
    struct rear_buffer buffers[MAX_BUFFERS];

    // input.
    struct rear_input input;
    int input_subdev;

    // running state.
    enum rear_state state;
    // running thread.
    struct task_struct *rear_thread;
    struct task_struct *input_thread;
    int rearview_on;

    // create/destroy interface.
    int (*create_context)(struct rear_context* context);
    int (*destroy_context)(struct rear_context* context);
};

int rearview_thread(void *arg);
int rearinput_thread(void *arg);

#endif
