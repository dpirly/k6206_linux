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
#include <media/v4l2-dev.h>
#include <media/videobuf-core.h>
#include <linux/export.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/dma-buf.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/bitops.h>
#include <linux/of.h>
#include <linux/of_platform.h>

#include "rearview.h"

static struct rear_context* g_context = NULL;
static const struct file_operations* char_fops = &def_chr_fops;

int rear_create_context(struct rear_context* context)
{
    struct rear_v4l2* camera = &context->camera;
    struct rear_fb* overlay = &context->overlay;
    struct rear_input* input = &context->input;

    overlay->dentry.d_inode = &overlay->inode;
    overlay->file.f_path.dentry = &overlay->dentry;
    overlay->file.f_inode = &overlay->inode;
    context->overlay_opened = 0;
    context->overlay_on = 0;

    camera->dentry.d_inode = &camera->inode;
    camera->file.f_path.dentry = &camera->dentry;
    camera->file.f_inode = &camera->inode;
    context->camera_opened = 0;
    context->camera_on = 0;

    input->dentry.d_inode = &input->inode;
    input->file.f_path.dentry = &input->dentry;
    input->file.f_inode = &input->inode;

    mutex_init(&context->lock);
    context->rear_thread = kthread_run(rearview_thread, context, "rearview");
    context->input_thread = kthread_run(rearinput_thread, context, "rearinput");

    return 0;
}

int rear_destroy_context(struct rear_context* context)
{
    mutex_lock(&context->lock);
    context->state = REAL_STATE_EXIT;
    mutex_unlock(&context->lock);

    return 0;
}

int camera_open(struct rear_context* context)
{
    // open camera 0 => /dev/video0.
    int err = 0;
    int try_count = 50;
    struct rear_v4l2* camera = &context->camera;

    camera->inode.i_rdev = (dev_t)MKDEV(VIDEO_MAJOR, context->camera_subdev);
    while (try_count >= 0) {
        try_count--;
        err = char_fops->open(&camera->inode, &camera->file);
        if (err == 0) {
            camera->fops = camera->file.f_op;
            printk("open camera 0 success!\n");
            break;
        }
        msleep(100);
    }

    if(err < 0) {
        printk("camera open failed\n");
        return err;
    }

    context->camera_opened = 1;
    return 0;
}

int camera_config(struct rear_context* context)
{
    int err = 0;
    int input;
    struct v4l2_streamparm param;
    struct v4l2_format fmt;
    struct rear_v4l2* camera = &context->camera;
    // below parameters hard code here.
    // they should be get from camera device.
    // format: NV12
    context->format = v4l2_fourcc('N', 'V', '1', '2');
    context->size = context->width * context->height * 3 / 2;

    // camera init.
    input = 1;
    err = camera->fops->unlocked_ioctl(&camera->file, VIDIOC_S_INPUT, (unsigned long)&input);
    if (err < 0) {
        printk("camera: set input failed\n");
        return err;
    }

    memset(&param, 0, sizeof(param));
    param.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    param.parm.capture.timeperframe.numerator   = 1;
    param.parm.capture.timeperframe.denominator = context->fps;
    param.parm.capture.capturemode = 4;//1280x720.
    err = camera->fops->unlocked_ioctl(&camera->file, VIDIOC_S_PARM, (unsigned long)&param);
    if (err < 0) {
        printk("camera: set param failed\n");
        return err;
    }

    memset(&fmt, 0, sizeof(fmt));
    fmt.type                 = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width        = context->width & 0xFFFFFFF8;
    fmt.fmt.pix.height       = context->height & 0xFFFFFFF8;
    fmt.fmt.pix.pixelformat  = context->format;
    fmt.fmt.pix.priv         = 0;
    fmt.fmt.pix.sizeimage    = 0;
    fmt.fmt.pix.bytesperline = 0;
    err = camera->fops->unlocked_ioctl(&camera->file, VIDIOC_S_FMT, (unsigned long)&fmt);
    if (err < 0) {
        printk("camera: set fmt failed\n");
        return err;
    }

    return 0;
}

int camera_start(struct rear_context* context)
{
    int err = 0;
    uint32_t i;
    struct v4l2_buffer buf;
    struct v4l2_requestbuffers req;
    struct v4l2_buffer cfilledbuffer;
    enum v4l2_buf_type bufType;
    struct rear_v4l2* camera = &context->camera;

    memset(&req, 0, sizeof (req));
    req.count = MAX_BUFFERS;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_USERPTR;
    err = camera->fops->unlocked_ioctl(&camera->file, VIDIOC_REQBUFS, (unsigned long)&req);
    if (err < 0) {
        printk("%s VIDIOC_REQBUFS failed\n", __func__);
        return err;
    }

    for (i = 0; i < MAX_BUFFERS; i++) {
        memset(&buf, 0, sizeof (buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.index = i;
        buf.memory = V4L2_MEMORY_USERPTR;
        buf.m.offset = context->buffers[i].phys;
        buf.length   = context->size;
        err = camera->fops->unlocked_ioctl(&camera->file, VIDIOC_QUERYBUF, (unsigned long)&buf);
        if (err < 0) {
            printk("%s VIDIOC_QUERYBUF failed\n", __func__);
            return err;
        }
    }

    for (i = 0; i < MAX_BUFFERS; i++) {
        memset(&cfilledbuffer, 0, sizeof (struct v4l2_buffer));
        cfilledbuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        cfilledbuffer.memory = V4L2_MEMORY_USERPTR;
        cfilledbuffer.index    = i;
        cfilledbuffer.m.offset = context->buffers[i].phys;

        printk("%s VIDIOC_QBUF phy:0x%x\n", __func__, context->buffers[i].phys);
        err = camera->fops->unlocked_ioctl(&camera->file, VIDIOC_QBUF, (unsigned long)&cfilledbuffer);
        if (err < 0) {
            printk("%s VIDIOC_QBUF Failed\n", __func__);
            return err;
        }
    }

    //-------stream on-------
    bufType = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    err = camera->fops->unlocked_ioctl(&camera->file, VIDIOC_STREAMON, (unsigned long)&bufType);
    if (err < 0) {
        printk("%s VIDIOC_STREAMON failed\n", __func__);
        return err;
    }
    context->camera_on = 1;

    return 0;
}

int camera_stop(struct rear_context* context)
{
    int err = 0;
    struct rear_v4l2* camera = &context->camera;
    enum v4l2_buf_type bufType;
    bufType = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    err = camera->fops->unlocked_ioctl(&camera->file, VIDIOC_STREAMOFF, (unsigned long)&bufType);
    if (err < 0) {
        printk("%s VIDIOC_STREAMOFF failed\n", __func__);
        return err;
    }
    context->camera_on = 0;

    return 0;
}

int camera_close(struct rear_context* context)
{
    struct rear_v4l2* camera = &context->camera;
    if (context->camera_opened) {
        char_fops->release(&camera->inode, &camera->file);
        context->camera_opened = 0;
    }

    return 0;
}

int camera_get_frame(struct rear_context* context, int* index)
{
    int err = 0;
    struct rear_v4l2* camera = &context->camera;
    struct v4l2_buffer cfilledbuffer;
    memset(&cfilledbuffer, 0, sizeof (cfilledbuffer));
    cfilledbuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    cfilledbuffer.memory = V4L2_MEMORY_USERPTR;

    err = camera->fops->unlocked_ioctl(&camera->file, VIDIOC_DQBUF, (unsigned long)&cfilledbuffer);
    if (err < 0) {
        printk("%s VIDIOC_DQBUF, failed\n", __func__);
        return err;
    }

    *index = cfilledbuffer.index;
    return 0;
}

int camera_put_frame(struct rear_context* context, int index)
{
    int err = 0;
    struct rear_v4l2* camera = &context->camera;
    struct v4l2_buffer cfilledbuffer;
    memset(&cfilledbuffer, 0, sizeof (struct v4l2_buffer));
    cfilledbuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    cfilledbuffer.memory = V4L2_MEMORY_USERPTR;
    cfilledbuffer.index    = index;
    cfilledbuffer.m.offset = context->buffers[index].phys;

    err = camera->fops->unlocked_ioctl(&camera->file, VIDIOC_QBUF, (unsigned long)&cfilledbuffer);
    if (err < 0) {
        printk("%s VIDIOC_QBUF, failed\n", __func__);
        return err;
    }

    return 0;
}

int overlay_open(struct rear_context* context)
{
    int err = 0;
    int try_count = 50;
    struct rear_fb* overlay = &context->overlay;
    struct fb_fix_screeninfo fb_fix;

    overlay->inode.i_rdev = (dev_t)MKDEV(FB_MAJOR, context->overlay_subdev);
    while (try_count >= 0) {
        try_count--;
        err = char_fops->open(&overlay->inode, &overlay->file);
        if (err == 0) {
            overlay->fops = overlay->file.f_op;
            printk("open fb1 success!\n");
            break;
        }
        msleep(100);
    }

    if(err < 0) {
        printk("fail to open fb1\n");
        return err;
    }

    err = overlay->fops->unlocked_ioctl(&overlay->file, FBIOGET_FSCREENINFO, (unsigned long)&fb_fix);
    if(err < 0) {
        printk("fb1 FBIOGET_FSCREENINFO failed\n");
        return err;
    }
    if (!strstr(fb_fix.id, "FG")) {
        printk("fb1 name:%s is not overlay\n", fb_fix.id);
        return -1;
    }
    context->overlay_opened = 1;

    return 0;
}

int overlay_close(struct rear_context* context)
{
    struct rear_fb* overlay = &context->overlay;
    if (context->overlay_opened) {
        char_fops->release(&overlay->inode, &overlay->file);
        context->overlay_opened = 0;
    }

    return 0;
}

int overlay_start(struct rear_context* context)
{
    int err = 0;
    uint32_t i;
    struct rear_fb* overlay = &context->overlay;
    struct fb_fix_screeninfo fb_fix;
    struct fb_var_screeninfo fb_var;
    int blank;

    err = overlay->fops->unlocked_ioctl(&overlay->file, FBIOGET_VSCREENINFO, (unsigned long)&fb_var);
    if(err < 0) {
        printk("fb1 FBIOGET_VSCREENINFO failed\n");
        return err;
    }

    fb_var.xoffset = 0;
    fb_var.yoffset = 0;
    fb_var.xres = context->width;
    fb_var.yres = context->height;
    fb_var.yres_virtual = fb_var.yres * MAX_BUFFERS;
    fb_var.xres_virtual = fb_var.xres;
    fb_var.nonstd = context->format;
    fb_var.activate |= FB_ACTIVATE_FORCE;
    fb_var.bits_per_pixel = 12;

    err = overlay->fops->unlocked_ioctl(&overlay->file, FBIOPUT_VSCREENINFO, (unsigned long)&fb_var);
    if(err < 0) {
        printk("overlay: FBIOPUT_VSCREENINFO failed\n");
        return err;
    }

    err = overlay->fops->unlocked_ioctl(&overlay->file, FBIOGET_VSCREENINFO, (unsigned long)&fb_var);
    if(err < 0) {
        printk("overlay: FBIOGET_VSCREENINFO failed\n");
        return err;
    }

    err = overlay->fops->unlocked_ioctl(&overlay->file, FBIOGET_FSCREENINFO, (unsigned long)&fb_fix);
    if(err < 0) {
        printk("overlay: FBIOGET_FSCREENINFO failed\n");
        return err;
    }

    if (fb_fix.smem_len <= 0) {
        printk("overlay: can't get effective memory\n");
        return -ENOMEM;
    }

    for (i = 0; i < MAX_BUFFERS; i++) {
        context->buffers[i].phys = fb_fix.smem_start + i * fb_fix.line_length * fb_var.yres;
    }

    blank = FB_BLANK_UNBLANK;
    err = overlay->fops->unlocked_ioctl(&overlay->file, FBIOBLANK, (unsigned long)blank);
    if(err < 0) {
        printk("overlay FBIOBLANK failed\n");
        return err;
    }

    overlay->fb_var = fb_var;
    overlay->fb_fix = fb_fix;
    context->overlay_on = 1;

    return 0;
}

int overlay_update_frame(struct rear_context* context, int index)
{
    int err = 0;
    struct rear_fb* overlay = &context->overlay;

    const size_t offset = context->buffers[index].phys - context->buffers[0].phys;
    overlay->fb_var.activate = FB_ACTIVATE_VBL;
    overlay->fb_var.yoffset = offset / overlay->fb_fix.line_length;
    err = overlay->fops->unlocked_ioctl(&overlay->file, FBIOPAN_DISPLAY, (unsigned long)&overlay->fb_var);
    if(err < 0) {
        printk("overlay FBIOPAN_DISPLAY failed\n");
        return err;
    }

    return 0;
}

int overlay_stop(struct rear_context* context)
{
    int err = 0;
    int blank;
    struct rear_fb* overlay = &context->overlay;

    if (!context->overlay_on) {
        return 0;
    }

    blank = FB_BLANK_POWERDOWN;
    err = overlay->fops->unlocked_ioctl(&overlay->file, FBIOBLANK, (unsigned long)blank);
    if(err < 0) {
        printk("overlay: FBIOBLANK failed\n");
        return err;
    }

    context->overlay_on = 0;
    return 0;
}

int rearview_open(struct rear_context* context)
{
    int err = 0;
    err = camera_open(context);
    if (err != 0) {
        printk("camera_open failed\n");
        return err;
    }

    err = overlay_open(context);
    if (err != 0) {
        printk("overlay_open failed\n");
        return err;
    }

    return 0;
}

int rearview_start(struct rear_context* context)
{
    int err = 0;

    err = camera_config(context);
    if (err != 0) {
        printk("camera config failed\n");
        return err;
    }

    err = overlay_start(context);
    if (err != 0) {
        printk("overlay_start failed\n");
        return err;
    }

    err = camera_start(context);
    if (err != 0) {
        printk("camera start failed\n");
        return err;
    }

    return 0;
}

int rearview_run(struct rear_context* context)
{
    int err = 0;
    int index = -1;

    err = camera_get_frame(context, &index);
    if (err != 0) {
        printk("camera get frame failed\n");
        return err;
    }

    err = overlay_update_frame(context, index);
    if (err != 0) {
        printk("overlay show frame failed\n");
        return err;
    }

    err = camera_put_frame(context, index);
    if (err != 0) {
        printk("camera put frame failed\n");
        return err;
    }

    return 0;
}

int rearview_stop(struct rear_context* context)
{
    int err = 0;
    err = camera_stop(context);
    if (err != 0) {
        printk("camera stop failed\n");
        return err;
    }

    err = overlay_stop(context);
    if (err != 0) {
        printk("overlay stop failed\n");
        return err;
    }

    return 0;
}

int rearview_close(struct rear_context* context)
{
    int err = 0;
    err = camera_close(context);
    if (err != 0) {
        printk("camera close failed\n");
        return err;
    }

    err = overlay_close(context);
    if (err != 0) {
        printk("overlay close failed\n");
        return err;
    }

    return 0;
}

int rearview_thread(void *arg)
{
    struct rear_context* context = (struct rear_context*)arg;
    int state;
    char quit = 0;
    int err = 0;

    err = rearview_open(context);
    if (err != 0) {
        printk("rearview open failed\n");
        return err;
    }

    while (true) {
        mutex_lock(&context->lock);
        state = context->state;
        mutex_unlock(&context->lock);

        switch (state) {
            case REAL_STATE_IDLE:
                mutex_lock(&context->lock);
                if (context->rearview_on) {
                    context->state = REAL_STATE_START;
                    mutex_unlock(&context->lock);
                    break;
                }
                mutex_unlock(&context->lock);
                msleep(100);
                break;

            case REAL_STATE_START:
                printk("start state\n");
                err = rearview_start(context);
                mutex_lock(&context->lock);
                if (err != 0) {
                    printk("rearview_start failed exit.\n");
                    quit = 1;
                    context->state = REAL_STATE_EXIT;
                }
                else {
                    context->state = REAL_STATE_RUN;
                }
                mutex_unlock(&context->lock);
                break;

            case REAL_STATE_RUN:
                mutex_lock(&context->lock);
                if (!context->rearview_on) {
                    printk("run but rearview is off, go into idle\n");
                    context->state = REAL_STATE_STOP;
                    mutex_unlock(&context->lock);
                    break;
                }
                mutex_unlock(&context->lock);

                rearview_run(context);
                break;

            case REAL_STATE_EXIT:
                printk("exit state\n");
                quit = 1;
            case REAL_STATE_STOP:
                printk("stop state\n");
                mutex_lock(&context->lock);
                context->state = REAL_STATE_IDLE;
                mutex_unlock(&context->lock);

                rearview_stop(context);
                break;

            default:
                printk("invalid state\n");
                msleep(100);
                break;
        }

        if (quit) {
            printk("quit\n");
            break;
        }
    }

    err = rearview_close(context);
    if (err != 0) {
        printk("rearview close failed\n");
        return err;
    }

    return 0;
}

static int rearinput_open(struct rear_context* context)
{
    int err = 0;
    int try_count = 200;
    struct rear_input* input = &context->input;
    uint8_t key_bitmask[(KEY_MAX+1)/8];

    input->inode.i_rdev = (dev_t)MKDEV(INPUT_MAJOR, context->input_subdev);
    while (try_count >= 0) {
        try_count--;
        err = char_fops->open(&input->inode, &input->file);
        if (err == 0) {
            input->fops = input->file.f_op;
            printk("open input success!\n");
            break;
        }
        msleep(200);
    }

    if(err < 0) {
        printk("open input:%d failed:%d\n", context->input_subdev, err);
        return err;
    }

    memset(key_bitmask, 0, sizeof(key_bitmask));
    err = input->fops->unlocked_ioctl(&input->file, EVIOCGBIT(EV_KEY, sizeof(key_bitmask)), (unsigned long)key_bitmask);
    if (err < 0) {
        printk("input EVIOCGBIT failed\n");
        return err;
    }

    if (!test_bit(KEY_VOLUMEUP, (const volatile unsigned long *)key_bitmask)) {
        printk("KEY_VOLUMEUP not supported\n");
        return -ENODEV;
    }

    return 0;
}

static int rearinput_close(struct rear_context* context)
{
    struct rear_input* input = &context->input;
    char_fops->release(&input->inode, &input->file);
    return 0;
}

int rearinput_thread(void *arg)
{
    struct rear_context* context = (struct rear_context*)arg;
    struct rear_input* input = &context->input;
    struct input_event iev;
    int res = 0;
    loff_t offset = 0;

    res = rearinput_open(context);
    if (res != 0) {
        printk("rearinput open failed\n");
        return res;
    }

    while (true) {
        res = input->fops->read(&input->file, (char *)&iev, sizeof(iev), &offset);
        if (res == sizeof(iev)) {
            if (iev.code == KEY_VOLUMEUP && iev.value == 1) {
                mutex_lock(&context->lock);
                if (context->rearview_on)
                    context->rearview_on = 0;
                else
                    context->rearview_on = 1;
                mutex_unlock(&context->lock);
            }
        }
        else {
            printk("can not get event\n");
            break;
        }
    }

    res = rearinput_close(context);
    if (res != 0) {
        printk("rearinput close failed\n");
        return res;
    }

    return 0;
}

static long rearview_dev_ioctl(struct file *filep, unsigned long cmd, unsigned long arg)
{
    return 0;
}

static int rearview_dev_open(struct inode *inode, struct file *filep)
{
    printk("open rearview dev.\n");
    //g_context;
    return 0;
}

static int rearview_dev_release(struct inode *inode, struct file *filep)
{
    printk("close rearview dev.\n");
    return 0;
}

static struct file_operations rearview_dev_fops = {
    .owner = THIS_MODULE,
    .open = rearview_dev_open,
    .release = rearview_dev_release,
#ifdef CONFIG_COMPAT
    .compat_ioctl = rearview_dev_ioctl,
#endif
    .unlocked_ioctl = rearview_dev_ioctl
};

static struct miscdevice rearview_dev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "rearview",
    .fops = &rearview_dev_fops
};

static int __init rearview_dev_probe(struct platform_device *pdev)
{
    int err = 0;
    struct rear_context* context = NULL;
    int on = 0, fps = 0, width = 0, height = 0;
    int camera_dev = 0, overlay_dev = 0, input_dev = 0;
    const struct device_node *node = pdev->dev.of_node;

    context = kmalloc(sizeof(*context), GFP_KERNEL);
    if (context == NULL) {
        printk("not enough memory to malloc rearview context\n");
        return -ENOMEM;
    }

    memset(context, 0, sizeof(*context));
    context->device = rearview_dev;

    err = misc_register(&context->device);
    if (err) {
        printk("rearview: misc driver register failed\n");
        rear_destroy_context(context);
        kfree(context);
        context = NULL;
        return err;
    }

    if (node) {
        printk("rearview: get property from DTS\n");
        of_property_read_s32(node, "on", &on);
        of_property_read_s32(node, "fps", &fps);
        of_property_read_s32(node, "width", &width);
        of_property_read_s32(node, "height", &height);
        of_property_read_s32(node, "camera_subdev", &camera_dev);
        of_property_read_s32(node, "overlay_subdev", &overlay_dev);
        of_property_read_s32(node, "input_subdev", &input_dev);
        printk("on:%d, fps:%d, w:%d, h:%d, cam_dev:%d, ov_dev:%d, in_dev:%d\n",
           on, fps, width, height, camera_dev, overlay_dev, input_dev);
    }

    // set initial value.
    if (!on) on = 1;
    if (!fps) fps = 30;
    if (!width) width = 1280;
    if (!height) height = 720;
    if (!camera_dev) camera_dev = 0;
    if (!overlay_dev) overlay_dev = 1;
    if (!input_dev) input_dev = 70;

    platform_set_drvdata(pdev, (void*)context);
    g_context = context;

    context->rearview_on = on;
    context->fps = fps;
    context->camera_subdev = camera_dev;
    context->overlay_subdev = overlay_dev;
    context->input_subdev = input_dev;
    context->width = width;
    context->height = height;

    context->state = REAL_STATE_START;
    rear_create_context(context);

    printk("rearview: init success.\n");
    return 0;
}

static int rearview_dev_remove(struct platform_device *pdev)
{
    struct rear_context* context = (struct rear_context*)platform_get_drvdata(pdev);

    misc_deregister(&context->device);
    rear_destroy_context(context);
    kfree(context);
    g_context = NULL;

    return 0;
}

static struct of_device_id rearview_match_table[] = {
    {.compatible = "fsl,rearview"},
    {},
};

static struct platform_driver rearview_driver = {
    .probe = rearview_dev_probe,
    .remove = rearview_dev_remove,
    .driver = {
        .name = "rearview",
        .of_match_table = rearview_match_table,
     },
};

static int __init rearview_dev_init(void)
{
    return platform_driver_register(&rearview_driver);
}

static void __exit rearview_dev_exit(void)
{
    platform_driver_unregister(&rearview_driver);
}

module_init(rearview_dev_init);
module_exit(rearview_dev_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("rearview for IVI");
MODULE_LICENSE("GPL");
