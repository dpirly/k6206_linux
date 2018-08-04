#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include "gpio-smi-ip175d.h"

#define KMAX  1024
#define GPIO_SMI_MDC_PIN			136		//clock
#define GPIO_SMI_MDIO_PIN			135		//data
#define DEVICE_NAME					"smi-ip175d"	//device name   /dev/smi-ip175d

#define CLK_CYCLE					250    //T = 400ns  frequency = 2.5MHz
#define CLK_READ_CYCLE				350    //T = 400ns  frequency = 2.5MHz

dev_t devno;
int major = 0;
int minor = 0;

int count = 1;
struct cdev *pdev;
struct class *pclass;
struct device *pdevices;

char kbuf[KMAX] = {};
int counter = 0;

typedef struct smi {
	int phy_addr;
	int reg_addr;
	int value;
}smi_dev;

smi_data_t kernle_buf;

void smi_start(void);
void mark_read(void);

int gpio_smi_open(struct inode *inodep, struct file *filep)
{
	//	printk("%s, %d\n", __func__, __LINE__);
	return 0;
}

int gpio_smi_release(struct inode *inodep, struct file *filep)
{
	//	printk("%s,%d\n", __func__, __LINE__);
	return 0;
}

ssize_t gpio_smi_read(struct file *file, char __user *buff, size_t size, loff_t *offlen)
{
	//	smi_dev smi_read;

	if(copy_to_user(buff, kbuf, size) != 0) {
		printk("failed to copy_to_user.\n");
		return -1;
	}
	counter = 0;
	return size;
}

ssize_t gpio_smi_write(struct file *file, const char __user *buff, size_t size, loff_t *offlen)
{
	if(copy_from_user(kbuf, buff, size) != 0) {
		return -1;
	}

	counter = size;
	printk("kbuf:%s size:%d\n", kbuf, counter);
	return size;
}

void smi_start(void)
{
	gpio_direction_output(GPIO_SMI_MDIO_PIN, 0);

	gpio_set_value(GPIO_SMI_MDC_PIN, 0);
	gpio_set_value(GPIO_SMI_MDIO_PIN, 0);
	ndelay(CLK_CYCLE);

	gpio_set_value(GPIO_SMI_MDC_PIN, 1);
	ndelay(CLK_CYCLE);

	gpio_set_value(GPIO_SMI_MDC_PIN, 0);
	gpio_set_value(GPIO_SMI_MDIO_PIN, 1);
	ndelay(CLK_CYCLE);

	gpio_set_value(GPIO_SMI_MDC_PIN, 1);
	ndelay(CLK_CYCLE);
}
void mark_read(void)
{
	gpio_set_value(GPIO_SMI_MDC_PIN, 0);
	gpio_set_value(GPIO_SMI_MDIO_PIN, 1);
	ndelay(CLK_CYCLE);

	gpio_set_value(GPIO_SMI_MDC_PIN, 1);
	ndelay(CLK_CYCLE);

	gpio_set_value(GPIO_SMI_MDC_PIN, 0);
	gpio_set_value(GPIO_SMI_MDIO_PIN, 0);
	ndelay(CLK_CYCLE);

	gpio_set_value(GPIO_SMI_MDC_PIN, 1);
	ndelay(CLK_CYCLE);
}
void mark_write(void)
{
	gpio_set_value(GPIO_SMI_MDC_PIN, 0);
	gpio_set_value(GPIO_SMI_MDIO_PIN, 0);
	ndelay(CLK_CYCLE);

	gpio_set_value(GPIO_SMI_MDC_PIN, 1);
	ndelay(CLK_CYCLE);

	gpio_set_value(GPIO_SMI_MDC_PIN, 0);
	gpio_set_value(GPIO_SMI_MDIO_PIN, 1);
	ndelay(CLK_CYCLE);

	gpio_set_value(GPIO_SMI_MDC_PIN, 1);
	ndelay(CLK_CYCLE);
}
void set_idle(void)
{
	int i;
	gpio_set_value(GPIO_SMI_MDC_PIN, 0);
	ndelay(CLK_CYCLE);
	gpio_set_value(GPIO_SMI_MDC_PIN, 1);
	ndelay(CLK_CYCLE);

	for(i=0; i<32; i++) {
		gpio_direction_output(GPIO_SMI_MDIO_PIN, 0);
		gpio_set_value(GPIO_SMI_MDC_PIN, 0);
		gpio_set_value(GPIO_SMI_MDIO_PIN, 1);
		ndelay(CLK_CYCLE);

		gpio_set_value(GPIO_SMI_MDC_PIN, 1);
		ndelay(CLK_CYCLE);
	}
}
int smi_ip175d_read(int PHYAD, int REGAD)
{
	int value = 0x0;
	int i;
	set_idle();
	smi_start();
	mark_read();
	for(i = 4; i >= 0; i--) {
		gpio_set_value(GPIO_SMI_MDC_PIN, 0);
		gpio_set_value(GPIO_SMI_MDIO_PIN, (PHYAD >> i) & 0x1);
		ndelay(CLK_CYCLE);

		gpio_set_value(GPIO_SMI_MDC_PIN, 1);
		ndelay(CLK_CYCLE);
	}
	for(i = 4; i >= 0; i--) {
		gpio_set_value(GPIO_SMI_MDC_PIN, 0);
		gpio_set_value(GPIO_SMI_MDIO_PIN, (REGAD >> i) & 0x1);
		ndelay(CLK_CYCLE);

		gpio_set_value(GPIO_SMI_MDC_PIN, 1);
		ndelay(CLK_CYCLE);
	}

	gpio_direction_input(GPIO_SMI_MDIO_PIN);
	gpio_set_value(GPIO_SMI_MDC_PIN, 0);
	ndelay(CLK_CYCLE);
	gpio_set_value(GPIO_SMI_MDC_PIN, 1);
	ndelay(CLK_CYCLE);
	////	gpio_set_value(GPIO_SMI_MDC_PIN, 0);
	////	ndelay(CLK_CYCLE);
	////	gpio_set_value(GPIO_SMI_MDC_PIN, 1);
	////	ndelay(CLK_CYCLE);
	for(i = 15; i >= 0; i--) {
		gpio_set_value(GPIO_SMI_MDC_PIN, 0);
		ndelay(CLK_CYCLE);

		gpio_set_value(GPIO_SMI_MDC_PIN, 1);
		value |= gpio_get_value(GPIO_SMI_MDIO_PIN) << i;
		ndelay(CLK_CYCLE);
	}
	gpio_set_value(GPIO_SMI_MDC_PIN, 0);
	ndelay(CLK_CYCLE);
	gpio_set_value(GPIO_SMI_MDC_PIN, 1);
	ndelay(CLK_CYCLE);
	//	printk("value = %x\n", value);
	return value;
}

void smi_ip175d_write(int PHYAD, int REGAD, int value)
{
	int i;
	smi_start();
	mark_write();

	for(i = 4; i >= 0; i--) {
		gpio_set_value(GPIO_SMI_MDC_PIN, 0);
		gpio_set_value(GPIO_SMI_MDIO_PIN, (PHYAD >> i) & 0x1);
		ndelay(CLK_CYCLE);

		gpio_set_value(GPIO_SMI_MDC_PIN, 1);
		ndelay(CLK_CYCLE);
	}
	for(i = 4; i >= 0; i--) {
		gpio_set_value(GPIO_SMI_MDC_PIN, 0);
		gpio_set_value(GPIO_SMI_MDIO_PIN, (REGAD >> i) & 0x1);
		ndelay(CLK_CYCLE);

		gpio_set_value(GPIO_SMI_MDC_PIN, 1);
		ndelay(CLK_CYCLE);
	}
	gpio_set_value(GPIO_SMI_MDC_PIN, 0);
	gpio_set_value(GPIO_SMI_MDIO_PIN, 1);
	ndelay(CLK_CYCLE);
	gpio_set_value(GPIO_SMI_MDC_PIN, 1);
	ndelay(CLK_CYCLE);

	gpio_set_value(GPIO_SMI_MDC_PIN, 0);
	gpio_set_value(GPIO_SMI_MDIO_PIN, 0);
	ndelay(CLK_CYCLE);
	gpio_set_value(GPIO_SMI_MDC_PIN, 1);
	ndelay(CLK_CYCLE);

	for(i = 15; i >= 0; i--) {
		gpio_set_value(GPIO_SMI_MDC_PIN, 0);
		gpio_set_value(GPIO_SMI_MDIO_PIN, (value >> i) & 0x1);
		ndelay(CLK_CYCLE);

		gpio_set_value(GPIO_SMI_MDC_PIN, 1);
		ndelay(CLK_CYCLE);
	}
}
static long smi_ip175d_ioctl(struct file * filp, unsigned int cmd, unsigned long arg)
{
	//	printk("-----------in_iocrl---------------\n");
	//转换参数为已知的指针类型
	smi_data_t* p = (smi_data_t*)arg;
	if(copy_from_user(&kernle_buf, p, sizeof(smi_data_t))) {
		printk("ioctl copy_to_user_failed!");
		return -EINVAL;
	}
	/*
	   printk("-----------ioctl 1--------\n");
	   printk("p->phy_id = %d\n",p->phy_id);
	   printk("kernle_buf.phy_id = %d\n",kernle_buf.phy_id);
	   printk("kernle_buf.rev_num = %d\n",kernle_buf.reg_num);
	   printk("kernle_buf.val_in = %x\n",kernle_buf.val_in);	*/
	//判断命令字的类型
	if(_IOC_TYPE(cmd) != GPIO_SMI_MARK) {
		printk("Wrong cmd!\n");
		return -EIO;
	}
	//判断读写方向，做相应的操作
	switch(_IOC_DIR(cmd)) {
		case _IOC_READ:
			//	printk("-----------ioctl read--------\n");
			kernle_buf.val_out = smi_ip175d_read(kernle_buf.phy_id, kernle_buf.reg_num);
			if(copy_to_user(p, &kernle_buf, sizeof(smi_data_t))) {
				printk("ioctl copy_to_user_failed!");
				return -EINVAL;
			}
			break;
		case _IOC_WRITE:
			//	printk("-----------ioctl write--------\n");
			smi_ip175d_write(kernle_buf.phy_id, kernle_buf.reg_num, kernle_buf.val_in);
			break;
		default:
			break;
	}
	return 0;

}
struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = gpio_smi_open,
	.read = gpio_smi_read,
	.write = gpio_smi_write,
	.release = gpio_smi_release,
	.unlocked_ioctl = smi_ip175d_ioctl,
};
static int __init gpio_smi_init(void)
{
	int ret = 0;
	printk("%s, %d\n", __func__, __LINE__);
	ret = alloc_chrdev_region(&devno, minor, count, "gpio-smi");
	if(ret) {
		printk("failed to alloc_chrdev_region.\n");
		return ret;
	}
	printk("devno:%d,major:%d minor:%d\n", devno, MAJOR(devno), MINOR(devno));
	pdev = cdev_alloc();
	if(pdev == NULL) {
		printk("failed to cdev_alloc.\n");
		goto err1;
	}
	cdev_init(pdev, &fops);
	ret = cdev_add(pdev, devno, count);
	if(ret) {
		printk("failed to cdev_add.\n");
		goto err2;
	}
	pclass = class_create(THIS_MODULE, "myclass");
	if(IS_ERR(pclass)) {
		printk("failed to class_create.\n");
		ret = PTR_ERR(pclass);
		goto err3;
	}
	pdevices = device_create(pclass, NULL, MKDEV(MAJOR(devno), MINOR(devno)), NULL, DEVICE_NAME);
	if(IS_ERR(pdevices)) {
		printk("failed to device_create.\n");
		ret = PTR_ERR(pdevices);
		goto err4;
	}
	gpio_request(GPIO_SMI_MDC_PIN, "smi-clk");
	gpio_direction_output(GPIO_SMI_MDC_PIN, 1);

	gpio_request(GPIO_SMI_MDIO_PIN, "smi-data");
	gpio_direction_input(GPIO_SMI_MDIO_PIN);
	return 0;

err4:
	device_destroy(pclass, MKDEV(MAJOR(devno), 0));
	class_destroy(pclass);
err3:
	cdev_del(pdev);
err2:
	kfree(pdev);
err1:
	unregister_chrdev_region(devno, count);
	return ret;
}
static void __exit gpio_smi_exit(void)
{
	printk("%s, %d\n", __func__, __LINE__);
	gpio_free(GPIO_SMI_MDC_PIN);
	gpio_free(GPIO_SMI_MDIO_PIN);
	device_destroy(pclass, MKDEV(MAJOR(devno), MINOR(devno)));
	class_destroy(pclass);
	cdev_del(pdev);
	kfree(pdev);
	unregister_chrdev_region(devno, count);
}

MODULE_LICENSE("GPL");
module_init(gpio_smi_init);
module_exit(gpio_smi_exit);
