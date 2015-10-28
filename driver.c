/* driver.c
 *
 * open board led driver
 *
 * Copyright (c) 
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <asm/irq.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include "../../header_file/gpio_table.h"
#include "../../header_file/ioctl_magic.h"


#define DEVICE_NAME "my_lcd"

//#define DEG
#ifdef DEG
#define deg(fmt, args...) printk(fmt, ## args)
#else
#define deg(fmt, args...)
#endif

//DEFINE MACROS 
#define SET	1
#define CLEAR	0
#define MAXSIZE	17


#define LCDClear() LCDWriteCommand(0x01)	  /* Clear display LCD */
#define LCDRow1()  LCDWriteCommand(0x80)	  /* Begin at Line 1 */
#define LCDRow2()  LCDWriteCommand(0xC0)   /* Begin at Line 2 */

//#define Delay(x)   usleep(x*1000)
//USERDEFINE DATA TYPES
typedef unsigned char LCDubyte; 

//DEFINE PROTOTYPES
static void  LCDEnable(void);
void LCDWriteCommand(LCDubyte command);
void LCDWriteData(LCDubyte ascii);
void LCDWriteString(LCDubyte *lcd_string);
static void LCDWriteByte(LCDubyte  LCDData);
void lcdinit(void);
void LCDDisplayInitializing(void);
void LCDDisplayByte(LCDubyte LCDAdress, LCDubyte Value);
static void LCDReset(void);

static unsigned int value[128];
static void  gpio_req(unsigned int arg);
static void gpio_set_val(unsigned int arg, int val);

static char d_buf[MAXSIZE];

/**************** This is used for opening the gpio_node ********************/

int open_board_open (struct inode *inode, struct file *file)
{
	deg(KERN_INFO"Gpio_module opened\n");
	return 0;
}

/****************** This is used for closing the gpio_node *****************/

int open_board_close (struct inode *inode, struct file *file)
{
	deg(KERN_INFO"Gpio_module closed\n");
	return 0;
}

irqreturn_t gpio_irq_handler(int irq, void *dev) 
{
	int device = (int)dev;

	deg("module is in irq handler irq\tdevice =%d\n",device);
	value[device]^=1;
	deg("i am in irq value is %d\t and device = %d\n\
		",value[device],device);
	return IRQ_HANDLED;
}

/******* IOCTL is used for setting the direction of gpio ******************/

long open_board_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	if (arg > 128)                      
	{
		return -EINVAL;
	}

	switch(cmd)
	{
	case IOCTL_GPIO_TO_IRQ:
		if (gpio_to_irq(arg))
			deg("gpio_to_irq for gpio_pin = %lu failed\n",arg);
		else
			deg("gpio_to_irq for gpio_pin = %lu successful\n",arg);
		break;
	case IOCTL_GPIO_REQ:
		if(gpio_request(arg, "Gpio Driver"))
			deg(KERN_WARNING "Gpio_driver: unable to request \
				GPIO %lu\n",arg);
		else
			deg("gpio request cmd = %d\t arg = %lu\n"\
				,IOCTL_GPIO_REQ,arg);
		break;
	case IOCTL_DIR_IN:
			gpio_direction_input(gpio_table[arg]);
			deg("gpio direction as input cmd = %d\t arg \
				= %lu\n",IOCTL_DIR_IN,arg);
		break;
	case IOCTL_REQUEST_IRQ:
		if (request_irq(gpio_to_irq(arg),gpio_irq_handler,\
			IRQ_TYPE_EDGE_RISING,"gpio irq handler",(void*)arg))
			deg("gpio interrupt request failed\n");
		else
			deg("gpio interrupt request successful\n");
		break;       
	case IOCTL_DIR_OUT:
			gpio_direction_output(gpio_table[arg], 1);
			deg("\ngpio direction as output cmd = %d\t arg \
				= %lu\n",IOCTL_DIR_OUT,arg);
		break;
	case IOCTL_SET_GPIO:
			deg("gpio set cmd = %d\t arg \
				= %lu\n",IOCTL_SET_GPIO,arg);
			gpio_set_value(gpio_table[arg], 1);
		break;
	case IOCTL_CLR_GPIO:
			deg("gpio clr cmd = %d\t arg \
				= %lu\n",IOCTL_CLR_GPIO,arg);
			gpio_set_value(gpio_table[arg], 0);
		break;
	case IOCTL_FREE_GPIO:
			gpio_free(arg);
			deg("gpio free cmd = %d\t arg \
				= %lu\n",IOCTL_FREE_GPIO,arg);
		break;
	case IOCTL_FREE_IRQ:
			free_irq(gpio_to_irq(arg),(void*)arg);
		break;
	default:
		return -EINVAL;
	}
	return 0;	
}

/***************************** READ **********************************/

ssize_t open_board_read(struct file *file, char __user *usr, 
			size_t ret, loff_t *off)
{
	ret=copy_to_user(usr, value, ret);
	return ret;
}

static ssize_t open_board_write(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
    if(count < MAXSIZE) {
        copy_from_user(d_buf, buf, count);
        d_buf[count-1] = 0;
	LCDWriteString(d_buf);
        *f_pos += count;
        return count;
        } else {
        copy_from_user(d_buf, buf, MAXSIZE - 1);
        d_buf[MAXSIZE - 1] = 0;
	LCDWriteString(d_buf);
        *f_pos += MAXSIZE - 1;
        return MAXSIZE - 1;
    }
}
static struct file_operations dev_fops = {
	.owner	        = THIS_MODULE,
	.open           = open_board_open,
	.release        = open_board_close,
	.unlocked_ioctl	= open_board_ioctl,
	.read           = open_board_read,
	.write		= open_board_write,
};

static struct miscdevice misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &dev_fops,
};

static int __init dev_init(void)
{
	int ret;

	ret = misc_register(&misc);
	deg(KERN_INFO DEVICE_NAME"\tinitialized\n");
  		
	lcdinit();
	LCDDisplayInitializing();


	return ret;
}

static void __exit dev_exit(void)
{
	misc_deregister(&misc);
}

static void gpio_req(unsigned int arg)
{

	if(gpio_request(arg, "Gpio Driver"))
			deg(KERN_WARNING "Gpio_driver: unable to request \
				GPIO %lu\n",arg);
	else
			deg("gpio request cmd = %d\t arg = %lu\n"\
				,IOCTL_GPIO_REQ,arg);

	gpio_direction_output(arg, 1);

}

static void gpio_set_val(unsigned int arg, int val)
{
	gpio_set_value(gpio_table[arg], val);
}

static void  LCDEnable(void)
{
    gpio_set_val(75,SET);
	udelay(50);
    gpio_set_val(75,CLEAR);
}
	
void LCDWriteCommand(LCDubyte LCDData)
{
	gpio_set_val(74,CLEAR); 
	LCDWriteByte(LCDData);
 }

void LCDWriteData(LCDubyte LCDData)
{
   	gpio_set_val(74,SET); 
	LCDWriteByte(LCDData);
}

void LCDWriteString(LCDubyte *lcd_string)
{
	while (*lcd_string) 
		LCDWriteData(*lcd_string++);
}

void lcdinit(void)
{
	udelay(1000);
	gpio_req(70);
	gpio_req(71);
	gpio_req(72);
	gpio_req(73);
	gpio_req(74);
	gpio_req(75);
	udelay(5000);	
	gpio_set_val(70,0);
	gpio_set_val(71,0);
	gpio_set_val(72,0);
	gpio_set_val(73,0);
	gpio_set_val(74,0);
	gpio_set_val(75,0);
	
	udelay(5000);
    LCDReset();
    LCDWriteCommand(0x28);
    LCDWriteCommand(0x0C);
    LCDWriteCommand(0x06);
    LCDWriteCommand(0x01); 
}
static void LCDReset(void)
{
	LCDWriteCommand(0x33);
	LCDWriteCommand(0x33);
	LCDWriteCommand(0x32);
}
void LCDDisplayInitializing(void)
{
	
	LCDubyte i;
	LCDWriteString("LCD Init: OK");
}
static void  LCDWriteByte(LCDubyte  LCDData)
{
    unsigned char x;

	x = (LCDData & 0xF0) >> 4;
	if (x & 0x01)
		gpio_set_val(70,SET);
  	else
		gpio_set_val(70,CLEAR);
	if (x & 0x02)
		gpio_set_val(71,SET);
	 else
		gpio_set_val(71,CLEAR);
 	if (x & 0x04)
		gpio_set_val(72,SET);
  	else
		gpio_set_val(72,CLEAR);
	if (x & 0x08)
		gpio_set_val(73,SET);
  	else
		gpio_set_val(73,CLEAR);
    	LCDEnable();
    	x = LCDData & 0x0f;
	if (x & 0x01)
		gpio_set_val(70,SET);
  	else
		gpio_set_val(70,CLEAR);
	if (x & 0x02)
		gpio_set_val(71,SET);
	 else
		gpio_set_val(71,CLEAR);
 	if (x & 0x04)
		gpio_set_val(72,SET);
  	else
		gpio_set_val(72,CLEAR);
	if (x & 0x08)
		gpio_set_val(73,SET);
  	else
		gpio_set_val(73,CLEAR);

	LCDEnable();
	udelay(1000);
}

module_init(dev_init);
module_exit(dev_exit);
MODULE_AUTHOR("PRASHANTH BS)");
MODULE_DESCRIPTION("this modules is used for lcd access");
MODULE_LICENSE("GPL");
