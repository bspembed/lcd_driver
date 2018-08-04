/* lcd_driver.c */

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
#include "gpio_table.h"
#include "ioctl_magic.h"

#define DEVICE_NAME "new_lcd"

#define SET_ONE		1
#define CLEAR_ZERO	0
#define MAX_SIZE	17

#define LCD_Clear() LCDWriteCommand(0x01)	
#define LCD_Row1()  LCDWriteCommand(0x80)	
#define LCD_Row2()  LCDWriteCommand(0xC0)   	

typedef unsigned char LCDubyte; 

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

int open_board_open (struct inode *inode, struct file *file)
{
	PRINT(KERN_INFO"GPIO OPEN\n");
	return 0;
}

int open_board_close (struct inode *inode, struct file *file)
{
	PRINT(KERN_INFO"GPIO CLOSE\n");
	return 0;
}

irqreturn_t gpio_irq_handler(int irq, void *dev) 
{
	int device = (int)dev;
	value[device]^=1;
	return IRQ_HANDLED;
}

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
			PRINT("GPIO IRQ = %lu FAIL\n",arg);
		else
			PRINT("GPIO IRQ = %lu SUCCESS\n",arg);
		break;
	case IOCTL_GPIO_REQ:
		if(gpio_request(arg, "GPIO DRIVER"))
			PRINT(KERN_WARNING "GPIO FAILED %lu\n",arg);
		else
			PRINT("GPIO CMD = %d\t ARG = %lu\n",IOCTL_GPIO_REQ,arg);
		break;
	case IOCTL_DIR_IN:
			gpio_direction_input(gpio_table[arg]);
			PRINT("GPIO DIRECTION = %d\t ARG = %lu\n",IOCTL_DIR_IN,arg);
		break;
	case IOCTL_REQUEST_IRQ:
		if (request_irq(gpio_to_irq(arg),gpio_irq_handler, IRQ_TYPE_EDGE_RISING, "GPIO IEQ HANDLED", (void*)arg))
			PRINT("GPIO IRQ FAILED\n");
		else
			PRINT("GPIO IEQ SUCCESS\n");
		break;       
	case IOCTL_DIR_OUT:
			gpio_direction_output(gpio_table[arg], 1);
			PRINT("\nGPIO DIRECTION = %d\t ARG = %lu\n", IOCTL_DIR_OUT,arg);
		break;
	case IOCTL_SET_GPIO:
			PRINT("GPIO SET = %d\t ARG = %lu\n", IOCTL_SET_GPIO,arg);
			gpio_set_value(gpio_table[arg], 1);
		break;
	case IOCTL_CLR_GPIO:
			PRINT("GPIO CLEAR = %d\t ARG = %lu\n", IOCTL_CLR_GPIO,arg);
			gpio_set_value(gpio_table[arg], 0);
		break;
	case IOCTL_FREE_GPIO:
			gpio_free(arg);
			PRINT("GPIO FREE = %d\t ARG = %lu\n", IOCTL_FREE_GPIO,arg);
		break;
	case IOCTL_FREE_IRQ:
			free_irq(gpio_to_irq(arg),(void*)arg);
		break;
	default:
		return -EINVAL;
	}
	return 0;	
}

ssize_t open_board_read(struct file *file, char __user *usr, size_t ret, loff_t *off)
{
	ret=copy_to_user(usr, value, ret);
	return ret;
}

static ssize_t open_board_write(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	if(count < MAXSIZE) 
	{
		copy_from_user(d_buf, buf, count);
		d_buf[count-1] = 0;
		LCDClear();
		LCDWriteString(d_buf);
		*f_pos += count;
		return count;
	}
	else
	{
		copy_from_user(d_buf, buf, MAXSIZE - 1);
		d_buf[MAXSIZE - 1] = 0;
		LCDClear();
		LCDWriteString(d_buf);
		*f_pos += MAXSIZE - 1;
		return MAXSIZE - 1;
	}
}

static struct file_operations dev_fops = 
{
	.owner	        	= THIS_MODULE,
	.open           	= open_board_open,
	.release        	= open_board_close,
	.unlocked_ioctl		= open_board_ioctl,
	.read           	= open_board_read,
	.write				= open_board_write,
};

static struct miscdevice misc = 
{
	.minor 		= MISC_DYNAMIC_MINOR,
	.name 		= DEVICE_NAME,
	.fops 		= &dev_fops,
};

static int __init dev_init(void)
{
	int ret;

	ret = misc_register(&misc);
	PRINT(KERN_INFO DEVICE_NAME"\tINIT\n");
  		
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
	if(gpio_request(arg, "Gpio DRIVER"))
		PRINT(KERN_WARNING "GPIO REQUEST FAILED %lu\n", arg);
	else
		PRINT("GPIO REQUEST CMD = %d\t ARG = %lu\n", IOCTL_GPIO_REQ,arg);
	gpio_direction_output(arg, 1);

}

static void gpio_set_val(unsigned int arg, int val)
{
	gpio_set_value(gpio_table[arg], val);
}

static void  LCDEnable(void)
{
    gpio_set_val(75, SET);
	udelay(50);
    gpio_set_val(75, CLEAR);
}
	
void LCDWriteCommand(LCDubyte LCDData)
{
	gpio_set_val(74, CLEAR); 
	LCDWriteByte(LCDData);
 }

void LCDWriteData(LCDubyte LCDData)
{
   	gpio_set_val(74, SET); 
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
