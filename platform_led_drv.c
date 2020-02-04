#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/arch/regs-gpio.h>
#include <asm/hardware.h>

/* platform_device s3c24xx_led defined in linux-2.6.22.6_jz_drv/arch/arm/plat-s3c24xx/common-smdk2440.c */

volatile unsigned long *gpfcon = NULL;
volatile unsigned long *gpfdat = NULL;

volatile unsigned long *gpgcon = NULL;
volatile unsigned long *gpgdat = NULL;

struct platform_led_dev {
	struct cdev cdev;
	unsigned int flag;
	struct mutex mutex;
	wait_queue_head_t r_wait;
//	wait_queue_head_t w_wait;
//	struct fasync_struct *async_queue;
	struct miscdevice miscdev;
};

struct platform_led_dev *pld;


static int platform_led_open(struct inode *inode, struct file *filp)
{
	filp->private_data = pld;

	printk("platform_led_open\n");
	/* 配置GPF4,5,6为输出 中文 */
	//*gpfcon &= ~((0x3<<(4*2)) | (0x3<<(5*2)) | (0x3<<(6*2)));
	//*gpfcon |= ((0x1<<(4*2)) | (0x1<<(5*2)) | (0x1<<(6*2)));
	//request_irq(IRQ_EINT0,  buttons_irq, IRQT_BOTHEDGE, "S2", &pins_desc[0]);
	//request_irq(IRQ_EINT2,  buttons_irq, IRQT_BOTHEDGE, "S3", &pins_desc[1]);
	//request_irq(IRQ_EINT11, buttons_irq, IRQT_BOTHEDGE, "S4", &pins_desc[0]);
	//request_irq(IRQ_EINT19, buttons_irq, IRQT_BOTHEDGE, "S5", &pins_desc[1]);
	printk("platform_led_open ok\n");
	return 0;
}

static int platform_led_poll(struct file *filp, poll_table * wait)
{
	unsigned int mask =0;

	struct platform_led_dev *dev = filp->private_data;

	mutex_lock(&dev->mutex);

	poll_wait(filp, &dev->r_wait, wait);

	if (dev->flag == 1) {
		mask |= POLLIN | POLLRDNORM;
		printk(KERN_INFO "mask is : %d\n", mask);
	}
	
	if (dev->flag == 0) {
		mask &= ~(POLL_IN | POLLRDNORM);
		printk(KERN_INFO "mask is : %d\n", mask);
	}

	mutex_unlock(&dev->mutex);
	return mask;
}

static ssize_t platform_led_write(struct file *filp, const char __user *buf, size_t count, loff_t * ppos)
{
	int val;
	//struct platform_led_dev *dev = container_of(filp->private_data, struct platform_led_dev, miscdev);
	struct platform_led_dev *dev = filp->private_data;
	printk(KERN_INFO "dev address:%d\n", dev);
	printk(KERN_INFO "pld address:%d\n", pld);
	
	int ret;
	 
//	DECLARE_WAITQUEUE(wait, current);
/*  
	mutex_lock(&dev->mutex);
	add_wait_queue(&dev->w_wait, &wait);

	//printk("first_drv_write\n");

	while (dev->flag == 0) {
		if (filp->f_flags & O_NONBLOCK){
			ret = -EAGAIN;
			goto out;
		}

		__set_current_state(TASK_INTERRUPTIBLE);
		
		mutex_unlock(&dev->mutex);

		schedule();

		if (signal_pending(current)){
			ret = -ERESTARTSYS;
			goto out2;
		}

		mutex_lock(&dev->mutex);
	}
*/
	printk(KERN_INFO "copy_from_user begin\n");
	copy_from_user(&val, buf, count); //	copy_to_user();
	printk(KERN_INFO "copy_from_user end\n");

	if (val == 11)
	{
		s3c2410_gpio_setpin(S3C2410_GPF4, 0);
		dev->flag = 1;
		//wake_up_interruptible(&dev->r_wait);
	}
	else if (val == 21)
	{
		s3c2410_gpio_setpin(S3C2410_GPF5, 0);
		dev->flag = 1;
		//wake_up_interruptible(&dev->r_wait);
	}
	else if (val == 31)
	{
		s3c2410_gpio_setpin(S3C2410_GPF6, 0);
		dev->flag = 1;
		//wake_up_interruptible(&dev->r_wait);
	}
	else if (val == 41)
	{
		s3c2410_gpio_setpin(S3C2410_GPF7, 0);
		dev->flag = 1;
		//wake_up_interruptible(&dev->r_wait);
	}

	else if(val == 10)
	{
		//*gpfdat |= (1<<4) | (1<<5) | (1<<6);
		s3c2410_gpio_setpin(S3C2410_GPF4, 1);
		dev->flag = 0;
	}
	else if(val == 20)
	{
		//*gpfdat |= (1<<4) | (1<<5) | (1<<6);
		s3c2410_gpio_setpin(S3C2410_GPF5, 1);
		dev->flag = 0;
	}
	else if(val == 30)
	{
		//*gpfdat |= (1<<4) | (1<<5) | (1<<6);
		s3c2410_gpio_setpin(S3C2410_GPF6, 1);
		dev->flag = 0;
	}
	else if(val == 40)
	{
		//*gpfdat |= (1<<4) | (1<<5) | (1<<6);
		s3c2410_gpio_setpin(S3C2410_GPF7, 1);
		dev->flag = 0;
	}
	
	//wake_up_interruptible(&dev->r_wait);
/*  
	if (dev->async_queue){
		kill_fasync(&dev->async_queue, SIGIO, POLL_IN);
		printk(KERN_DEBUG "%s kill SIGIO\n", __func__);
	}
*/
out:
	//mutex_unlock(&dev->mutex);
//out2:
//	remove_wait_queue(&dev->w_wait, &wait);
//	set_current_state(TASK_RUNNING);
	return 0;
}

static ssize_t platform_led_read(struct file *filp, char __user *buf, size_t count, loff_t * ppos)
{
	int val;
	//struct platform_led_dev *dev = container_of(filp->private_data, struct platform_led_dev, miscdev);
	struct platform_led_dev *dev = filp->private_data;

	int ret;
	
	
	DECLARE_WAITQUEUE(wait, current);

	mutex_lock(&dev->mutex);
	add_wait_queue(&dev->r_wait, &wait);

	//printk("first_drv_write\n");
  
	while (dev->flag == 0) {
	// while (*gpfdat & ((1<<4) | (1<<5) | (1<<6))) {
		if (filp->f_flags & O_NONBLOCK){
			ret = -EAGAIN;
			goto out;
		}

		__set_current_state(TASK_INTERRUPTIBLE);
		
		mutex_unlock(&dev->mutex);

		schedule();

		if (signal_pending(current)){
			ret = -ERESTARTSYS;
			goto out2;
		}

		mutex_lock(&dev->mutex);
	}
	
	/*
	if (*gpfdat & ((1<<4) | (1<<5) | (1<<6)))
	{
		// 点灯
		val = 0;
		printk(KERN_INFO "\nval = %d\n", val);
	}
	else
	{
		// 灭灯
		val = 1;
		printk(KERN_INFO "\nval = %d\n", val);
	}
	*/

	if (count != 1)
		return -EINVAL;

	//wait_event_interruptible(button_waitq, ev_press);

	printk(KERN_INFO "copy_to_user begin\n");
	//copy_to_user(buf, &key_state, count); //	copy_to_user();
	printk(KERN_INFO "copy_to_user end\n");
	//wake_up_interruptible(&dev->r_wait);
	//ev_press = 0;
/*  
	if (dev->async_queue){
		kill_fasync(&dev->async_queue, SIGIO, POLL_IN);
		printk(KERN_DEBUG "%s kill SIGIO\n", __func__);
	}
*/
out:
	mutex_unlock(&dev->mutex);
out2:
	remove_wait_queue(&dev->r_wait, &wait);
	set_current_state(TASK_RUNNING);
	return 0;
//	return 1;
}


static struct file_operations platform_led_fops = {
	.owner 		= THIS_MODULE,
	.open  		= platform_led_open,
	//.read  		= platform_led_read,
	.write 		= platform_led_write,
	.poll  		= platform_led_poll,
	//.release 	= platform_led_release,
	
};

static int platform_led_probe(struct platform_device *pdev)
{
	
	//struct platform_led_dev *pl;
	int ret;

	pld = devm_kzalloc(&pdev->dev, sizeof(*pld), GFP_KERNEL);
	if(!pld)
		return -ENOMEM;
	pld->miscdev.minor = MISC_DYNAMIC_MINOR;
	pld->miscdev.name = "s3c24xx_led";
	pld->miscdev.fops = &platform_led_fops;

	mutex_init(&pld->mutex);
//	init_waitqueue_head(&pld->r_wait);
//	init_waitqueue_head(&pld->w_wait);
	platform_set_drvdata(pdev, pld);

	ret = misc_register(&pld->miscdev);
	if (ret < 0)
		goto err;

	//gpfcon = (volatile unsigned long *)ioremap(0x56000050, 16);
	//gpfdat = gpfcon + 1;
	
	//gpgcon = (volatile unsigned long *)ioremap(0x56000060, 16);
	//gpgdat = gpgcon + 1;
	
	dev_info(&pdev->dev, "s3c24xx_led.0 drv probed\n");

	return 0;
err:
	return ret;
}

static int platform_led_remove(struct platform_device *pdev)
{
	struct platform_led_dev *pl = platform_get_drvdata(pdev);

	misc_deregister(&pl->miscdev);

	dev_info(&pdev->dev, "platform_led dev removed\n");

	//iounmap(gpfcon);
	//iounmap(gpgcon);

	return 0;
}

static struct platform_driver platform_led_driver = {
	.driver = {
		.name = "s3c24xx_led",
		.bus = &platform_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = platform_led_probe,
	.remove = platform_led_remove,
};

static int __init platform_led_init(void)
{
	return platform_driver_register(&platform_led_driver);
}
module_init(platform_led_init);

static int __exit platform_led_exit(void)
{
	platform_driver_unregister(&platform_led_driver);
}
module_exit(platform_led_exit);


MODULE_AUTHOR("WL");
MODULE_LICENSE("GPL");
