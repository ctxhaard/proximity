/*
 * hcsr04-proximity.c - Controller driver for HC-SR04 Ultrasound Proximity Sensor
 * connected to GPIO lines.
 *
 * Copyright (C) 2016 Carlo Tomasin
 *
 * Author: Carlo Tomasin <c.tomasin@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#define DEBUG

#include <linux/module.h>
#include <linux/gpio/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/interrupt.h>

#include <linux/fs.h>    // < file system support
#include <asm/uaccess.h> // < copy from user
#include <linux/cdev.h>  // < character device
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/ktime.h>
#include <linux/time64.h>

#define MODULE_NAME "hcsr04"
#define DEVICE_NAME "proximity"
#define CLASS_NAME DEVICE_NAME
//#define SOUND_SPEED_MS (343)
#define OUT_BUFFER_SIZE (10)
#define MEAS_COUNT (10)

#define SAMPLING_INTERVAL_JIFFIES (HZ / 10)

struct hcsr04_data {
	struct platform_device *pdev;       // < the platform device
	struct gpio_desc       *trigger;    // < the trigger pin

	struct gpio_desc       *echo;       // < the echo pin

	struct pinctrl         *p;          // < reference to pinctrl device
	struct pinctrl_state   *s;          // < the selected state of the pinctrl device
	struct cdev             cdev;       // < character device linked to the actual platform device
	int                     irq;
	struct delayed_work     begin_dwork;
	int                     count;      // < n. of users
	ktime_t                 echo_start_t; // < echo signal risen at...
	char                    out_buffer[OUT_BUFFER_SIZE];
	struct completion       read_done;
	dev_t                   devn;
	long                    meas[MEAS_COUNT];
	size_t                  meas_index;
};

#define MAX_DEVICES_NUM 10

dev_t first;  // < the device MAJOR,MINOR next number
dev_t next;

struct class *proximity_class;

static irqreturn_t hcsr04_echo_interrupt(int irq, void *dev_id) {
	struct hcsr04_data *pdata = dev_id;
	long delta_t_us;
	ktime_t echo_end_t;

	if (gpiod_get_value(pdata->echo)/* rising */) {
		if (ktime_compare(pdata->echo_start_t,ktime_set(KTIME_SEC_MAX,0)) == 0)
			pdata->echo_start_t = ktime_get();
		//printk(KERN_DEBUG "rising\n");
		
	} else if (!gpiod_get_value(pdata->echo)/* falling */) {
		echo_end_t = ktime_get();
		if (ktime_after(echo_end_t,pdata->echo_start_t)) {
			size_t i;
			long mm;

			delta_t_us = ktime_us_delta(echo_end_t,pdata->echo_start_t);
			pdata->meas[pdata->meas_index] = delta_t_us * 10 / 58;
			pdata->meas_index = ((pdata->meas_index + 1) % ARRAY_SIZE(pdata->meas));

			mm = 0;
			for(i = 0; i < ARRAY_SIZE(pdata->meas); ++i) {
				mm += pdata->meas[i];
			}

			mm /= ARRAY_SIZE(pdata->meas);

			memset(pdata->out_buffer,'\0',OUT_BUFFER_SIZE);
			snprintf(pdata->out_buffer,OUT_BUFFER_SIZE,"%li",mm);
			complete(&pdata->read_done);
		}
		pdata->echo_start_t = ktime_set(KTIME_SEC_MAX,0);
	}
	return IRQ_HANDLED;
}

void hcsr04_trigger_pulse(struct gpio_desc *trigger) {

	//printk(KERN_DEBUG "p");
	gpiod_set_value(trigger,1);
	usleep_range(10,15);
	gpiod_set_value(trigger,0);
	//printk(KERN_DEBUG "**p\n");
}

void hcsr04_sample_work(struct work_struct *work) {
	struct hcsr04_data *pdata;
	struct delayed_work *dwork;

	//printk(KERN_DEBUG "begin\n");

	dwork = container_of(work,struct delayed_work,work);
	pdata = container_of(dwork,struct hcsr04_data,begin_dwork);

	hcsr04_trigger_pulse(pdata->trigger);


	// re-schedule the read operation
	if (pdata->count) {
		INIT_DELAYED_WORK(&pdata->begin_dwork,hcsr04_sample_work); // serve???
		schedule_delayed_work(&pdata->begin_dwork,SAMPLING_INTERVAL_JIFFIES);
	} else {
		dev_dbg(&pdata->pdev->dev, "reading stopped\n");
	}
}


int hcsr04_file_open (struct inode *inode, struct file *filp) {

	struct hcsr04_data *pdata; /* device information */
	pdata = container_of(inode->i_cdev, struct hcsr04_data, cdev);
	filp->private_data = pdata; /* for other methods */

	if (pdata->count == 0) {
		// NOTE: IRQ is shared beacuse the same IRQ line is shared between some gpio pins	
		if (IS_ERR_VALUE(request_irq(pdata->irq,hcsr04_echo_interrupt,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_SHARED,"proximity", pdata))) {
			goto err_file_open;
		}
	
		INIT_DELAYED_WORK(&pdata->begin_dwork,hcsr04_sample_work);
		schedule_delayed_work(&pdata->begin_dwork,0);
		dev_dbg( &pdata->pdev->dev, "file open succeeded: scheduled immediate sensor polling\n");
	}
	pdata->count++;
	return 0;

err_file_open:
	dev_err(&pdata->pdev->dev, "can't request echo irq for rising/falling detection\n");
	pdata->count = 0;
	dev_err( &pdata->pdev->dev, "file open error\n");
	return -ENODEV;
}

int hcsr04_file_release (struct inode *inode, struct file *filp) {

	struct hcsr04_data *pdata; /* device information */
	pdata = filp->private_data;

	pdata->count--;
	dev_dbg( &pdata->pdev->dev, "file release\n");
	if (pdata->count == 0) free_irq(pdata->irq, pdata);
	return 0;
}

ssize_t hcsr04_file_read (struct file *filp, char __user *buffer, size_t count, loff_t *off) {

	struct hcsr04_data *pdata; /* device information */
	ssize_t len = 0;

	pdata = filp->private_data;
	//dev_dbg(pdata->dev, "file read\n");
	
	if(pdata) {
		wait_for_completion(&pdata->read_done);

		len = strlen(pdata->out_buffer);
		len = (len < count ? len : count);
		if (len) {
			if (copy_to_user(buffer,pdata->out_buffer,len)) {
				dev_err(&pdata->pdev->dev, "error copying to userspace\n");
				return -EFAULT;
			}
			memmove(pdata->out_buffer,pdata->out_buffer+len,(OUT_BUFFER_SIZE - len));
			reinit_completion(&pdata->read_done);
		}
	}
	//dev_dbg(pdata->dev, "read %d bytes\n",len);
	return len;
}

loff_t hcsr04_file_llseek(struct file *filp, loff_t off, int val) {
	struct hcsr04_data *pdata;

	pdata = filp->private_data;
	dev_dbg(&pdata->pdev->dev, "file llseek\n");
	// TODO: implementare
	
	return 0;
}

static struct file_operations file_ops = {
		.owner = THIS_MODULE,
		.open = hcsr04_file_open,
		.release = hcsr04_file_release,
		.read = hcsr04_file_read,
		.llseek = hcsr04_file_llseek,
};

static const  struct of_device_id of_hcsr04_match[] = {
		{ .compatible = "hcsr04" },
		{},
};

MODULE_DEVICE_TABLE(of,of_hcsr04_match);

static int hcsr04_device_probe(struct platform_device *pdev)
{
	int retval = -1;
	int rc;

	struct hcsr04_data *pdata;

	dev_dbg(&pdev->dev, "Probe\n");

	// allocazione della mia struttura dati nella memoria gestita
	// assieme al device (viene deallocata quando il device scompare)
	pdata = devm_kzalloc(&pdev->dev,sizeof(*pdata),GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->pdev = pdev;

	memset(pdata->meas,0x00,sizeof(pdata->meas));
	pdata->meas_index = 0;
	pdata->echo_start_t = ktime_set(KTIME_SEC_MAX,0);
	
	init_completion(&pdata->read_done);

	pdata->p = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pdata->p)) {
		dev_err(&pdev->dev, "can't get pinctrl handle\n");
		retval = PTR_ERR(pdata->p);
		goto err_probe;
	}

	pdata->s = pinctrl_lookup_state(pdata->p,PINCTRL_STATE_DEFAULT);
	if (IS_ERR(pdata->s)) {
		dev_err(&pdev->dev, "can't get pinctrl selected state\n");
		retval = PTR_ERR(pdata->s);
		goto err_probe;
	}

	if (pinctrl_select_state(pdata->p,pdata->s) < 0) {
		dev_err(&pdev->dev, "can't select pinctrl default state\n");
		goto err_probe;	
	}

	if ( IS_ERR(pdata->trigger = devm_gpiod_get_index(&pdev->dev,"proximity",0,0)) ) {

		dev_err(&pdev->dev, "can't get the TRIGGER gpio descriptor, aborting\n");
		retval = -ENODEV;
		goto err_probe;
	}

	// ricerca del descrittore del GPIO (effettuarla per ECHO e TRIGGER)
	if ( IS_ERR(pdata->echo = devm_gpiod_get_index(&pdev->dev,"proximity",1,0)) ) {
		dev_err(&pdev->dev, "can't get the ECHO gpio descriptor, aborting\n");
		retval = -ENODEV;
		goto err_probe;
	}

	if (0 != gpiod_get_value(pdata->echo)) {
		dev_err(&pdev->dev, "ECHO is HIGH; it should be low, aborting\n");
		retval = -EBUSY;
		goto err_probe;
	}

	// TODO: eventualmente accendere il sensore

	// NOTA: per ora è commentato perchè non funzionava correttamente!!!!
//	// verifico che:
//	// ECHO sia LOW e che a seguito di un impulso su TRIGGER
//	// passi ad HIGH (senza verificarne la durata)
//	hcsr04_trigger_pulse(pdata->trigger);
//
//	counter = 100;
//	do {
//		LOGD("try #%d of 100",(100 - counter));
//
//		echo_val = gpiod_get_value(pdata->echo);
//		if(!echo_val) {
//			usleep_range(100,150);
//		}
//		--counter;
//
//	} while(0 == echo_val && counter > 0);
//
//	if (!echo_val) {
//		LOGE("can't get the echo back, aborting!");
//		return -EBUSY;
//	}
	// -----------------------------------------------------------------

	pdata->irq = gpiod_to_irq(pdata->echo);
	if (IS_ERR_VALUE(pdata->irq)) {
		dev_err(&pdev->dev, "can't get the echo irq\n");
		goto err_probe;
	}

	dev_set_drvdata(&pdev->dev, pdata);

	pdata->devn = next;
	next = MKDEV(MAJOR(next), MINOR(next) + 1);

	if (NULL == device_create(proximity_class, NULL, pdata->devn, NULL, "proximity%d", MINOR(pdata->devn))) {
		retval = -EIO;
		goto err_probe;
	}
	

	cdev_init(&pdata->cdev,&file_ops);
	pdata->cdev.owner = THIS_MODULE;
	pdata->cdev.ops = &file_ops;

	if (IS_ERR_VALUE(rc = cdev_add(&pdata->cdev, pdata->devn, 1))) {
		dev_err(&pdev->dev, "can't add the character device, aborting\n");
		return rc;
	}

	dev_info(&pdev->dev,"major: %d minor: %d\n",MAJOR(pdata->devn), MINOR(pdata->devn));

	retval = 0;

err_probe:
	return retval;
}

static int hcsr04_device_remove(struct platform_device *pdev)
{
	struct hcsr04_data *pdata;

	dev_dbg(&pdev->dev, "device removed\n");

	pdata = (struct hcsr04_data *)dev_get_drvdata(&pdev->dev);

	pdata->count = 0;
	cancel_delayed_work_sync(&pdata->begin_dwork);
	if (proximity_class) {
		device_destroy(proximity_class, pdata->devn);
	}
	cdev_del(&pdata->cdev);
	// TODO: eventualmente spegnere il sensore
	return 0;
}


static const struct of_device_id hcsr04_match[] = {
		{
				.compatible = "hcsr04",
		},
		{},
};

// non so a cosa serva questo
MODULE_DEVICE_TABLE(of, hcsr04_match);

static struct platform_driver hcsr04_platform_driver = {
		.driver = {
				.name = MODULE_NAME,
				.owner = THIS_MODULE,
				// questo, in abbinata al DTS (o all'overlay) permette che sia chiamato
				// il metodo probe()
				.of_match_table = of_match_ptr(hcsr04_match),
		},
		.probe = hcsr04_device_probe,
		.remove = hcsr04_device_remove,
};


static int __init hcsr04_module_init(void)
{
	int rc;
	dev_t devn;

	printk(KERN_DEBUG "module init\n");

	if(IS_ERR_VALUE(rc = alloc_chrdev_region(&devn,0,MAX_DEVICES_NUM,DEVICE_NAME))) {
		printk(KERN_ERR "failed to alloc the chardev region\n");
		return rc;
	}
	first = devn;
	next = first;

	proximity_class = class_create(THIS_MODULE, CLASS_NAME);
	if(IS_ERR(proximity_class)) {

		printk(KERN_ERR "cannot create sysfs device class\n");
		return PTR_ERR(proximity_class);
	}

	return platform_driver_register(&hcsr04_platform_driver);
}

static void __exit hcsr04_module_exit(void)
{
	printk(KERN_DEBUG "module exit\n");

	platform_driver_unregister(&hcsr04_platform_driver);
	if (proximity_class)
		class_destroy(proximity_class);
	unregister_chrdev_region(first,MAX_DEVICES_NUM);
}

// incrementare il log level a 8 con
// # echo 8 > /proc/sys/kernel/printk
module_init(hcsr04_module_init);
module_exit(hcsr04_module_exit);
//module_platform_driver(hcsr04_platform_driver);

MODULE_AUTHOR("Carlo Tomasin <c.tomasin@gmail.com>");
MODULE_DESCRIPTION("Device driver for HC-SR04 Ultrasound Proximity Sensor");
MODULE_LICENSE("GPL");
