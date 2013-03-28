#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/usb/otg.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <plat/platform.h>
#include <plat/plat_dev.h>
#include <plat/lm.h>
#include <mach/clock.h>
#include <mach/map.h>
#include <mach/gpio.h>
#include <mach/gpio_data.h>
#include <linux/delay.h>
#include <plat/regops.h>
#include <mach/reg_addr.h>



#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
static struct early_suspend ledlogo_early_suspend;
#endif

static int g_ledlogoEnableFlag ;
static struct device *dev;
#ifdef CONFIG_HAS_EARLYSUSPEND
static int aml_ledlogo_suspend(struct early_suspend *handler);
static int aml_ledlogo_resume(struct early_suspend *handler);
#else

#define aml_ledlogo_suspend NULL
#define aml_ledlogo_resume NULL

#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static int aml_ledlogo_suspend(struct early_suspend *handler)
{
   // printk("aml_ledlogo_suspend !!!!!!!!!!!!!###################\n");
  
    if(g_ledlogoEnableFlag) 
    {
       // printk("do aml_ledlogo_suspend !!!!!!!!!!!!!###################\n");
	gpio_out(PAD_GPIOD_3, 0);  
    }

    return 0 ;
}

static int aml_ledlogo_resume(struct early_suspend *handler)
{
   // printk("aml_ledlogo_resume !!!!!!!!!!!!!###################\n");
 
    if(g_ledlogoEnableFlag)
    {
       // printk("do aml_ledlogo_resume !!!!!!!!!!!!!###################\n");
    	
    	gpio_out(PAD_GPIOD_3, 1);  
    }
 
    return 0 ;
}
#endif

static ssize_t get_ledlogoEnableFlag(struct class *cla, struct class_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", g_ledlogoEnableFlag ); 
}

static ssize_t set_ledlogoEnableFlag(struct class *cla, struct class_attribute *attr, char *buf, size_t count)
{
    if(!strlen(buf)){
       // printk("set_ledlogoEnableFlag 1==%d\n",g_ledlogoEnableFlag);
        return 0;
    }
    g_ledlogoEnableFlag = (int)(buf[0]-'0');
    g_ledlogoEnableFlag = !!g_ledlogoEnableFlag ;
	//printk("set_ledlogoEnableFlag 2==%d\n",g_ledlogoEnableFlag);
    if(g_ledlogoEnableFlag){
	gpio_out(PAD_GPIOD_3, 1); 
    }else{
    	gpio_out(PAD_GPIOD_3, 0); 
    }

    return count;
}

static struct class_attribute Amlledlogo_class_attrs[] = {
    __ATTR(enable,S_IRUGO|S_IWUGO,get_ledlogoEnableFlag,set_ledlogoEnableFlag),
    __ATTR_NULL
};
static struct class Amlledlogo_class = {
    .name = "aml_ledlogo",
    .class_attrs = Amlledlogo_class_attrs,
};


static int __init aml_ledlogo_probe(struct platform_device *pdev)
{
    int ret = 0;
    printk(" aml_ledlogo probe  \n");
	
    if (pdev->id == -1) {
    	dev_err(dev, "it's meaningless to register several "
    		"pda_powers; use id = -1\n");
    	ret = -EINVAL;
    	goto exit;
    }
	// printk(" !!!!!!!!!!!g_ledlogoEnableFlag=%d\n",g_ledlogoEnableFlag);
   /* if(g_ledlogoEnableFlag){
	gpio_out(PAD_GPIOD_3, 1); 
	    printk(" aml_ledlogo on ##############\n");
    }*/
    
    #ifdef CONFIG_HAS_EARLYSUSPEND
        ledlogo_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN-1;
        ledlogo_early_suspend.suspend = aml_ledlogo_suspend;
        ledlogo_early_suspend.resume = aml_ledlogo_resume;
        ledlogo_early_suspend.param = (void *)&g_ledlogoEnableFlag;
        register_early_suspend(&ledlogo_early_suspend);
    #endif
    
    return 0 ;
    
exit:;	
    return ret;
}

static int aml_ledlogo_remove(struct platform_device *pdev)
{
    int ret = 0;

    printk(" aml_modem_remove \n");

    dev = &pdev->dev;

    if (pdev->id != -1) {
        dev_err(dev, "it's meaningless to register several "
        	"pda_powers; use id = -1\n");
        ret = -EINVAL;
        goto exit;
    }

    #ifdef CONFIG_HAS_EARLYSUSPEND
        unregister_early_suspend(&ledlogo_early_suspend);
    #endif
    
exit:;	
	return ret;
}


MODULE_ALIAS("platform:aml-ledlogo");

static struct platform_driver aml_ledlogo_pdrv = {
    .driver = {
        .name = "aml-ledlogo",
        .owner = THIS_MODULE,	
    },
    .probe = aml_ledlogo_probe,
    .remove = aml_ledlogo_remove,
//    .suspend = aml_modem_suspend,
 //   .resume = aml_modem_resume,
};

static int __init aml_ledlogo_init(void)
{
	printk("ledlogo init \n");
       class_register(&Amlledlogo_class);
	return platform_driver_register(&aml_ledlogo_pdrv);
}

static void __exit aml_ledlogo_exit(void)
{
      platform_driver_unregister(&aml_ledlogo_pdrv);
      class_unregister(&Amlledlogo_class);
}

module_init(aml_ledlogo_init);
module_exit(aml_ledlogo_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alex Deng");



