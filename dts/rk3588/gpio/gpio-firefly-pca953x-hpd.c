/*
 * Driver for pwm demo on Firefly board.
 *
 * Copyright (C) 2016, Zhongshan T-chip Intelligent Technology Co.,ltd.
 * Copyright 2006  JC.Lin
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include "gpio-firefly-pca953x-hpd.h"

int firefly_pca953x_is_inserted(struct platform_device *pdev, bool *is_inserted) 
{
    struct firefly_pca953x_hpd_info *info;
    int gpio_value;

    if (pdev == NULL) {
        return -1;
    }

    info = platform_get_drvdata(pdev);
    if (info == NULL) {
        return -1;
    }

    gpio_value = gpio_get_value(info->det_gpio);
    if (gpio_value == info->flag) {
        *is_inserted = true;
    } else {
        *is_inserted = false;
    }
    
    return 0;
}

// static irqreturn_t firefly_pca953x_det_irq(int irq, void *dev_id)
// {
//     int ret;
//     bool is_inserted;
//     struct firefly_pca953x_hpd_info *info = (struct firefly_pca953x_hpd_info *)dev_id;
//     printk("firefly_pca953x_det_irq program!\n");


//     ret = firefly_pca953x_is_inserted(info->pdev, &is_inserted);
//     if (ret) {
//         printk("firefly_pca953x_is_inserted fail!\n");
//     } else {
//         if (is_inserted) {
//             printk("抽屉板插入!\n");
//         } else {
//             printk("抽屉板拔出!\n");
//         }
//     }

//     return IRQ_HANDLED;
// }

static int firefly_pca953x_hpd_probe(struct platform_device *pdev)
{
    // int ret;
    int gpio;
    enum of_gpio_flags flag;
    struct firefly_pca953x_hpd_info *info;
    struct device_node *node = pdev->dev.of_node;

    info = devm_kzalloc(&pdev->dev,sizeof(struct firefly_pca953x_hpd_info), GFP_KERNEL);
    if (!info) {
        dev_err(&pdev->dev, "devm_kzalloc failed!\n");
        return -ENOMEM;
    }

    info->pdev = pdev;

    gpio = of_get_named_gpio_flags(node, "det-gpio", 0, &flag);
    if (!gpio_is_valid(gpio))
    {
        dev_err(&pdev->dev, "det-gpio: %d is invalid\n", gpio);
        return -ENODEV;
    }
    if (gpio_request(gpio, "det-gpio"))
    {
        dev_err(&pdev->dev, "det-gpio: %d request failed!\n", gpio);
        gpio_free(gpio);
        return -ENODEV;
    }
    info->det_gpio = gpio;
    info->flag = (flag == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
    gpio_direction_input(info->det_gpio);

    info->det_irq = gpio_to_irq(info->det_gpio);
    if (info->det_irq < 0) {
        dev_err(&pdev->dev, "det-gpio: %d request irq failed!\n", gpio);
        gpio_free(gpio);
        return -ENODEV;
    }

    // ret = request_irq(info->det_irq, firefly_pca953x_det_irq,  IRQ_TYPE_EDGE_BOTH | IRQF_SHARED , "pca953x-det-gpio", info);
    // if (ret != 0) {
    //     free_irq(info->det_irq, info);
    //     dev_err(&pdev->dev, "Failed to request IRQ: %d\n", ret);
    //     return -ENODEV;
    // }

    platform_set_drvdata(pdev,info);
    printk("Firefly pca953x hpd probe finish \n");
    return 0;
}

static struct of_device_id firefly_pca953x_hpd_match_table[] = {
    { .compatible = "firefly,pca9555-hpd",},
    {},
};

static struct platform_driver firefly_pca953x_hpd_driver = {
    .driver = {
        .name = "firefly-pca953x-hpd",
        .owner = THIS_MODULE,
        .of_match_table = firefly_pca953x_hpd_match_table,
    },
    .probe = firefly_pca953x_hpd_probe,
};

static int firefly_pca953x_hpd_init(void)
{
    int ret;
    printk("Firefly pca953x hpd driver init!\n");
    ret = platform_driver_register(&firefly_pca953x_hpd_driver);
    printk("Firefly pca953x hpd driver register return : %d \n",ret);
    return ret ;
}
module_init(firefly_pca953x_hpd_init);

static void firefly_pca953x_hpd_exit(void)
{
        platform_driver_unregister(&firefly_pca953x_hpd_driver);
}
module_exit(firefly_pca953x_hpd_exit);

MODULE_AUTHOR("maocl <service@t-firefly.com>");
MODULE_DESCRIPTION("Firefly PCA953X Hot Plug GPIO Detect driver");
MODULE_ALIAS("platform:firefly-gpio");
MODULE_LICENSE("GPL");
