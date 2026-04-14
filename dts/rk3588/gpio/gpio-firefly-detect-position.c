#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h> /* for enum gpiod_flags */
#include <linux/sysfs.h>


struct firefly_posistion_info {
    int  position[7];
    int  gpios[7];
    int  used_num;

    int  sub_position[7];
    int  sub_gpios[7];
    int  sub_used_num;

    bool sub_board_exist;
};

static struct firefly_posistion_info *info;



static int firefly_parse_dt_sub(struct platform_device *pdev,struct firefly_posistion_info *info)
{
    int sub_used,gpio,ret;
    enum of_gpio_flags flag;
    struct device_node *node = pdev->dev.of_node;

    ret = of_property_read_u32(node,"firefly-sub-used-num",&sub_used);
    if (ret != 0) {
        dev_err(&pdev->dev, "get property firefly-sub-used-num fail: %d\n",ret);
        return ret;
    }

    switch (sub_used) {
        case 7:
            gpio = of_get_named_gpio_flags(node, "firefly-sub-bit6-gpio", 0, &flag);
            if (!gpio_is_valid(gpio))
            {
                dev_err(&pdev->dev, "firefly-sub-bit6-gpio: %d is invalid\n", gpio);
                goto err;
            }
            info->sub_gpios[6] = gpio;
        case 6:
            gpio = of_get_named_gpio_flags(node, "firefly-sub-bit5-gpio", 0, &flag);
            if (!gpio_is_valid(gpio))
            {
                dev_err(&pdev->dev, "firefly-sub-bit5-gpio: %d is invalid\n", gpio);
                goto err;
            }
            info->sub_gpios[5] = gpio;
        case 5:
            gpio = of_get_named_gpio_flags(node, "firefly-sub-bit4-gpio", 0, &flag);
            if (!gpio_is_valid(gpio))
            {
                dev_err(&pdev->dev, "firefly-sub-bit4-gpio: %d is invalid\n", gpio);
                goto err;
            }
            info->sub_gpios[4] = gpio;
        case 4:
            gpio = of_get_named_gpio_flags(node, "firefly-sub-bit3-gpio", 0, &flag);
            if (!gpio_is_valid(gpio))
            {
                dev_err(&pdev->dev, "firefly-sub-bit3-gpio: %d is invalid\n", gpio);
                goto err;
            }
            info->sub_gpios[3] = gpio;
        case 3:
            gpio = of_get_named_gpio_flags(node, "firefly-sub-bit2-gpio", 0, &flag);
            if (!gpio_is_valid(gpio))
            {
                dev_err(&pdev->dev, "firefly-sub-bit2-gpio: %d is invalid\n", gpio);
                goto err;
            }
            info->sub_gpios[2] = gpio;
        case 2:
            gpio = of_get_named_gpio_flags(node, "firefly-sub-bit1-gpio", 0, &flag);
            if (!gpio_is_valid(gpio))
            {
                dev_err(&pdev->dev, "firefly-sub-bit1-gpio: %d is invalid\n", gpio);
                goto err;
            }
            info->sub_gpios[1] = gpio;
        case 1:
            gpio = of_get_named_gpio_flags(node, "firefly-sub-bit0-gpio", 0, &flag);
            if (!gpio_is_valid(gpio))
            {
                dev_err(&pdev->dev, "firefly-sub-bit0-gpio: %d is invalid\n", gpio);
                goto err;
            }
            info->sub_gpios[0] = gpio;
            break;
        default:
            dev_err(&pdev->dev, "property firefly-sub-used-num is invalid\n");
            goto err;
    }

    info->sub_used_num = sub_used;
    return 0;

err:
    return -1;
}


static int firefly_parse_dt(struct platform_device *pdev,struct firefly_posistion_info *info)
{
    int used,gpio,ret;
    enum of_gpio_flags flag;
    struct device_node *node = pdev->dev.of_node;

    ret = of_property_read_u32(node,"firefly-used-num",&used);
    if (ret != 0) {
        dev_err(&pdev->dev, "get property firefly-used-num fail: %d\n",ret);
        return ret;
    }

    switch (used) {
        case 7:
            gpio = of_get_named_gpio_flags(node, "firefly-bit6-gpio", 0, &flag);
            if (!gpio_is_valid(gpio))
            {
                dev_err(&pdev->dev, "firefly-bit6-gpio: %d is invalid\n", gpio);
                goto err;
            }
            info->gpios[6] = gpio;
        case 6:
            gpio = of_get_named_gpio_flags(node, "firefly-bit5-gpio", 0, &flag);
            if (!gpio_is_valid(gpio))
            {
                dev_err(&pdev->dev, "firefly-bit5-gpio: %d is invalid\n", gpio);
                goto err;
            }
            info->gpios[5] = gpio;
        case 5:
            gpio = of_get_named_gpio_flags(node, "firefly-bit4-gpio", 0, &flag);
            if (!gpio_is_valid(gpio))
            {
                dev_err(&pdev->dev, "firefly-bit4-gpio: %d is invalid\n", gpio);
                goto err;
            }
            info->gpios[4] = gpio;
        case 4:
            gpio = of_get_named_gpio_flags(node, "firefly-bit3-gpio", 0, &flag);
            if (!gpio_is_valid(gpio))
            {
                dev_err(&pdev->dev, "firefly-bit3-gpio: %d is invalid\n", gpio);
                goto err;
            }
            info->gpios[3] = gpio;
        case 3:
            gpio = of_get_named_gpio_flags(node, "firefly-bit2-gpio", 0, &flag);
            if (!gpio_is_valid(gpio))
            {
                dev_err(&pdev->dev, "firefly-bit2-gpio: %d is invalid\n", gpio);
                goto err;
            }
            info->gpios[2] = gpio;
        case 2:
            gpio = of_get_named_gpio_flags(node, "firefly-bit1-gpio", 0, &flag);
            if (!gpio_is_valid(gpio))
            {
                dev_err(&pdev->dev, "firefly-bit1-gpio: %d is invalid\n", gpio);
                goto err;
            }
            info->gpios[1] = gpio;
        case 1:
            gpio = of_get_named_gpio_flags(node, "firefly-bit0-gpio", 0, &flag);
            if (!gpio_is_valid(gpio))
            {
                dev_err(&pdev->dev, "firefly-bit0-gpio: %d is invalid\n", gpio);
                goto err;
            }
            info->gpios[0] = gpio;
            break;
        default:
            dev_err(&pdev->dev, "property firefly-used-num is invalid\n");
            goto err;
    }

    info->used_num = used;
    return 0;

err:
    return -1;
}


static int firefly_set_sub_gpio(struct platform_device *pdev,struct firefly_posistion_info *info)
{
    switch (info->sub_used_num) {
        case 7:
            if (gpio_request(info->sub_gpios[6], "detect-sub-position-6"))
            {
                dev_err(&pdev->dev, "sub-detect-position-6: %d request failed!\n", info->sub_gpios[6]);
                goto err_6;
            }
            gpio_direction_input(info->sub_gpios[6]);
        case 6:
            if (gpio_request(info->sub_gpios[5], "sub-detect-sub-position-5"))
            {
                dev_err(&pdev->dev, "sub-detect-position-5: %d request failed!\n", info->sub_gpios[5]);
                goto err_5;
            }
            gpio_direction_input(info->sub_gpios[5]);
        case 5:
            if (gpio_request(info->sub_gpios[4], "detect-sub-position-4"))
            {
                dev_err(&pdev->dev, "sub-detect-position-4: %d request failed!\n", info->sub_gpios[4]);
                goto err_4;
            }
            gpio_direction_input(info->sub_gpios[4]);
        case 4:
            if (gpio_request(info->sub_gpios[3], "detect-sub-position-3"))
            {
                dev_err(&pdev->dev, "sub-detect-position-3: %d request failed!\n", info->sub_gpios[3]);
                goto err_3;
            }
            gpio_direction_input(info->sub_gpios[3]);
        case 3:
            if (gpio_request(info->sub_gpios[2], "detect-sub-position-2"))
            {
                dev_err(&pdev->dev, "sub-detect-position-2: %d request failed!\n", info->sub_gpios[2]);
                goto err_2;
            }
            gpio_direction_input(info->sub_gpios[2]);
        case 2:
            if (gpio_request(info->sub_gpios[1], "detect-sub-position-1"))
            {
                dev_err(&pdev->dev, "sub-detect-position-1: %d request failed!\n", info->sub_gpios[1]);
                goto err_1;
            }
            gpio_direction_input(info->sub_gpios[1]);
        case 1:
            if (gpio_request(info->sub_gpios[0], "detect-sub-position-0"))
            {
                dev_err(&pdev->dev, "sub-detect-position-0: %d request failed!\n", info->sub_gpios[0]);
                goto err_0;
            }
            gpio_direction_input(info->sub_gpios[0]);
            break;
        default:
            dev_err(&pdev->dev, "property firefly-sub-used-num is invalid\n");
            goto err;
    }

    return 0;

err_0:
    gpio_free(info->sub_gpios[0]);
err_1:
    gpio_free(info->sub_gpios[1]);
err_2:
    gpio_free(info->sub_gpios[2]);
err_3:
    gpio_free(info->sub_gpios[3]);
err_4:
    gpio_free(info->sub_gpios[4]);
err_5:
    gpio_free(info->sub_gpios[5]);
err_6:
    gpio_free(info->sub_gpios[6]);
err:
    return -1;
}

static int firefly_set_gpio(struct platform_device *pdev,struct firefly_posistion_info *info)
{
    switch (info->used_num) {
        case 7:
            if (gpio_request(info->gpios[6], "detect-position-6"))
            {
                dev_err(&pdev->dev, "detect-position-6: %d request failed!\n", info->gpios[6]);
                goto err_6;
            }
            gpio_direction_input(info->gpios[6]);
        case 6:
            if (gpio_request(info->gpios[5], "detect-position-5"))
            {
                dev_err(&pdev->dev, "detect-position-5: %d request failed!\n", info->gpios[5]);
                goto err_5;
            }
            gpio_direction_input(info->gpios[5]);
        case 5:
            if (gpio_request(info->gpios[4], "detect-position-4"))
            {
                dev_err(&pdev->dev, "detect-position-4: %d request failed!\n", info->gpios[4]);
                goto err_4;
            }
            gpio_direction_input(info->gpios[4]);
        case 4:
            if (gpio_request(info->gpios[3], "detect-position-3"))
            {
                dev_err(&pdev->dev, "detect-position-3: %d request failed!\n", info->gpios[3]);
                goto err_3;
            }
            gpio_direction_input(info->gpios[3]);
        case 3:
            if (gpio_request(info->gpios[2], "detect-position-2"))
            {
                dev_err(&pdev->dev, "detect-position-2: %d request failed!\n", info->gpios[2]);
                goto err_2;
            }
            gpio_direction_input(info->gpios[2]);
        case 2:
            if (gpio_request(info->gpios[1], "detect-position-1"))
            {
                dev_err(&pdev->dev, "detect-position-1: %d request failed!\n", info->gpios[1]);
                goto err_1;
            }
            gpio_direction_input(info->gpios[1]);
        case 1:
            if (gpio_request(info->gpios[0], "detect-position-0"))
            {
                dev_err(&pdev->dev, "detect-position-0: %d request failed!\n", info->gpios[0]);
                goto err_0;
            }
            gpio_direction_input(info->gpios[0]);
            break;
        default:
            dev_err(&pdev->dev, "property firefly-used-num is invalid\n");
            goto err;
    }

    return 0;

err_0:
    gpio_free(info->gpios[0]);
err_1:
    gpio_free(info->gpios[1]);
err_2:
    gpio_free(info->gpios[2]);
err_3:
    gpio_free(info->gpios[3]);
err_4:
    gpio_free(info->gpios[4]);
err_5:
    gpio_free(info->gpios[5]);
err_6:
    gpio_free(info->gpios[6]);
err:
    return -1;
}

static struct class *class = NULL;
static struct device *position = NULL;

static ssize_t value_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int value;
    int sub_value;
    switch (info->used_num) {
        case 7:
            info->position[6] = gpio_get_value(info->gpios[6]);
            // info->position[6] = (char)(48+value);
        case 6:
            info->position[5] = gpio_get_value(info->gpios[5]);
        case 5:
            info->position[4] = gpio_get_value(info->gpios[4]);
        case 4:
            info->position[3] = gpio_get_value(info->gpios[3]);
        case 3:
            info->position[2] = gpio_get_value(info->gpios[2]);
        case 2:
            info->position[1] = gpio_get_value(info->gpios[1]);
        case 1:
            info->position[0] = gpio_get_value(info->gpios[0]);
            break;
        default:
        return sprintf(buf, "no position value");
    }
    value = info->position[0] + info->position[1] * 2 + info->position[2] * 4 + info->position[3] * 8 + info->position[4] * 16 + info->position[5] * 32 + info->position[6] * 64;
    value += 1;

    if(info->sub_board_exist == true ){
        switch (info->sub_used_num) {
            case 7:
                info->sub_position[6] = gpio_get_value(info->sub_gpios[6]);
                // info->position[6] = (char)(48+value);
            case 6:
                info->sub_position[5] = gpio_get_value(info->sub_gpios[5]);
            case 5:
                info->sub_position[4] = gpio_get_value(info->sub_gpios[4]);
            case 4:
                info->sub_position[3] = gpio_get_value(info->sub_gpios[3]);
            case 3:
                info->sub_position[2] = gpio_get_value(info->sub_gpios[2]);
            case 2:
                info->sub_position[1] = gpio_get_value(info->sub_gpios[1]);
            case 1:
                info->sub_position[0] = gpio_get_value(info->sub_gpios[0]);
                break;
            default:
            return sprintf(buf, "no sub position value");
        }

        sub_value = info->sub_position[0] + info->sub_position[1] * 2 + info->sub_position[2] * 4 + info->sub_position[3] * 8 + info->sub_position[4] * 16 + info->sub_position[5] * 32 + info->sub_position[6] * 64;
        sub_value += 1;
        return sprintf(buf, "sub%d-%02d",sub_value,value);
    }
    return sprintf(buf, "sub%02d",value);
}
static DEVICE_ATTR(value, S_IRUGO, value_show, NULL);

static int firefly_detect_position_probe(struct platform_device *pdev)
{
    int ret;

    info = devm_kzalloc(&pdev->dev,sizeof(struct firefly_posistion_info), GFP_KERNEL);
    if (!info) {
        dev_err(&pdev->dev, "devm_kzalloc failed!\n");
        return -ENOMEM;
    }
    info->sub_board_exist = false;
    ret = firefly_parse_dt(pdev,info);
    if (ret != 0) {
        return ret;
    }
    ret = firefly_set_gpio(pdev,info);
    if (ret != 0) {
        return ret;
    }

    ret = firefly_parse_dt_sub(pdev,info);
    if (ret != 0) {
        info->sub_board_exist = false;
    }else{
       ret = firefly_set_sub_gpio(pdev,info);
        if (ret != 0) {
            info->sub_board_exist = false;
        }else{
            info->sub_board_exist = true;
        }
    }

    class = class_create(THIS_MODULE, "sub_position");
    if (IS_ERR(class)) {
        pr_err("Firefly detect position failed to create class sub_position\n");
        return PTR_ERR(class);
    }

    position = device_create(class, NULL, MKDEV(0, 0), NULL, "position");
    if (IS_ERR(position)) {
        pr_err("Firefly detect position failed to create device position\n");
        class_destroy(class);
        return PTR_ERR(position);
    }

    ret = device_create_file(position, &dev_attr_value);
    if (ret) {
        pr_err("Firefly detect position failed to create attribute value \n");
        device_destroy(class, MKDEV(0, 0));
        class_destroy(class);
        return ret;
    }

    printk("Firefly detect position finish \n");
    return 0;
}

static struct of_device_id firefly_match_table[] = {
        { .compatible = "firefly,detect-position-gpio",},
        {},
};

static struct platform_driver firefly_detect_position_driver = {
        .driver = {
                .name = "firefly-detect-position",
                .owner = THIS_MODULE,
                .of_match_table = firefly_match_table,
        },
        .probe = firefly_detect_position_probe,
};

static int firefly_detect_position_init(void)
{
    int ret;
    printk("Firefly detect position driver init!\n");
    ret = platform_driver_register(&firefly_detect_position_driver);
    printk("Firefly detect position driver register return : %d \n",ret);
    return ret ;
}
module_init(firefly_detect_position_init);

static void firefly_detect_position_exit(void)
{
        platform_driver_unregister(&firefly_detect_position_driver);
}
module_exit(firefly_detect_position_exit);

MODULE_AUTHOR("maocl <service@t-firefly.com>");
MODULE_DESCRIPTION("Firefly Detect Position driver");
MODULE_ALIAS("platform:firefly-detect-position");
MODULE_LICENSE("GPL");

