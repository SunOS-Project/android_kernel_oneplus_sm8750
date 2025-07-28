// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/hwmon.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/thermal.h>
#include <linux/timer.h>
#include <linux/leds.h>
#include <linux/delay.h>
#include <linux/pm.h>

#define MAX_DUTY 100
#define DEFAULT_FAN_PWM_PERIOD_NS 40000
#define DEFAULT_PULSES_PER_REVOLUTION 2
#define MAX_LEVEL_DEFAULT 10
#define DEVICE_ID_HONGYING 0
#define DEVICE_ID_TAIDA 1

struct oplus_fan_tach {
	int fg_irq_gpio;
	atomic_t pulses;
	unsigned int rpm;
	bool fg_irq_en;
	struct mutex irq_lock;
};

struct fan_hw_config {
	int max_level;
	int duty_config[MAX_LEVEL_DEFAULT];
	int pulses_per_revolution;
};

struct pwm_setting {
	u64	pre_period_ns;
	u64	period_ns;
	u32	duty;
	bool	enabled;
};

struct oplus_fan_chip {
	struct device *dev;
	struct mutex lock;
	struct led_classdev cdev;
	struct pwm_device *pwm_dev;
	struct pwm_setting pwm_setting;
	struct fan_hw_config *hw_config;
	int device_count;
	int device_id;
	int level;
	struct regulator *reg_en;
	int reg_en_gpio;
	bool regulator_enabled;
	bool rpm_timer_enabled;
	struct oplus_fan_tach tach;
	ktime_t sample_start;
	struct timer_list rpm_timer;
};

static struct fan_hw_config default_hw_config = {
	.max_level = MAX_LEVEL_DEFAULT,
	.duty_config = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100},
	.pulses_per_revolution = DEFAULT_PULSES_PER_REVOLUTION,
};

/* This handler assumes self resetting edge triggered interrupt. */
static irqreturn_t pulse_handler(int irq, void *dev_id)
{
	struct oplus_fan_tach *tach = dev_id;

	atomic_inc(&tach->pulses);

	return IRQ_HANDLED;
}

static void sample_timer(struct timer_list *t)
{
	struct oplus_fan_chip *chip = from_timer(chip, t, rpm_timer);
	struct oplus_fan_tach *tach = &chip->tach;
	unsigned int delta = ktime_ms_delta(ktime_get(), chip->sample_start);
	int pulses;
	int pulses_per_rev;

	pulses_per_rev = chip->hw_config[chip->device_id].pulses_per_revolution;
	if (pulses_per_rev <= 0) {
		dev_err(chip->dev, "error pulses_per_revolution = %d\n", pulses_per_rev);
		pulses_per_rev = DEFAULT_PULSES_PER_REVOLUTION;
	}

	if (delta) {
		pulses = atomic_read(&tach->pulses);
		atomic_sub(pulses, &tach->pulses);
		tach->rpm = (unsigned int)(pulses * 1000 * 60) / (pulses_per_rev * delta);
		chip->sample_start = ktime_get();
	}

	if (!chip->rpm_timer_enabled) {
		tach->rpm = 0;
		dev_err(chip->dev, "sample_timer stop\n");
		return;
	}

	dev_err(chip->dev, "sample_timer:delta=%u ms, duty=%u, pulses=%d, rpm=%u\n",
			delta, chip->pwm_setting.duty, pulses, tach->rpm);

	mod_timer(&chip->rpm_timer, jiffies + HZ);
}

static void oplus_fan_fg_irq_config(struct oplus_fan_chip *chip,
						bool enabled)
{
	if (!chip)
		return;

	if (!gpio_is_valid(chip->tach.fg_irq_gpio))
		return;

	mutex_lock(&chip->tach.irq_lock);
	if (!chip->tach.fg_irq_en && enabled) {
		enable_irq(gpio_to_irq(chip->tach.fg_irq_gpio));
		chip->tach.fg_irq_en = true;
	} else if (chip->tach.fg_irq_en && !enabled) {
		disable_irq_nosync(gpio_to_irq(chip->tach.fg_irq_gpio));
		chip->tach.fg_irq_en = false;
	}
	mutex_unlock(&chip->tach.irq_lock);
}

static void oplus_fan_sample_timer_enable(struct oplus_fan_chip *chip,
						bool enabled)
{
	int pulses;

	if (!chip)
		return;

	chip->rpm_timer_enabled = enabled;
	dev_err(chip->dev, "sample_timer enable = %d\n", enabled);

	if (enabled) {
		oplus_fan_fg_irq_config(chip, true);
		pulses = atomic_read(&chip->tach.pulses);
		atomic_sub(pulses, &chip->tach.pulses);
		chip->sample_start = ktime_get();
		mod_timer(&chip->rpm_timer, jiffies + HZ);
	} else {
		oplus_fan_fg_irq_config(chip, false);
	}
}

static int oplus_fan_parse_hw_config(struct oplus_fan_chip *chip)
{
	struct device_node *np = chip->dev->of_node;
	struct device_node *temp;
	struct fan_hw_config *config;
	int ret;
	int count;
	int i = 0;

	count = of_get_child_count(np);
	if (count < 1) {
		dev_err(chip->dev, "don't have hw config\n");
		goto parse_err;
	}

	chip->device_count = count;
	chip->hw_config = devm_kcalloc(chip->dev,
			count, sizeof(struct fan_hw_config), GFP_KERNEL);
	if (!chip->hw_config) {
		dev_err(chip->dev, "failed to kcalloc memory\n");
		goto parse_err;
	}

	for_each_child_of_node(np, temp) {
		config = &chip->hw_config[i];
		ret = of_property_read_u32(temp, "pulses-per-revolution",
				&config->pulses_per_revolution);
		if (ret < 0) {
			dev_err(chip->dev, "pulses-per-revolution is not set\n");
			goto parse_err;
		}

		count = of_property_count_elems_of_size(temp, "duty-config", sizeof(int));
		if (count > 0 && count <= MAX_LEVEL_DEFAULT) {
			ret = of_property_read_u32_array(temp, "duty-config", (u32 *)config->duty_config, count);
			if (ret) {
				dev_err(chip->dev, "failed to get duty-config ret = %d\n", ret);
				goto parse_err;
			}
			config->max_level = count;
		} else {
			dev_err(chip->dev, "failed to get duty-config count = %d\n", count);
			goto parse_err;
		}
		dev_err(chip->dev, "parse config[%d] pulses_per_revolution = %d\n", i, config->pulses_per_revolution);
		dev_err(chip->dev, "duty_config = %d,%d,%d,%d,%d, %d,%d,%d,%d,%d\n",
				config->duty_config[0], config->duty_config[1], config->duty_config[2],
				config->duty_config[3], config->duty_config[4], config->duty_config[5],
				config->duty_config[6], config->duty_config[7], config->duty_config[8],
				config->duty_config[9]);
		i++;
	}

	return 0;

parse_err:
	if (chip->hw_config)
		devm_kfree(chip->dev, chip->hw_config);
	return -1;
}


static int oplus_fan_parse_dt(struct oplus_fan_chip *chip)
{
	struct device_node *np = chip->dev->of_node;
	struct pwm_args pargs;
	struct oplus_fan_tach *tach = &chip->tach;
	int irq_flags;
	int ret;

	chip->reg_en = NULL;
	chip->reg_en_gpio = of_get_named_gpio(np, "reg-en-gpio", 0);
	if (gpio_is_valid(chip->reg_en_gpio)) {
		ret = devm_gpio_request_one(chip->dev, chip->reg_en_gpio,
					    GPIOF_OUT_INIT_LOW, "fan_reg_en");
		if (ret)
			dev_err(chip->dev, "failed to request reg-en-gpio, ret=%d\n", ret);
		else
			dev_err(chip->dev, "request reg-en-gpio = %d\n", chip->reg_en_gpio);
	} else {
		dev_err(chip->dev, "failed to get reg-en-gpio, try fan regulator\n");
		chip->reg_en = devm_regulator_get_optional(chip->dev, "fan");
		if (IS_ERR(chip->reg_en)) {
			dev_err(chip->dev, "failed to get fan regulator, ret=%ld\n",
					PTR_ERR(chip->reg_en));
			chip->reg_en = NULL;
		}
	}

	tach->fg_irq_gpio = of_get_named_gpio(np, "fg-irq-gpio", 0);
	if (gpio_is_valid(tach->fg_irq_gpio)) {
		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		ret = devm_request_threaded_irq(chip->dev,
						gpio_to_irq(tach->fg_irq_gpio),
						NULL, pulse_handler, irq_flags,
						"fan_fg", tach);
		if (ret != 0) {
			dev_err(chip->dev, "Failed to request irq: %d, ret=%d\n",
					gpio_to_irq(tach->fg_irq_gpio), ret);
			return ret;
		}
		disable_irq_nosync(gpio_to_irq(tach->fg_irq_gpio));
		tach->fg_irq_en = false;
	}
	dev_err(chip->dev, "tach: fg_irq_gpio=%d\n", tach->fg_irq_gpio);

	chip->pwm_dev = devm_pwm_get(chip->dev, NULL);
	if (IS_ERR(chip->pwm_dev)) {
		dev_err(chip->dev, "failed to get pwm device, ret=%ld\n",
				PTR_ERR(chip->pwm_dev));
		chip->pwm_dev = NULL;
	}

	if (chip->pwm_dev) {
		pwm_get_args(chip->pwm_dev, &pargs);
		if (pargs.period == 0)
			chip->pwm_setting.pre_period_ns = DEFAULT_FAN_PWM_PERIOD_NS;
		else
			chip->pwm_setting.pre_period_ns = pargs.period;
		dev_err(chip->dev, "pwm setting pre_period_ns = %llu, ret=%d\n",
				chip->pwm_setting.pre_period_ns, ret);
	}

	return 0;
}

static int oplus_fan_set_pwm(struct oplus_fan_chip *chip)
{
	struct pwm_setting *pwm = &chip->pwm_setting;
	struct pwm_state pstate;
	int ret;

	if (!chip->pwm_dev)
		return 0;

	mutex_lock(&chip->lock);
	pwm_get_state(chip->pwm_dev, &pstate);
	pstate.enabled = pwm->enabled;
	pstate.period = pwm->period_ns;
	pstate.duty_cycle = DIV_ROUND_UP(pwm->duty * (pwm->period_ns - 1), MAX_DUTY);
	dev_err(chip->dev, "configure PWM:enabled=%d, period=%llu, duty=%u, duty_cycle=%llu\n",
			pstate.enabled, pstate.period, pwm->duty, pstate.duty_cycle);

	ret = pwm_apply_state(chip->pwm_dev, &pstate);
	if (ret)
		dev_err(chip->dev, "Failed to configure PWM: %d\n", ret);
	mutex_unlock(&chip->lock);

	return ret;
}

static int oplus_fan_regulator_set(struct oplus_fan_chip *chip, bool enable)
{
	int ret;

	if (enable) {
		if (gpio_is_valid(chip->reg_en_gpio)) {
			gpio_set_value_cansleep(chip->reg_en_gpio, 1);
			usleep_range(2000, 4000);
		} else if (chip->reg_en) {
			ret = regulator_enable(chip->reg_en);
			if (ret)
				dev_err(chip->dev, "failed to enable regulator ret = %d.\n", ret);
		}
	} else {
		if (gpio_is_valid(chip->reg_en_gpio)) {
			gpio_set_value_cansleep(chip->reg_en_gpio, 0);
			usleep_range(2000, 4000);
		} else if (chip->reg_en) {
			ret = regulator_disable(chip->reg_en);
			if (ret)
				dev_err(chip->dev, "failed to disable regulator ret = %d.\n", ret);
		}
	}
	chip->regulator_enabled = enable;
	dev_err(chip->dev, "fan regulator enabled = %d\n", enable);

	return 0;
}

static void oplus_fan_enable(struct oplus_fan_chip *chip, bool enabled)
{
	if (!chip)
		return;

	if (enabled) {
		oplus_fan_regulator_set(chip, true);
		msleep(100);
		chip->pwm_setting.enabled = true;
		oplus_fan_set_pwm(chip);
		oplus_fan_sample_timer_enable(chip, true);
	} else {
		chip->pwm_setting.enabled = false;
		oplus_fan_set_pwm(chip);
		msleep(100);
		oplus_fan_regulator_set(chip, false);
		oplus_fan_sample_timer_enable(chip, false);
	}

	return;
}

static int oplus_fan_init(struct oplus_fan_chip *chip)
{
	chip->pwm_setting.period_ns = chip->pwm_setting.pre_period_ns;
	chip->pwm_setting.duty = MAX_DUTY;
	chip->pwm_setting.enabled = false;
	chip->level = 0;
	oplus_fan_set_pwm(chip);

	return 0;
}

static ssize_t speed_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct led_classdev *fan_cdev = dev_get_drvdata(dev);
	struct oplus_fan_chip *chip =
		container_of(fan_cdev, struct oplus_fan_chip, cdev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", chip->pwm_setting.duty);
}

static ssize_t speed_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct led_classdev *fan_cdev = dev_get_drvdata(dev);
	struct oplus_fan_chip *chip =
		container_of(fan_cdev, struct oplus_fan_chip, cdev);
	int rc;
	u32 speed;

	rc = kstrtouint(buf, 0, &speed);
	if (rc < 0)
		return rc;

	if (speed > MAX_DUTY)
		speed = MAX_DUTY;

	chip->pwm_setting.duty = speed;
	oplus_fan_set_pwm(chip);

	return count;
}
static DEVICE_ATTR_RW(speed);

static ssize_t state_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct led_classdev *fan_cdev = dev_get_drvdata(dev);
	struct oplus_fan_chip *chip =
		container_of(fan_cdev, struct oplus_fan_chip, cdev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->pwm_setting.enabled);
}

static ssize_t state_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct led_classdev *fan_cdev = dev_get_drvdata(dev);
	struct oplus_fan_chip *chip =
		container_of(fan_cdev, struct oplus_fan_chip, cdev);
	int rc;
	bool enabled;

	rc = kstrtobool(buf, &enabled);
	if (rc < 0)
		return rc;
	dev_err(chip->dev, "state_store = %d\n", enabled);

	oplus_fan_enable(chip, enabled);

	return count;
}
static DEVICE_ATTR_RW(state);

static ssize_t rpm_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct led_classdev *fan_cdev = dev_get_drvdata(dev);
	struct oplus_fan_chip *chip =
		container_of(fan_cdev, struct oplus_fan_chip, cdev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", chip->tach.rpm);
}

static ssize_t rpm_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct led_classdev *fan_cdev = dev_get_drvdata(dev);
	struct oplus_fan_chip *chip =
		container_of(fan_cdev, struct oplus_fan_chip, cdev);
	bool enabled;
	int rc;

	rc = kstrtobool(buf, &enabled);
	if (rc < 0)
		return rc;

	dev_err(chip->dev, "rpm_store enable = %d\n", enabled);
	oplus_fan_sample_timer_enable(chip, enabled);

	return count;
}
static DEVICE_ATTR_RW(rpm);

static ssize_t level_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct led_classdev *fan_cdev = dev_get_drvdata(dev);
	struct oplus_fan_chip *chip =
			container_of(fan_cdev, struct oplus_fan_chip, cdev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->level);
}

static ssize_t level_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct led_classdev *fan_cdev = dev_get_drvdata(dev);
	struct oplus_fan_chip *chip =
			container_of(fan_cdev, struct oplus_fan_chip, cdev);
	int rc;
	u32 level;

	rc = kstrtouint(buf, 0, &level);
	if (rc < 0)
		return rc;

	if (level <= 0) {
		dev_err(chip->dev, "level range is between 1 and %d\n",
				chip->hw_config[chip->device_id].max_level);
		return count;
	}

	if (level > chip->hw_config[chip->device_id].max_level) {
		dev_err(chip->dev, "set level = %d, out of range, force level = %d\n",
				level, chip->hw_config[chip->device_id].max_level);
		level = chip->hw_config[chip->device_id].max_level;
	}

	chip->level = level;
	chip->pwm_setting.duty = chip->hw_config[chip->device_id].duty_config[level - 1];
	dev_err(chip->dev, "set level=%d, duty=%d\n", level, chip->pwm_setting.duty);

	oplus_fan_set_pwm(chip);

	return count;
}
static DEVICE_ATTR_RW(level);

static ssize_t device_id_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct led_classdev *fan_cdev = dev_get_drvdata(dev);
	struct oplus_fan_chip *chip =
			container_of(fan_cdev, struct oplus_fan_chip, cdev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->device_id);
}

static ssize_t device_id_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct led_classdev *fan_cdev = dev_get_drvdata(dev);
	struct oplus_fan_chip *chip =
			container_of(fan_cdev, struct oplus_fan_chip, cdev);
	int rc;
	u32 val;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	dev_err(chip->dev, "set device_id = %u\n", val);

	if (val >= chip->device_count) {
		val = chip->device_count - 1;
		dev_err(chip->dev, "only %d device, force device_id = %u\n",
				chip->device_count, val);
	}

	chip->device_id = val;
	if (chip->level > 0)
		chip->pwm_setting.duty = chip->hw_config[chip->device_id].duty_config[chip->level - 1];

	oplus_fan_set_pwm(chip);

	return count;
}
static DEVICE_ATTR_RW(device_id);

static struct attribute *oplus_fan_attrs[] = {
	&dev_attr_speed.attr,
	&dev_attr_state.attr,
	&dev_attr_rpm.attr,
	&dev_attr_level.attr,
	&dev_attr_device_id.attr,
	NULL
};

static struct attribute_group oplus_fan_attrs_group = {
	.attrs = oplus_fan_attrs
};

static void oplus_fan_set_brightness(struct led_classdev *fan_cdev,
		enum led_brightness brightness)
{
	struct oplus_fan_chip *chip =
		container_of(fan_cdev, struct oplus_fan_chip, cdev);

	dev_err(chip->dev, "fan class set brightness, brightness=%d\n", brightness);
}

static enum led_brightness oplus_fan_get_brightness(
			struct led_classdev *fan_cdev)
{
	struct oplus_fan_chip *chip =
		container_of(fan_cdev, struct oplus_fan_chip, cdev);

	dev_err(chip->dev, "fan class get brightness, brightness=%d\n", fan_cdev->brightness);

	return fan_cdev->brightness;
}

static int oplus_fan_cdev_register(struct oplus_fan_chip *chip)
{
	int rc;

	chip->cdev.name = "fan";
	chip->cdev.max_brightness = LED_FULL;
	chip->cdev.brightness_set = oplus_fan_set_brightness;
	chip->cdev.brightness_get = oplus_fan_get_brightness;
	chip->cdev.brightness = 0;
	rc = devm_led_classdev_register(chip->dev, &chip->cdev);
	if (rc < 0) {
		dev_err(chip->dev, "failed to register fan class, rc=%d\n", rc);
		return rc;
	}

	rc = sysfs_create_group(&chip->cdev.dev->kobj,
				 &oplus_fan_attrs_group);
	if (rc < 0) {
		dev_err(chip->dev, "failed to create sysfs attrs, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static void oplus_fan_cleanup(void *__ctx)
{
	struct oplus_fan_chip *chip = __ctx;

	del_timer_sync(&chip->rpm_timer);
	/* Switch off everything */
	oplus_fan_enable(chip, false);
}

static int oplus_fan_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct oplus_fan_chip *chip;
	int ret;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	mutex_init(&chip->lock);
	mutex_init(&chip->tach.irq_lock);
	chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, chip);

	ret = oplus_fan_parse_dt(chip);
	if (ret)
		return ret;

	ret = oplus_fan_parse_hw_config(chip);
	if (ret) {
		dev_err(chip->dev, "use default hw config\n");
		chip->device_count = 1;
		chip->hw_config = devm_kzalloc(chip->dev,
				sizeof(struct fan_hw_config), GFP_KERNEL);
		if (!chip->hw_config) {
			dev_err(chip->dev, "failed to kzalloc memory\n");
			return ret;
		}
		memcpy(chip->hw_config, &default_hw_config, sizeof(struct fan_hw_config));
	}
	chip->device_id = DEVICE_ID_HONGYING;

	timer_setup(&chip->rpm_timer, sample_timer, 0);
	ret = devm_add_action_or_reset(dev, oplus_fan_cleanup, chip);
	if (ret)
		return ret;

	oplus_fan_cdev_register(chip);
	oplus_fan_init(chip);

	dev_err(chip->dev, "probe complete!\n");
	return 0;
}

static int oplus_fan_remove(struct platform_device *pdev)
{
	struct oplus_fan_chip *chip = platform_get_drvdata(pdev);

	if (gpio_is_valid(chip->reg_en_gpio))
		gpio_free(chip->reg_en_gpio);

	if (gpio_is_valid(chip->tach.fg_irq_gpio))
		free_irq(gpio_to_irq(chip->tach.fg_irq_gpio), &chip->tach);

	return 0;
}

static void oplus_fan_shutdown(struct platform_device *pdev)
{
	struct oplus_fan_chip *chip = platform_get_drvdata(pdev);

	oplus_fan_cleanup(chip);
}

static int oplus_fan_suspend(struct device *dev)
{
	struct oplus_fan_chip *chip = dev_get_drvdata(dev);

	dev_err(chip->dev, "fan suspend\n");

	return 0;
}

static int oplus_fan_resume(struct device *dev)
{
	struct oplus_fan_chip *chip = dev_get_drvdata(dev);

	dev_err(chip->dev, "fan resume\n");

	return 0;
}

static const struct dev_pm_ops oplus_fan_pm_ops = {
	.suspend = oplus_fan_suspend,
	.resume = oplus_fan_resume,
};

static const struct of_device_id of_oplus_fan_match[] = {
	{ .compatible = "oplus,pwm-fan", },
	{},
};
MODULE_DEVICE_TABLE(of, of_oplus_fan_match);

static struct platform_driver oplus_fan_driver = {
	.probe		= oplus_fan_probe,
	.remove		= oplus_fan_remove,
	.shutdown	= oplus_fan_shutdown,
	.driver	= {
		.name		= "pwm-fan",
		.pm		= &oplus_fan_pm_ops,
		.of_match_table	= of_oplus_fan_match,
	},
};

module_platform_driver(oplus_fan_driver);

MODULE_ALIAS("platform:pwm-fan");
MODULE_DESCRIPTION("PWM FAN driver");
MODULE_LICENSE("GPL");
