// SPDX-License-Identifier: GPL-2.0
/* WV517 lens actuator, as fitted to the Lenovo Yoga Book OV8858 module. */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>

#define WV517_DATA_M			0x41
#define WV517_DSC_M			0x43
#define WV517_DSC_DEFAULT		0x0211
#define WV517_MAX_FOCUS			1023
#define WV517_DEFAULT_FOCUS		300
#define WV517_PARK_STEP			30
#define WV517_PARK_DELAY_US		20000
#define WV517_MOVE_DELAY_US		60000

struct wv517 {
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *focus;
	struct gpio_desc *enable_gpio;
	u16 position;
};

static inline struct wv517 *to_wv517(struct v4l2_subdev *sd)
{
	return container_of(sd, struct wv517, sd);
}

static int wv517_write(struct wv517 *vcm, u8 reg, u16 value)
{
	struct i2c_client *client = v4l2_get_subdevdata(&vcm->sd);
	u8 buf[] = { reg, value >> 8, value & 0xff };
	int ret;

	ret = i2c_master_send(client, buf, sizeof(buf));
	if (ret < 0)
		return ret;

	return ret == sizeof(buf) ? 0 : -EIO;
}

static int wv517_set_position(struct wv517 *vcm, u16 position)
{
	int ret;

	ret = wv517_write(vcm, WV517_DATA_M, position);
	if (!ret)
		vcm->position = position;

	return ret;
}

static int wv517_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct wv517 *vcm = container_of(ctrl->handler, struct wv517, ctrls);
	struct device *dev = vcm->sd.dev;
	int ret;

	if (ctrl->id != V4L2_CID_FOCUS_ABSOLUTE)
		return -EINVAL;

	/* Cache changes while closed; runtime resume restores the control. */
	ret = pm_runtime_get_if_in_use(dev);
	if (ret <= 0)
		return ret;

	ret = wv517_set_position(vcm, ctrl->val);
	pm_runtime_put(dev);

	return ret;
}

static const struct v4l2_ctrl_ops wv517_ctrl_ops = {
	.s_ctrl = wv517_set_ctrl,
};

static int wv517_runtime_resume(struct device *dev)
{
	struct wv517 *vcm = to_wv517(dev_get_drvdata(dev));
	int ret;

	mutex_lock(vcm->ctrls.lock);
	gpiod_set_value_cansleep(vcm->enable_gpio, 1);
	usleep_range(1000, 2000);

	/* Lenovo's WV517 initialization: register 0x43, big-endian 0x0211. */
	ret = wv517_write(vcm, WV517_DSC_M, WV517_DSC_DEFAULT);
	if (ret)
		goto power_off;

	ret = wv517_set_position(vcm, vcm->focus->val);
	if (ret)
		goto power_off;

	usleep_range(WV517_MOVE_DELAY_US, WV517_MOVE_DELAY_US + 1000);
	mutex_unlock(vcm->ctrls.lock);
	return 0;

power_off:
	gpiod_set_value_cansleep(vcm->enable_gpio, 0);
	mutex_unlock(vcm->ctrls.lock);
	return ret;
}

static int wv517_runtime_suspend(struct device *dev)
{
	struct wv517 *vcm = to_wv517(dev_get_drvdata(dev));
	int ret = 0;
	u16 position;

	mutex_lock(vcm->ctrls.lock);
	/* Reduce coil current gradually before removing actuator power. */
	while (vcm->position) {
		position = vcm->position > WV517_PARK_STEP ?
			vcm->position - WV517_PARK_STEP : 0;
		ret = wv517_set_position(vcm, position);
		if (ret)
			break;
		usleep_range(WV517_PARK_DELAY_US, WV517_PARK_DELAY_US + 1000);
	}
	if (ret)
		dev_warn(dev, "Failed to park lens: %d\n", ret);
	else
		usleep_range(WV517_MOVE_DELAY_US, WV517_MOVE_DELAY_US + 1000);

	/* Power must be removed even if the actuator no longer acknowledges. */
	gpiod_set_value_cansleep(vcm->enable_gpio, 0);
	mutex_unlock(vcm->ctrls.lock);
	return 0;
}

static int wv517_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return pm_runtime_resume_and_get(sd->dev);
}

static int wv517_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	int ret = pm_runtime_put_sync(sd->dev);

	return ret < 0 ? ret : 0;
}

static const struct v4l2_subdev_internal_ops wv517_internal_ops = {
	.open = wv517_open,
	.close = wv517_close,
};

static const struct v4l2_subdev_core_ops wv517_core_ops = {
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_ops wv517_subdev_ops = {
	.core = &wv517_core_ops,
};

static int wv517_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct wv517 *vcm;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EOPNOTSUPP;

	vcm = devm_kzalloc(dev, sizeof(*vcm), GFP_KERNEL);
	if (!vcm)
		return -ENOMEM;

	vcm->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(vcm->enable_gpio))
		return dev_err_probe(dev, PTR_ERR(vcm->enable_gpio),
				     "Failed to get enable GPIO\n");

	v4l2_i2c_subdev_init(&vcm->sd, client, &wv517_subdev_ops);
	vcm->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	vcm->sd.internal_ops = &wv517_internal_ops;
	vcm->sd.entity.function = MEDIA_ENT_F_LENS;

	v4l2_ctrl_handler_init(&vcm->ctrls, 1);
	vcm->focus = v4l2_ctrl_new_std(&vcm->ctrls, &wv517_ctrl_ops,
				       V4L2_CID_FOCUS_ABSOLUTE,
				       0, WV517_MAX_FOCUS, 1, WV517_DEFAULT_FOCUS);
	ret = vcm->ctrls.error;
	if (ret)
		goto free_ctrls;
	vcm->sd.ctrl_handler = &vcm->ctrls;

	ret = media_entity_pads_init(&vcm->sd.entity, 0, NULL);
	if (ret)
		goto free_ctrls;

	/* Do not touch the motor until its sensor's PM device link is ready. */
	pm_runtime_set_suspended(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev(&vcm->sd);
	if (ret)
		goto disable_pm;

	return 0;

disable_pm:
	pm_runtime_disable(dev);
	media_entity_cleanup(&vcm->sd.entity);
free_ctrls:
	v4l2_ctrl_handler_free(&vcm->ctrls);
	return ret;
}

static void wv517_remove(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct wv517 *vcm = to_wv517(i2c_get_clientdata(client));

	v4l2_async_unregister_subdev(&vcm->sd);
	pm_runtime_disable(dev);
	if (!pm_runtime_status_suspended(dev))
		wv517_runtime_suspend(dev);
	pm_runtime_set_suspended(dev);
	media_entity_cleanup(&vcm->sd.entity);
	v4l2_ctrl_handler_free(&vcm->ctrls);
}

static const struct dev_pm_ops wv517_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
	RUNTIME_PM_OPS(wv517_runtime_suspend, wv517_runtime_resume, NULL)
};

static const struct i2c_device_id wv517_id_table[] = {
	{ "wv517" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, wv517_id_table);

static struct i2c_driver wv517_driver = {
	.driver = {
		.name = "wv517",
		.pm = pm_ptr(&wv517_pm_ops),
	},
	.probe = wv517_probe,
	.remove = wv517_remove,
	.id_table = wv517_id_table,
};
module_i2c_driver(wv517_driver);

MODULE_DESCRIPTION("WV517 lens actuator driver");
MODULE_LICENSE("GPL");
