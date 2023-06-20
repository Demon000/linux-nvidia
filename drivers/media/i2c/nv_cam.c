// SPDX-License-Identifier: GPL-2.0
/*
 * Common Nvidia V4L2 Sensor Driver
 *
 * Copyright (C) 2023 Analog Devices Inc.
 */

#include <linux/property.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/tegra_v4l2_camera.h>
#include <media/tegracam_core.h>

static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_SENSOR_MODE_ID,
};

#define MAX_CHIP_ID_REGS		3

struct nv_cam {
	struct i2c_client		*i2c_client;
	struct v4l2_subdev		*subdev;
	struct camera_common_data	*s_data;
	struct tegracam_device		*tc_dev;

	unsigned int			reg_bits;
	unsigned int			val_bits;

	unsigned int			num_chip_id_regs;
	u32				chip_id_regs[MAX_CHIP_ID_REGS];
	u32				chip_id_masks[MAX_CHIP_ID_REGS];
	u32				chip_id_vals[MAX_CHIP_ID_REGS];
};

static const struct regmap_config sensor_regmap_config = {
	.use_single_read = true,
	.use_single_write = true,
};

static inline int nv_cam_read_reg(struct camera_common_data *s_data,
				  u16 addr, u8 *val)
{
	unsigned int reg_val;
	int ret;

	ret = regmap_read(s_data->regmap, addr, &reg_val);
	if (ret) {
		dev_err(s_data->dev, "%s: i2c read 0x%x failed: %d",
			__func__, addr, ret);
		return ret;
	}

	*val = reg_val;

	return 0;
}

static inline int nv_cam_write_reg(struct camera_common_data *s_data,
				   u16 addr, u8 val)
{
	int ret;

	ret = regmap_write(s_data->regmap, addr, val);
	if (ret) {
		dev_err(s_data->dev, "%s: i2c write 0x%x = 0x%x failed: %d",
			__func__, addr, val, ret);
		return ret;
	}

	return 0;
}

static int nv_cam_write_table(struct nv_cam *priv, const nv_cam_reg table[])
{
	return regmap_util_write_table_8(priv->s_data->regmap, table, NULL, 0,
		NV_CAM_TABLE_WAIT_MS, NV_CAM_TABLE_END);
}

static int nv_cam_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
	return 0;
}

static int nv_cam_set_gain(struct tegracam_device *tc_dev, s64 val)
{
	return 0;
}

static int nv_cam_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	return 0;
}

static int nv_cam_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
	return 0;
}

static struct tegracam_ctrl_ops nv_cam_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.set_gain = nv_cam_set_gain,
	.set_exposure = nv_cam_set_exposure,
	.set_frame_rate = nv_cam_set_frame_rate,
	.set_group_hold = nv_cam_set_group_hold,
};

static int nv_cam_board_setup(struct nv_cam *priv);

static int nv_cam_post_register(struct camera_common_data *s_data)
{
	struct nv_cam *priv = (struct nv_cam *)s_data->priv;
	struct device *dev = s_data->dev;
	int err;

	err = nv_cam_board_setup(priv);
	if (err) {
		dev_err(dev, "board setup failed\n");
		return err;
	}

	return 0;
}

static int nv_cam_power_on(struct camera_common_data *s_data)
{
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;
	int err = 0;

	if (pdata && pdata->power_on) {
		err = pdata->power_on(pw);
		if (err)
			dev_err(dev, "%s failed.\n", __func__);
		else
			pw->state = SWITCH_ON;
		return err;
	}

	if (pw->pwdn_gpio) {
		if (gpio_cansleep(pw->pwdn_gpio))
			gpio_set_value_cansleep(pw->pwdn_gpio, 1);
		else
			gpio_set_value(pw->pwdn_gpio, 1);
	}

	if (!pw->avdd && !pw->iovdd && !pw->dvdd))
		goto skip_power_seqn;

	if (pw->reset_gpio) {
		if (gpio_cansleep(pw->reset_gpio))
			gpio_set_value_cansleep(pw->reset_gpio, 0);
		else
			gpio_set_value(pw->reset_gpio, 0);
	}

	usleep_range(10, 20);

	if (pw->avdd) {
		err = regulator_enable(pw->avdd);
		if (err)
			goto nv_cam_avdd_fail;
	}

	if (pw->iovdd) {
		err = regulator_enable(pw->iovdd);
		if (err)
			goto nv_cam_iovdd_fail;
	}

	if (pw->dvdd) {
		err = regulator_enable(pw->dvdd);
		if (err)
			goto nv_cam_dvdd_fail;
	}

	usleep_range(10, 20);

skip_power_seqn:
	if (pw->reset_gpio) {
		if (gpio_cansleep(pw->reset_gpio))
			gpio_set_value_cansleep(pw->reset_gpio, 1);
		else
			gpio_set_value(pw->reset_gpio, 1);
	}

	usleep_range(10000, 10100);

	pw->state = SWITCH_ON;

	return 0;

nv_cam_dvdd_fail:
	regulator_disable(pw->iovdd);

nv_cam_iovdd_fail:
	regulator_disable(pw->avdd);

nv_cam_avdd_fail:
	dev_err(dev, "%s failed.\n", __func__);

	return -ENODEV;
}

static int nv_cam_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

	if (pdata && pdata->power_off) {
		err = pdata->power_off(pw);
		if (err) {
			dev_err(dev, "%s failed.\n", __func__);
			return err;
		}
	} else {
		if (pw->reset_gpio) {
			if (gpio_cansleep(pw->reset_gpio))
				gpio_set_value_cansleep(pw->reset_gpio, 0);
			else
				gpio_set_value(pw->reset_gpio, 0);
		}

		if (pw->pwdn_gpio) {
			if (gpio_cansleep(pw->pwdn_gpio))
				gpio_set_value_cansleep(pw->pwdn_gpio, 0);
			else
				gpio_set_value(pw->pwdn_gpio, 0);
		}

		usleep_range(10, 20);

		if (pw->dvdd)
			regulator_disable(pw->dvdd);
		if (pw->iovdd)
			regulator_disable(pw->iovdd);
		if (pw->avdd)
			regulator_disable(pw->avdd);
	}

	usleep_range(5000, 5000);
	pw->state = SWITCH_OFF;

	return 0;
}

static int nv_cam_power_put(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;

	if (!pw)
		return -EFAULT;

	if (pw->dvdd)
		devm_regulator_put(pw->dvdd);

	if (pw->avdd)
		devm_regulator_put(pw->avdd);

	if (pw->iovdd)
		devm_regulator_put(pw->iovdd);

	pw->dvdd = NULL;
	pw->avdd = NULL;
	pw->iovdd = NULL;

	if (pw->reset_gpio)
		gpio_free(pw->reset_gpio);

	if (pw->pwdn_gpio)
		gpio_free(pw->pwdn_gpio);

	return 0;
}

static int nv_cam_power_get(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct clk *parent;
	int err = 0;

	if (!pdata) {
		dev_err(dev, "pdata missing\n");
		return -EFAULT;
	}

	/* Sensor MCLK (aka. INCK) */
	if (pdata->mclk_name) {
		pw->mclk = devm_clk_get(dev, pdata->mclk_name);
		if (IS_ERR(pw->mclk)) {
			dev_err(dev, "unable to get clock %s\n",
				pdata->mclk_name);
			return PTR_ERR(pw->mclk);
		}

		if (pdata->parentclk_name) {
			parent = devm_clk_get(dev, pdata->parentclk_name);
			if (IS_ERR(parent)) {
				dev_err(dev, "unable to get parent clock %s",
					pdata->parentclk_name);
			} else
				clk_set_parent(pw->mclk, parent);
		}
	}

	/* analog 2.8v */
	if (pdata->regulators.avdd)
		err |= camera_common_regulator_get(dev,
				&pw->avdd, pdata->regulators.avdd);
	/* IO 1.8v */
	if (pdata->regulators.iovdd)
		err |= camera_common_regulator_get(dev,
				&pw->iovdd, pdata->regulators.iovdd);
	/* dig 1.2v */
	if (pdata->regulators.dvdd)
		err |= camera_common_regulator_get(dev,
				&pw->dvdd, pdata->regulators.dvdd);
	if (err) {
		dev_err(dev, "%s: unable to get regulator(s)\n", __func__);
		goto done;
	}

	/* Reset GPIO */
	pw->reset_gpio = pdata->reset_gpio;
	if (!pdata->reset_gpio)
		goto skip_reset_gpio;
	err = gpio_request(pw->reset_gpio, "cam_reset_gpio");
	if (err < 0) {
		dev_err(dev, "%s: unable to request reset_gpio (%d)\n",
			__func__, err);
		goto done;
	}

skip_reset_gpio:

	/* PWDN GPIO */
	if (!pdata->pwdn_gpio)
		goto skip_pwdn_gpio;
	pw->pwdn_gpio = pdata->pwdn_gpio;
	err = gpio_request(pw->pwdn_gpio, "cam_pwdn_gpio");
	if (err < 0) {
		dev_err(dev, "%s: unable to request pwdn_gpio (%d)\n",
			__func__, err);
		goto done;
	}

skip_pwdn_gpio:
done:
	pw->state = SWITCH_OFF;

	return err;
}

static struct camera_common_pdata *nv_cam_parse_dt(struct tegracam_device *tc_dev)
{
	struct camera_common_pdata *board_priv_pdata;
	struct camera_common_pdata *ret = NULL;
	struct device *dev = tc_dev->dev;
	struct device_node *np = dev->of_node;
	int err = 0;
	int gpio;

	if (!np)
		return NULL;

	board_priv_pdata = devm_kzalloc(dev,
		sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata)
		return NULL;

	gpio = of_get_named_gpio(np, "reset-gpios", 0);
	if (gpio < 0) {
		if (gpio == -EPROBE_DEFER) {
			ret = ERR_PTR(-EPROBE_DEFER);
			goto error;
		}
		dev_err(dev, "reset-gpios not found\n");
		gpio = 0;
	}
	board_priv_pdata->reset_gpio = (unsigned int)gpio;

	gpio = of_get_named_gpio(np, "pwdn-gpios", 0);
	if (gpio < 0) {
		if (gpio == -EPROBE_DEFER) {
			ret = ERR_PTR(-EPROBE_DEFER);
			goto error;
		}
		dev_err(dev, "pwdn-gpios not found\n");
		gpio = 0;
	}
	board_priv_pdata->pwdn_gpio = (unsigned int)gpio;

	err = of_property_read_string(np, "mclk", &board_priv_pdata->mclk_name);
	if (err)
		dev_dbg(dev, "mclk name not present, "
			"assume sensor driven externally\n");

	err = of_property_read_string(np, "avdd-reg",
		&board_priv_pdata->regulators.avdd);
	err |= of_property_read_string(np, "iovdd-reg",
		&board_priv_pdata->regulators.iovdd);
	err |= of_property_read_string(np, "dvdd-reg",
		&board_priv_pdata->regulators.dvdd);
	if (err)
		dev_dbg(dev, "avdd, iovdd and/or dvdd reglrs. not present, "
			"assume sensor powered independently\n");

	board_priv_pdata->has_eeprom =
		of_property_read_bool(np, "has-eeprom");

	return board_priv_pdata;

error:
	devm_kfree(dev, board_priv_pdata);

	return ret;
}

static int nv_cam_set_mode(struct tegracam_device *tc_dev)
{
	struct nv_cam *priv = (struct nv_cam *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;

	int err = 0;

	err = nv_cam_write_table(priv, mode_table[NV_CAM_MODE_COMMON]);
	if (err)
		return err;

	if (s_data->mode < 0)
		return -EINVAL;

	err = nv_cam_write_table(priv, mode_table[s_data->mode]);
	if (err)
		return err;

	return 0;
}

static int nv_cam_start_streaming(struct tegracam_device *tc_dev)
{
	struct nv_cam *priv = (struct nv_cam *)tegracam_get_privdata(tc_dev);

	return nv_cam_write_table(priv, mode_table[NV_CAM_START_STREAM]);
}

static int nv_cam_stop_streaming(struct tegracam_device *tc_dev)
{
	struct nv_cam *priv = (struct nv_cam *)tegracam_get_privdata(tc_dev);

	return nv_cam_write_table(priv, mode_table[NV_CAM_STOP_STREAM]);
}

static struct camera_common_sensor_ops nv_cam_common_ops = {
	.numfrmfmts = ARRAY_SIZE(nv_cam_frmfmt),
	.frmfmt_table = nv_cam_frmfmt,
	.post_register = nv_cam_post_register,
	.power_on = nv_cam_power_on,
	.power_off = nv_cam_power_off,
	.write_reg = nv_cam_write_reg,
	.read_reg = nv_cam_read_reg,
	.parse_dt = nv_cam_parse_dt,
	.power_get = nv_cam_power_get,
	.power_put = nv_cam_power_put,
	.set_mode = nv_cam_set_mode,
	.start_streaming = nv_cam_start_streaming,
	.stop_streaming = nv_cam_stop_streaming,
};

static int __nv_cam_check_id(struct nv_cam *priv, unsigned int i)
{
	struct camera_common_data *s_data = priv->s_data;
	struct device *dev = s_data->dev;
	unsigned int reg, mask, val;
	int ret;

	reg = priv->chip_id_regs[i];
	mask = priv->chip_id_masks[i];
	val = priv->chip_id_vals[i];

	ret = nv_cam_read_reg(s_data, reg, &reg_val);
	if (ret)
		return ret;

	val &= mask;
	reg_val &= mask;

	if (reg_val !== val) {
		dev_err(dev, "Invalid chip id 0x%x, expected 0x%x\n",
			reg_val, val);
		return -EINVAL;
	}

	return 0;
}

static int __nv_cam_check_ids(struct nv_cam *priv)
{
	unsigned int reg_val;
	unsigned int i;
	int ret;

	for (i = 0; i < priv->num_chip_id_regs; i++) {
		ret = __nv_cam_check_id(priv, i);
		if (ret)
			return ret;
	}

	return 0;
}

static int nv_cam_check_ids(struct nv_cam *priv)
{
	int retry = 100;
	int err;

retry:
	err = __nv_cam_check_ids(priv);
	if (err && retry) {
		retry--;
		udelay(1000);
		goto retry;
	}

	return err;
}

static int nv_cam_board_setup(struct nv_cam *priv)
{
	struct camera_common_data *s_data = priv->s_data;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;
	int err = 0;

	if (pdata->mclk_name) {
		err = camera_common_mclk_enable(s_data);
		if (err) {
			dev_err(dev, "error turning on mclk (%d)\n", err);
			goto done;
		}
	}

	err = nv_cam_power_on(s_data);
	if (err) {
		dev_err(dev, "error during power on sensor (%d)\n", err);
		goto err_power_on;
	}

	err = nv_cam_check_ids(priv);

	nv_cam_power_off(s_data);

err_power_on:
	if (pdata->mclk_name)
		camera_common_mclk_disable(s_data);

done:
	return err;
}

static int nv_cam_parse_dt_chip_ids(struct nv_cam *priv)
{
	int ret;

	ret = device_property_count_u32(dev, "nv,chip-id-regs");
	if (ret <= 0) {
		dev_err(dev, "Failed to reach chip ID regs: %d\n", ret);
		return ret;
	}

	priv->num_chip_id_regs = ret;

	ret = device_property_read_u32_array(dev, "nv,chip-id-regs",
					     priv->chip_id_regs,
					     priv->num_chip_id_regs);
	if (ret) {
		dev_err(dev, "Failed to read chip ID regs: %d\n", ret);
		return ret;
	}

	ret = device_property_read_u32_array(dev, "nv,chip-id-masks",
					     priv->chip_id_masks,
					     priv->num_chip_id_regs);
	if (ret) {
		dev_err(dev, "Failed to read chip ID masks: %d\n", ret);
		return ret;
	}

	ret = device_property_read_u32_array(dev, "nv,chip-id-vals",
					     priv->chip_id_vals,
					     priv->num_chip_id_regs);
	if (ret) {
		dev_err(dev, "Failed to read chip ID vals: %d\n", ret);
		return ret;
	}

	return 0;
}

static int nv_cam_parse_dt_extra(struct nv_cam *priv)
{
	struct device *dev = &priv->i2c_client->dev;
	int ret;
	u32 val;

	val = 8;
	ret = device_property_read_u32(dev, "nv,reg-bits", &val);
	if (ret)
		dev_info(dev, "Failed to read register bits, using default: %d\n", ret);
	priv->reg_bits = val;

	val = 8;
	ret = device_property_read_u32(dev, "nv,val-bits", &val);
	if (ret)
		dev_info(dev, "Failed to read value bits, using default: %d\n", ret);
	priv->val_bits = val;

	ret = nv_cam_parse_dt_chip_ids(priv);
	if (ret)
		return ret;

	return 0;
}

static int nv_cam_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct regmap_config regmap_config;
	struct device *dev = &client->dev;
	struct tegracam_device *tc_dev;
	struct nv_cam *priv;
	int err;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	tc_dev = devm_kzalloc(dev, sizeof(*tc_dev), GFP_KERNEL);
	if (!tc_dev)
		return -ENOMEM;

	priv->i2c_client = tc_dev->client = client;

	err = nv_cam_parse_dt_extra(dev);
	if (err)
		return err;

	regmap_config = sensor_regmap_config;
	regmap_config->reg_bits = priv->reg_bits;
	regmap_config->val_bits = priv->val_bits;

	tc_dev->dev = dev;
	strncpy(tc_dev->name, "nv_cam", sizeof(tc_dev->name));
	tc_dev->dev_regmap_config = &regmap_config;
	tc_dev->sensor_ops = &nv_cam_common_ops;
	tc_dev->tcctrl_ops = &nv_cam_ctrl_ops;

	err = tegracam_device_register(tc_dev);
	if (err) {
		dev_err(dev, "tegra camera driver registration failed\n");
		return err;
	}
	priv->tc_dev = tc_dev;
	priv->s_data = tc_dev->s_data;
	priv->subdev = &tc_dev->s_data->subdev;
	tegracam_set_privdata(tc_dev, (void *)priv);

	return tegracam_v4l2subdev_register(tc_dev, true);
}

static int nv_cam_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct nv_cam *priv = (struct nv_cam *)s_data->priv;

	tegracam_v4l2subdev_unregister(priv->tc_dev);
	tegracam_device_unregister(priv->tc_dev);

	return 0;
}

static const struct of_device_id nv_cam_of_match[] = {
	{ .compatible = "nv,nv_cam", },
	{ },
};
MODULE_DEVICE_TABLE(of, nv_cam_of_match);

static const struct i2c_device_id nv_cam_id[] = {
	{ "nv_cam", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nv_cam_id);

static struct i2c_driver nv_cam_i2c_driver = {
	.driver = {
		.name = "nv_cam",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(nv_cam_of_match),
	},
	.probe = nv_cam_probe,
	.remove = nv_cam_remove,
	.id_table = nv_cam_id,
};
module_i2c_driver(nv_cam_i2c_driver);

MODULE_DESCRIPTION("Common Nvidia V4L2 Sensor Driver");
MODULE_AUTHOR("Analog Devices Inc.");
MODULE_LICENSE("GPL v2");
