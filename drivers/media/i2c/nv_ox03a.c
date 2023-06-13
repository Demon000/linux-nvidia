/*
 * ox03a.c - ox03a sensor driver
 *
 * Copyright (c) 2023, Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

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

#include "../platform/tegra/camera/camera_gpio.h"
#include "ox03a_mode_tbls.h"

#define OX03A_PID 0x300A
#define OX03A_VER 0x300B

#define OX03A_MEDIA_BUS_FMT MEDIA_BUS_FMT_SBGGR12_1X12

static const struct of_device_id ox03a_of_match[] = {
	{ .compatible = "ov,ox03a", },
	{ },
};
MODULE_DEVICE_TABLE(of, ox03a_of_match);

static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_SENSOR_MODE_ID,
	TEGRA_CAMERA_CID_HCG_WB_GAIN_B,
	TEGRA_CAMERA_CID_HCG_WB_GAIN_Gb,
	TEGRA_CAMERA_CID_HCG_WB_GAIN_Gr,
	TEGRA_CAMERA_CID_HCG_WB_GAIN_R,
	TEGRA_CAMERA_CID_LCG_WB_GAIN_B,
	TEGRA_CAMERA_CID_LCG_WB_GAIN_Gb,
	TEGRA_CAMERA_CID_LCG_WB_GAIN_Gr,
	TEGRA_CAMERA_CID_LCG_WB_GAIN_R,
	TEGRA_CAMERA_CID_VS_WB_GAIN_B,
	TEGRA_CAMERA_CID_VS_WB_GAIN_Gb,
	TEGRA_CAMERA_CID_VS_WB_GAIN_Gr,
	TEGRA_CAMERA_CID_VS_WB_GAIN_R,
};

struct ox03a {
	struct i2c_client		*i2c_client;
	struct v4l2_subdev		*subdev;
	struct camera_common_data	*s_data;
	struct tegracam_device		*tc_dev;
	struct dentry			*debugfs_root;
	unsigned			cached_reg_addr;
	char				read_buf[20];
	unsigned int			read_buf_len;
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0)
	.use_single_rw = true,
#else
	.use_single_read = true,
	.use_single_write = true,
#endif
};

static inline int ox03a_read_reg(struct camera_common_data *s_data,
	u16 addr, u8 *val)
{
	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(s_data->regmap, addr, &reg_val);
	*val = reg_val & 0xff;

	return err;
}

static inline int ox03a_write_reg(struct camera_common_data *s_data,
	u16 addr, u8 val)
{
	int err = 0;

	err = regmap_write(s_data->regmap, addr, val);
	if (err)
		dev_err(s_data->dev, "%s: i2c write failed, 0x%x = %x",
			__func__, addr, val);

	return err;
}

static int ox03a_write_table(struct ox03a *priv, const ox03a_reg table[])
{
	return regmap_util_write_table_8(priv->s_data->regmap, table, NULL, 0,
		OX03A_TABLE_WAIT_MS, OX03A_TABLE_END);
}

static int ox03a_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
	/* ox03a does not support group hold */
	return 0;
}

static int ox03a_set_gain(struct tegracam_device *tc_dev, s64 val)
{
	return 0;
}

static int ox03a_set_awb_gain(struct tegracam_device *tc_dev, enum awb_gain_type type, s64 val)
{
	unsigned int reg = 0x5180 + 0x20 * (type / 4) + 0x2 * (type % 4);
	struct camera_common_data *s_data = tc_dev->s_data;
	int err;

	err = ox03a_write_reg(s_data, reg, (val >> 8) & 0x7f);
	if (err)
		return err;

	err = ox03a_write_reg(s_data, reg + 1, val & 0xff);
	if (err)
		return err;

	return 0;
}

static int ox03a_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	return 0;
}

static int ox03a_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
	return 0;
}

static struct tegracam_ctrl_ops ox03a_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.set_gain = ox03a_set_gain,
	.set_awb_gain = ox03a_set_awb_gain,
	.set_exposure = ox03a_set_exposure,
	.set_frame_rate = ox03a_set_frame_rate,
	.set_group_hold = ox03a_set_group_hold,
};

static int ox03a_board_setup(struct ox03a *priv);

static int ox03a_post_register(struct camera_common_data *s_data)
{
	struct ox03a *priv = (struct ox03a *)s_data->priv;
	struct device *dev = s_data->dev;
	int err;

	err = ox03a_board_setup(priv);
	if (err) {
		dev_err(dev, "board setup failed\n");
		return err;
	}

	return 0;
}

static int ox03a_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

	dev_dbg(dev, "%s: power on\n", __func__);
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

	if (unlikely(!(pw->avdd || pw->iovdd || pw->dvdd)))
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
			goto ox03a_avdd_fail;
	}

	if (pw->iovdd) {
		err = regulator_enable(pw->iovdd);
		if (err)
			goto ox03a_iovdd_fail;
	}

	if (pw->dvdd) {
		err = regulator_enable(pw->dvdd);
		if (err)
			goto ox03a_dvdd_fail;
	}

	usleep_range(10, 20);

skip_power_seqn:
	if (pw->reset_gpio) {
		if (gpio_cansleep(pw->reset_gpio))
			gpio_set_value_cansleep(pw->reset_gpio, 1);
		else
			gpio_set_value(pw->reset_gpio, 1);
	}

	/* Need to wait for t4 + t5 + t9 time as per the data sheet */
	/* t4 - 200us, t5 - 6ms, t9 - 1.2ms */
	/* also add 2ms to workaround the i2c timeout issue on 2nd camera */
	usleep_range(10000, 10100);

	pw->state = SWITCH_ON;

	return 0;

ox03a_dvdd_fail:
	regulator_disable(pw->iovdd);

ox03a_iovdd_fail:
	regulator_disable(pw->avdd);

ox03a_avdd_fail:
	dev_err(dev, "%s failed.\n", __func__);

	return -ENODEV;
}

static int ox03a_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

	dev_dbg(dev, "%s: power off\n", __func__);

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

static int ox03a_power_put(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;

	if (unlikely(!pw))
		return -EFAULT;

	if (likely(pw->dvdd))
		devm_regulator_put(pw->dvdd);

	if (likely(pw->avdd))
		devm_regulator_put(pw->avdd);

	if (likely(pw->iovdd))
		devm_regulator_put(pw->iovdd);

	pw->dvdd = NULL;
	pw->avdd = NULL;
	pw->iovdd = NULL;

	if (likely(pw->reset_gpio))
		gpio_free(pw->reset_gpio);

	if (likely(pw->pwdn_gpio))
		gpio_free(pw->pwdn_gpio);

	return 0;
}

static int ox03a_power_get(struct tegracam_device *tc_dev)
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

static struct camera_common_pdata *ox03a_parse_dt(
	struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *np = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	struct camera_common_pdata *ret = NULL;
	int err = 0;
	int gpio;

	if (!np)
		return NULL;

	match = of_match_device(ox03a_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(dev,
		sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata)
		return NULL;

	gpio = of_get_named_gpio(np, "reset-gpios", 0);
	if (gpio < 0) {
		if (gpio == -EPROBE_DEFER)
			ret = ERR_PTR(-EPROBE_DEFER);
		dev_err(dev, "reset-gpios not found\n");
		gpio = 0;
	}
	board_priv_pdata->reset_gpio = (unsigned int)gpio;

	gpio = of_get_named_gpio(np, "pwdn-gpios", 0);
	if (gpio < 0) {
		if (gpio == -EPROBE_DEFER)
			ret = ERR_PTR(-EPROBE_DEFER);
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

	devm_kfree(dev, board_priv_pdata);

	return ret;
}

static int ox03a_set_mode(struct tegracam_device *tc_dev)
{
	struct ox03a *priv = (struct ox03a *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;

	int err = 0;

	err = ox03a_write_table(priv, mode_table[OX03A_MODE_COMMON]);
	if (err)
		return err;

	if (s_data->mode < 0)
		return -EINVAL;
	err = ox03a_write_table(priv, mode_table[s_data->mode]);
	if (err)
		return err;

	return 0;
}

static int ox03a_start_streaming(struct tegracam_device *tc_dev)
{
	struct ox03a *priv = (struct ox03a *)tegracam_get_privdata(tc_dev);

	return ox03a_write_table(priv, mode_table[OX03A_START_STREAM]);
}

static int ox03a_stop_streaming(struct tegracam_device *tc_dev)
{
	struct ox03a *priv = (struct ox03a *)tegracam_get_privdata(tc_dev);

	return ox03a_write_table(priv, mode_table[OX03A_STOP_STREAM]);
}

static struct camera_common_sensor_ops ox03a_common_ops = {
	.numfrmfmts = ARRAY_SIZE(ox03a_frmfmt),
	.frmfmt_table = ox03a_frmfmt,
	.post_register = ox03a_post_register,
	.power_on = ox03a_power_on,
	.power_off = ox03a_power_off,
	.write_reg = ox03a_write_reg,
	.read_reg = ox03a_read_reg,
	.parse_dt = ox03a_parse_dt,
	.power_get = ox03a_power_get,
	.power_put = ox03a_power_put,
	.set_mode = ox03a_set_mode,
	.start_streaming = ox03a_start_streaming,
	.stop_streaming = ox03a_stop_streaming,
};

static int __ox03a_check_id(struct ox03a *priv)
{
	struct camera_common_data *s_data = priv->s_data;
	struct device *dev = s_data->dev;
	u8 reg_val[2];
	int err;

	/* Probe sensor model id registers */
	err = ox03a_read_reg(s_data, OX03A_PID, &reg_val[0]);
	if (err) {
		dev_err(dev, "%s: error during i2c read probe (%d)\n",
			__func__, err);
		return err;
	}
	err = ox03a_read_reg(s_data, OX03A_VER, &reg_val[1]);
	if (err) {
		dev_err(dev, "%s: error during i2c read probe (%d)\n",
			__func__, err);
		return err;
	}
	if (!((reg_val[0] == 0x58) && reg_val[1] == 0x03)) {
		dev_err(dev, "%s: invalid sensor model id: %x%x\n",
			__func__, reg_val[0], reg_val[1]);
		return -EINVAL;
	}

	return 0;
}

static int ox03a_check_id(struct ox03a *priv)
{
	int retry = 100;
	int err;

retry:
	err = __ox03a_check_id(priv);
	if (err && retry) {
		retry--;
		udelay(1000);
		goto retry;
	}

	return err;
}

static int ox03a_board_setup(struct ox03a *priv)
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

	err = ox03a_power_on(s_data);
	if (err) {
		dev_err(dev, "error during power on sensor (%d)\n", err);
		goto err_power_on;
	}

	err = ox03a_check_id(priv);

	ox03a_power_off(s_data);

err_power_on:
	if (pdata->mclk_name)
		camera_common_mclk_disable(s_data);

done:
	return err;
}

static int ox03a_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	return 0;
}

static const struct v4l2_subdev_internal_ops ox03a_subdev_internal_ops = {
	.open = ox03a_open,
};

static int ox03a_dump_regs(struct ox03a *priv, struct seq_file *m)
{
	static const struct {
		unsigned int start;
		unsigned int end;
	} registers[] = {
		{0x0100, 0x010b},
		{0x0300, 0x032c},
		{0x0400, 0x041a},
		{0x2800, 0x2803},
		{0x3000, 0x303f},
		{0x3100, 0x3109},
		{0x3180, 0x3194},
		{0x3200, 0x3221},
		{0x3501, 0x3522},
		{0x3541, 0x3562},
		{0x3581, 0x35a2},
		{0x3600, 0x3687},
		{0x3800, 0x3883},
		{0x3b40, 0x3b53},
		{0x3b80, 0x3b99},
		{0x3d80, 0x3da4},
		{0x3e00, 0x3e1f},
		{0x3f00, 0x3f0f},
		{0x4000, 0x40cf},
		{0x4200, 0x4208},
		{0x4500, 0x4508},
		{0x4580, 0x45a6},
		{0x4600, 0x460e},
		{0x4800, 0x4870},
		{0x4880, 0x4886},
		{0x4900, 0x4903},
		{0x4d00, 0x4d24},
		{0x4f00, 0x4f07},
		{0x5000, 0x5079},
		{0x5080, 0x50ad},
		{0x50c0, 0x50ed},
		{0x5100, 0x512d},
		{0x5180, 0x5191},
		{0x51a0, 0x51b1},
		{0x51c0, 0x51d1},
		{0x5200, 0x5229},
		{0x5280, 0x52a9},
		{0x5300, 0x5329},
		{0x5380, 0x53d5},
		{0x5400, 0x5455},
		{0x5480, 0x54d5},
		{0x5600, 0x561b},
		{0x5640, 0x565b},
		{0x5680, 0x569b},
		{0x5700, 0x5715},
		{0x5740, 0x5745},
		{0x5780, 0x5795},
		{0x5800, 0x5819},
	};
	unsigned int i, j;
	int ret;
	u8 val;

	for (i = 0; i < ARRAY_SIZE(registers); i++) {
		for (j = registers[i].start; j <= registers[i].end; j++) {
			ret = ox03a_read_reg(priv->s_data, j, &val);
			if (ret)
				return -EINVAL;

			seq_printf(m, "0x%04x: 0x%02x\n", j, val);
		}
	}

	return 0;
}

static int ox03a_dump_regs_show(struct seq_file *m, void *private)
{
	struct ox03a *priv = m->private;

	return ox03a_dump_regs(priv, m);
}
DEFINE_SHOW_ATTRIBUTE(ox03a_dump_regs);

static ssize_t ox03a_debugfs_read_reg(struct file *file, char __user *userbuf,
				      size_t count, loff_t *ppos)
{
	struct ox03a *priv = file->private_data;
	u8 val = 0;
	int ret;

	if (*ppos > 0)
		return simple_read_from_buffer(userbuf, count, ppos,
					       priv->read_buf,
					       priv->read_buf_len);

	ret = ox03a_read_reg(priv->s_data, priv->cached_reg_addr, &val);
	if (ret) {
		dev_err(priv->s_data->dev, "%s: read failed\n", __func__);
		return ret;
	}

	priv->read_buf_len = snprintf(priv->read_buf,
				      sizeof(priv->read_buf),
				      "0x%02X\n", val);

	return simple_read_from_buffer(userbuf, count, ppos,
				       priv->read_buf,
				       priv->read_buf_len);
}

static ssize_t ox03a_debugfs_write_reg(struct file *file,
				       const char __user *userbuf,
				       size_t count, loff_t *ppos)
{
	struct ox03a *priv = file->private_data;
	unsigned reg, val;
	char buf[80];
	int ret;

	count = min_t(size_t, count, (sizeof(buf)-1));
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;

	buf[count] = 0;

	ret = sscanf(buf, "%i %i", &reg, &val);

	if (ret != 1 && ret != 2)
		return -EINVAL;

	priv->cached_reg_addr = reg;

	if (ret == 1)
		return count;

	ret = ox03a_write_reg(priv->s_data, reg, val);
	if (ret) {
		dev_err(priv->s_data->dev, "%s: write failed\n", __func__);
		return ret;
	}

	return count;
}

static const struct file_operations ox03a_reg_fops = {
	.open = simple_open,
	.read = ox03a_debugfs_read_reg,
	.write = ox03a_debugfs_write_reg,
};

static int ox03a_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct tegracam_device *tc_dev;
	struct ox03a *priv;
	int err;

	dev_dbg(dev, "probing v4l2 sensor at addr 0x%0x\n", client->addr);

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return -EINVAL;

	priv = devm_kzalloc(dev,
			sizeof(struct ox03a), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	tc_dev = devm_kzalloc(dev,
			sizeof(struct tegracam_device), GFP_KERNEL);
	if (!tc_dev)
		return -ENOMEM;

	priv->i2c_client = tc_dev->client = client;
	tc_dev->dev = dev;
	strncpy(tc_dev->name, "ox03a", sizeof(tc_dev->name));
	tc_dev->dev_regmap_config = &sensor_regmap_config;
	tc_dev->sensor_ops = &ox03a_common_ops;
	tc_dev->v4l2sd_internal_ops = &ox03a_subdev_internal_ops;
	tc_dev->tcctrl_ops = &ox03a_ctrl_ops;

	priv->debugfs_root = debugfs_create_dir(dev_name(&client->dev), NULL);
	debugfs_create_file("dump_regs", 0600, priv->debugfs_root, priv,
			    &ox03a_dump_regs_fops);
	debugfs_create_file("reg", 0600, priv->debugfs_root, priv,
			    &ox03a_reg_fops);

	err = tegracam_device_register(tc_dev);
	if (err) {
		dev_err(dev, "tegra camera driver registration failed\n");
		return err;
	}
	priv->tc_dev = tc_dev;
	priv->s_data = tc_dev->s_data;
	priv->subdev = &tc_dev->s_data->subdev;
	tegracam_set_privdata(tc_dev, (void *)priv);

	err = tegracam_v4l2subdev_register(tc_dev, true);
	if (err) {
		dev_err(dev, "tegra camera subdev registration failed\n");
		return err;
	}

	dev_dbg(dev, "detected ox03a sensor\n");

	return 0;
}

static int ox03a_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct ox03a *priv = (struct ox03a *)s_data->priv;

	tegracam_v4l2subdev_unregister(priv->tc_dev);
	tegracam_device_unregister(priv->tc_dev);

	return 0;
}

static const struct i2c_device_id ox03a_id[] = {
	{ "ox03a", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ox03a_id);

static struct i2c_driver ox03a_i2c_driver = {
	.driver = {
		.name = "ox03a",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ox03a_of_match),
	},
	.probe = ox03a_probe,
	.remove = ox03a_remove,
	.id_table = ox03a_id,
};
module_i2c_driver(ox03a_i2c_driver);

MODULE_DESCRIPTION("V4L2 Sensor Driver for OmniVision OX03A");
MODULE_AUTHOR("Analog Devices Inc.");
MODULE_LICENSE("GPL v2");
