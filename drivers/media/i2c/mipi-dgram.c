/*
 * mipi_dgram.c - mipi_dgram sensor driver
 *
 * Copyright (c) 2015-2019, NVIDIA CORPORATION.  All rights reserved.
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
#define DEBUG

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
#include <media/max9295.h>
#include <media/max9296.h>

#include "../platform/tegra/camera/camera_gpio.h"

static const struct of_device_id mipi_dgram_of_match[] = {
    {
        .compatible = "nvidia,mipi_dgram",
    },
    {},
};
MODULE_DEVICE_TABLE(of, mipi_dgram_of_match);

static const u32 ctrl_cid_list[] = {
    TEGRA_CAMERA_CID_GAIN,
    TEGRA_CAMERA_CID_EXPOSURE,
    TEGRA_CAMERA_CID_FRAME_RATE,
    TEGRA_CAMERA_CID_SENSOR_MODE_ID,
};

struct mipi_dgram
{
    struct i2c_client *i2c_client;
    struct v4l2_subdev *subdev;
    struct device *ser_dev;
    struct device *dser_dev;
    struct gmsl_link_ctx g_ctx;
    u16 fine_integ_time;
    u32 frame_length;
    struct camera_common_data *s_data;
    struct tegracam_device *tc_dev;
};

static inline int mipi_dgram_read_reg(struct camera_common_data *s_data,
                                      u16 addr, u16 *val)
{
    *val = 0;
    return 0;
}

static inline int mipi_dgram_write_reg(struct camera_common_data *s_data,
                                       u16 addr, u16 val)
{
    return 0;
}

static inline int mipi_dgram_read_reg_u8(struct camera_common_data *s_data,
                                         u16 addr, u8 *val)
{
    *val = 0;
    return 0;
}

static int mipi_dgram_write_reg_u8(struct camera_common_data *s_data,
                                   u16 addr, u8 val)
{
    return 0;
}

static int mipi_dgram_write_table(struct mipi_dgram *priv, const mipi_dgram_reg table[])
{
    return 0;
}

static struct mutex serdes_lock__;

extern int max9295_write_reg(struct device *dev, u16 addr, u8 val);

static int mipi_dgram_gmsl_serdes_setup(struct mipi_dgram *priv)
{
    int err = 0;
    int des_err = 0;
    struct device *dev;
    /*	const unsigned short GPIO_REG_A = 0x2be;*/

    if (!priv || !priv->ser_dev || !priv->dser_dev || !priv->i2c_client)
        return -EINVAL;

    dev = &priv->i2c_client->dev;

    mutex_lock(&serdes_lock__);

    /* For now no separate power on required for serializer device */
    max9296_power_on(priv->dser_dev);

    /* setup serdes addressing and control pipeline */
    err = max9296_setup_link(priv->dser_dev, &priv->i2c_client->dev);
    if (err)
    {
        dev_err(dev, "gmsl deserializer link config failed\n");
        goto error;
    }

    err = max9295_setup_control(priv->ser_dev);
    /* proceed even if ser setup failed, to setup deser correctly */
    if (err)
        dev_err(dev, "gmsl serializer setup failed\n");

    des_err = max9296_setup_control(priv->dser_dev, &priv->i2c_client->dev);
    if (des_err)
    {
        dev_err(dev, "gmsl deserializer setup failed\n");
        /* overwrite err only if deser setup also failed */
        err = des_err;
    }

error:
    mutex_unlock(&serdes_lock__);
    return err;
}

static void mipi_dgram_gmsl_serdes_reset(struct mipi_dgram *priv)
{
    mutex_lock(&serdes_lock__);

    /* reset serdes addressing and control pipeline */
    max9295_reset_control(priv->ser_dev);
    max9296_reset_control(priv->dser_dev, &priv->i2c_client->dev);

    max9296_power_off(priv->dser_dev);

    mutex_unlock(&serdes_lock__);
}

static int mipi_dgram_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
    /* mipi_dgram does not support group hold */
    return 0;
}

static int mipi_dgram_set_gain(struct tegracam_device *tc_dev, s64 val)
{
    return 0;
}

static int mipi_dgram_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
    return 0;
}

static int mipi_dgram_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
    return 0;
}

static struct tegracam_ctrl_ops mipi_dgram_ctrl_ops = {
    .numctrls = ARRAY_SIZE(ctrl_cid_list),
    .ctrl_cid_list = ctrl_cid_list,
    .set_gain = mipi_dgram_set_gain,
    .set_exposure = mipi_dgram_set_exposure,
    .set_frame_rate = mipi_dgram_set_frame_rate,
    .set_group_hold = mipi_dgram_set_group_hold,
};

static int mipi_dgram_power_on(struct camera_common_data *s_data)
{
    return 0;
}

static int mipi_dgram_power_off(struct camera_common_data *s_data)
{
    return 0;
}

static int mipi_dgram_power_put(struct tegracam_device *tc_dev)
{
    return 0;
}

static int mipi_dgram_power_get(struct tegracam_device *tc_dev)
{
    return;
}

static struct camera_common_pdata *mipi_dgram_parse_dt(
    struct tegracam_device *tc_dev)
{
    struct device *dev = tc_dev->dev;
    struct device_node *np = dev->of_node;
    struct mipi_dgram *priv;
    struct camera_common_pdata *board_priv_pdata;
    const struct of_device_id *match;
    struct camera_common_pdata *ret = NULL;
    int err = 0;
    int gpio;

    if (!np)
        return NULL;

    match = of_match_device(mipi_dgram_of_match, dev);
    if (!match)
    {
        dev_err(dev, "Failed to find matching dt id\n");
        return NULL;
    }

    board_priv_pdata = devm_kzalloc(dev,
                                    sizeof(*board_priv_pdata), GFP_KERNEL);
    if (!board_priv_pdata)
        return NULL;

    mipi_dgram_priv = (struct mipi_dgram *)tegracam_get_privdata(tc_dev);

    return board_priv_pdata;
}

static int mipi_dgram_set_mode(struct tegracam_device *tc_dev)
{
    return 0;
}

static int mipi_dgram_start_streaming(struct tegracam_device *tc_dev)
{
    struct mipi_dgram *priv = (struct mipi_dgram *)tegracam_get_privdata(tc_dev);
    struct device *dev = tc_dev->dev;
    int err;

    /* enable serdes streaming */
    err = max9295_setup_streaming(priv->ser_dev);
    if (err)
        goto exit;
    err = max9296_setup_streaming(priv->dser_dev, dev);
    if (err)
        goto exit;
    err = max9296_start_streaming(priv->dser_dev, dev);
    if (err)
        goto exit;

    dev_dbg(dev, "%s starting streaming\n", __func__);

    msleep(20);

    return 0;

exit:
    dev_err(dev, "%s: error setting stream\n", __func__);

    return err;
}

static int mipi_dgram_stop_streaming(struct tegracam_device *tc_dev)
{
    int err;
    struct device *dev = tc_dev->dev;
    struct mipi_dgram *priv = (struct mipi_dgram *)tegracam_get_privdata(tc_dev);

    /* disable serdes streaming */
    max9296_stop_streaming(priv->dser_dev, dev);

    usleep_range(50000, 51000);

    return err;
}

static struct camera_common_sensor_ops mipi_dgram_common_ops = {
    .numfrmfmts = ARRAY_SIZE(mipi_dgram_frmfmt),
    .frmfmt_table = mipi_dgram_frmfmt,
    .power_on = mipi_dgram_power_on,
    .power_off = mipi_dgram_power_off,
    .write_reg = mipi_dgram_write_reg_u8,
    .read_reg = mipi_dgram_read_reg_u8,
    .parse_dt = mipi_dgram_parse_dt,
    .power_get = mipi_dgram_power_get,
    .power_put = mipi_dgram_power_put,
    .set_mode = mipi_dgram_set_mode,
    .start_streaming = mipi_dgram_start_streaming,
    .stop_streaming = mipi_dgram_stop_streaming,
};

static int mipi_dgram_board_setup(struct mipi_dgram *priv)
{
    struct tegracam_device *tc_dev = priv->tc_dev;
    struct device *dev = tc_dev->dev;
    struct device_node *node = dev->of_node;
    struct device_node *ser_node;
    struct i2c_client *ser_i2c = NULL;
    struct device_node *dser_node;
    struct i2c_client *dser_i2c = NULL;
    struct device_node *gmsl;
    int value = 0xFFFF;
    const char *str_value;
    const char *str_value1[2];
    int i;
    int err;

    err = of_property_read_u32(node, "reg", &priv->g_ctx.sdev_reg);
    if (err < 0)
    {
        dev_err(dev, "reg not found\n");
        goto error;
    }

    err = of_property_read_u32(node, "def-addr",
                               &priv->g_ctx.sdev_def);
    if (err < 0)
    {
        dev_err(dev, "def-addr not found\n");
        goto error;
    }

    ser_node = of_parse_phandle(node, "nvidia,gmsl-ser-device", 0);
    if (ser_node == NULL)
    {
        dev_err(dev,
                "missing %s handle\n",
                "nvidia,gmsl-ser-device");
        goto error;
    }

    err = of_property_read_u32(ser_node, "reg", &priv->g_ctx.ser_reg);
    if (err < 0)
    {
        dev_err(dev, "serializer reg not found\n");
        goto error;
    }

    ser_i2c = of_find_i2c_device_by_node(ser_node);
    of_node_put(ser_node);

    if (ser_i2c == NULL)
    {
        dev_err(dev, "missing serializer dev handle\n");
        goto error;
    }
    if (ser_i2c->dev.driver == NULL)
    {
        dev_err(dev, "missing seriailzer addr %d\n", ser_i2c->addr);
        dev_err(dev, "missing serializer driver - ask for reprobe?\n");
        // return -EPROBE_DEFER;
        goto error;
    }

    priv->ser_dev = &ser_i2c->dev;

    dser_node = of_parse_phandle(node, "nvidia,gmsl-dser-device", 0);
    if (dser_node == NULL)
    {
        dev_err(dev,
                "missing %s handle\n",
                "nvidia,gmsl-dser-device");
        goto error;
    }

    dser_i2c = of_find_i2c_device_by_node(dser_node);
    of_node_put(dser_node);

    if (dser_i2c == NULL)
    {
        dev_err(dev, "missing deserializer dev handle\n");
        goto error;
    }
    if (dser_i2c->dev.driver == NULL)
    {
        dev_err(dev, "missing deserializer driver - ask for reprobe?\n");
        // return -EPROBE_DEFER;
        goto error;
    }

    priv->dser_dev = &dser_i2c->dev;

    /* populate g_ctx from DT */
    gmsl = of_get_child_by_name(node, "gmsl-link");
    if (gmsl == NULL)
    {
        dev_err(dev, "missing gmsl-link device node\n");
        err = -EINVAL;
        goto error;
    }

    err = of_property_read_string(gmsl, "dst-csi-port", &str_value);
    if (err < 0)
    {
        dev_err(dev, "No dst-csi-port found\n");
        goto error;
    }
    priv->g_ctx.dst_csi_port =
        (!strcmp(str_value, "a")) ? GMSL_CSI_PORT_A : GMSL_CSI_PORT_B;

    err = of_property_read_string(gmsl, "src-csi-port", &str_value);
    if (err < 0)
    {
        dev_err(dev, "No src-csi-port found\n");
        goto error;
    }
    priv->g_ctx.src_csi_port =
        (!strcmp(str_value, "a")) ? GMSL_CSI_PORT_A : GMSL_CSI_PORT_B;

    err = of_property_read_string(gmsl, "csi-mode", &str_value);
    if (err < 0)
    {
        dev_err(dev, "No csi-mode found\n");
        goto error;
    }

    if (!strcmp(str_value, "1x4"))
    {
        priv->g_ctx.csi_mode = GMSL_CSI_1X4_MODE;
    }
    else if (!strcmp(str_value, "2x4"))
    {
        priv->g_ctx.csi_mode = GMSL_CSI_2X4_MODE;
    }
    else if (!strcmp(str_value, "4x2"))
    {
        priv->g_ctx.csi_mode = GMSL_CSI_4X2_MODE;
    }
    else if (!strcmp(str_value, "2x2"))
    {
        priv->g_ctx.csi_mode = GMSL_CSI_2X2_MODE;
    }
    else
    {
        dev_err(dev, "invalid csi mode\n");
        goto error;
    }

    err = of_property_read_string(gmsl, "serdes-csi-link", &str_value);
    if (err < 0)
    {
        dev_err(dev, "No serdes-csi-link found\n");
        goto error;
    }
    priv->g_ctx.serdes_csi_link =
        (!strcmp(str_value, "a")) ? GMSL_SERDES_CSI_LINK_A : GMSL_SERDES_CSI_LINK_B;

    err = of_property_read_u32(gmsl, "st-vc", &value);
    if (err < 0)
    {
        dev_err(dev, "No st-vc info\n");
        goto error;
    }
    priv->g_ctx.st_vc = value;

    err = of_property_read_u32(gmsl, "vc-id", &value);
    if (err < 0)
    {
        dev_err(dev, "No vc-id info\n");
        goto error;
    }
    priv->g_ctx.dst_vc = value;

    err = of_property_read_u32(gmsl, "num-lanes", &value);
    if (err < 0)
    {
        dev_err(dev, "No num-lanes info\n");
        goto error;
    }
    priv->g_ctx.num_csi_lanes = value;

    priv->g_ctx.num_streams =
        of_property_count_strings(gmsl, "streams");
    if (priv->g_ctx.num_streams <= 0)
    {
        dev_err(dev, "No streams found\n");
        err = -EINVAL;
        goto error;
    }

    for (i = 0; i < priv->g_ctx.num_streams; i++)
    {
        of_property_read_string_index(gmsl, "streams", i,
                                      &str_value1[i]);
        if (!str_value1[i])
        {
            dev_err(dev, "invalid stream info\n");
            goto error;
        }
        if (!strcmp(str_value1[i], "raw12"))
        {
            priv->g_ctx.streams[i].st_data_type =
                GMSL_CSI_DT_RAW_12;
        }
        else if (!strcmp(str_value1[i], "embed"))
        {
            priv->g_ctx.streams[i].st_data_type =
                GMSL_CSI_DT_EMBED;
        }
        else if (!strcmp(str_value1[i], "ued-u1"))
        {
            priv->g_ctx.streams[i].st_data_type =
                GMSL_CSI_DT_UED_U1;
        }
        else
        {
            dev_err(dev, "invalid stream data type\n");
            goto error;
        }
    }

    priv->g_ctx.s_dev = dev;

    dev_dbg(dev, "board setup complete\n");

    return 0;

error:
    dev_err(dev, "board setup failed\n");
    return err;
}

static int mipi_dgram_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);

    dev_dbg(&client->dev, "%s:\n", __func__);

    return 0;
}

static const struct v4l2_subdev_internal_ops mipi_dgram_subdev_internal_ops = {
    .open = mipi_dgram_open,
};

static int mipi_dgram_probe(struct i2c_client *client,
                            const struct i2c_device_id *id)
{
    struct device *dev = &client->dev;
    struct tegracam_device *tc_dev;
    struct mipi_dgram *priv;
    int err;

    dev_dbg(dev, "probing v4l2 sensor at addr 0x%0x\n", client->addr);

    if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
        return -EINVAL;

    priv = devm_kzalloc(dev,
                        sizeof(struct mipi_dgram), GFP_KERNEL);
    if (!priv)
        return -ENOMEM;

    tc_dev = devm_kzalloc(dev,
                          sizeof(struct tegracam_device), GFP_KERNEL);
    if (!tc_dev)
        return -ENOMEM;

    priv->i2c_client = tc_dev->client = client;
    tc_dev->dev = dev;
    strncpy(tc_dev->name, "mipi_dgram", sizeof(tc_dev->name));
    tc_dev->dev_regmap_config = &sensor_regmap_config;
    tc_dev->sensor_ops = &mipi_dgram_common_ops;
    tc_dev->v4l2sd_internal_ops = &mipi_dgram_subdev_internal_ops;
    tc_dev->tcctrl_ops = &mipi_dgram_ctrl_ops;

    err = tegracam_device_register(tc_dev);
    if (err)
    {
        dev_err(dev, "tegra camera driver registration failed\n");
        return err;
    }
    priv->tc_dev = tc_dev;
    priv->s_data = tc_dev->s_data;
    priv->subdev = &tc_dev->s_data->subdev;
    tegracam_set_privdata(tc_dev, (void *)priv);

    err = mipi_dgram_board_setup(priv);
    if (err)
    {
        tegracam_device_unregister(tc_dev);
        dev_err(dev, "board setup failed\n");
        return err;
    }

    /* Pair sensor to serializer dev */
    err = max9295_sdev_pair(priv->ser_dev, &priv->g_ctx);
    if (err)
    {
        tegracam_device_unregister(tc_dev);
        dev_err(&client->dev, "gmsl ser pairing failed\n");
        return err;
    }

    /* Register sensor to deserializer dev */
    err = max9296_sdev_register(priv->dser_dev, &priv->g_ctx);
    if (err)
    {
        tegracam_device_unregister(tc_dev);
        dev_err(&client->dev, "gmsl deserializer register failed\n");
        return err;
    }

    /*
     * gmsl serdes setup
     *
     * Sensor power on/off should be the right place for serdes
     * setup/reset. But the problem is, the total required delay
     * in serdes setup/reset exceeds the frame wait timeout, looks to
     * be related to multiple channel open and close sequence
     * issue (#BUG 200477330).
     * Once this bug is fixed, these may be moved to power on/off.
     * The delays in serdes is as per guidelines and can't be reduced,
     * so it is placed in probe/remove, though for that, deserializer
     * would be powered on always post boot, until 1.2v is supplied
     * to deserializer from CVB.
     */
    err = mipi_dgram_gmsl_serdes_setup(priv);
    if (err)
    {
        tegracam_device_unregister(tc_dev);
        dev_err(&client->dev,
                "%s gmsl serdes setup failed\n", __func__);
        return err;
    }

    /* Try to read the part ID */
    {
        unsigned int chip_id;
        err = regmap_read(priv->s_data->regmap, 0x3000, &chip_id);
        if (err)
        {
            tegracam_device_unregister(tc_dev);
            dev_err(dev, "unable to read sensor chip revision\n");
            return err;
        }

        dev_err(dev, "Read sensor chip ID %x\n", chip_id);
    }

    err = tegracam_v4l2subdev_register(tc_dev, true);
    if (err)
    {
        tegracam_device_unregister(tc_dev);
        dev_err(dev, "tegra camera subdev registration failed\n");
        return err;
    }

    dev_dbg(dev, "detected mipi_dgram sensor\n");

    return 0;
}

static int mipi_dgram_remove(struct i2c_client *client)
{
    struct camera_common_data *s_data = to_camera_common_data(&client->dev);
    struct mipi_dgram *priv = (struct mipi_dgram *)s_data->priv;

    mipi_dgram_gmsl_serdes_reset(priv);

    tegracam_v4l2subdev_unregister(priv->tc_dev);
    tegracam_device_unregister(priv->tc_dev);

    return 0;
}

static const struct i2c_device_id mipi_dgram_id[] = {
    {"mipi_dgram", 0},
    {}};
MODULE_DEVICE_TABLE(i2c, mipi_dgram_id);

static struct i2c_driver mipi_dgram_i2c_driver = {
    .driver = {
        .name = "mipi_dgram",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(mipi_dgram_of_match),
    },
    .probe = mipi_dgram_probe,
    .remove = mipi_dgram_remove,
    .id_table = mipi_dgram_id,
};
module_i2c_driver(mipi_dgram_i2c_driver);

static int __init mipi_dgram_init(void)
{
    mutex_init(&serdes_lock__);

    return i2c_add_driver(&mipi_dgram_i2c_driver);
}

static void __exit mipi_dgram_exit(void)
{
    mutex_destroy(&serdes_lock__);
}

module_init(mipi_dgram_init);
module_exit(mipi_dgram_exit);

MODULE_DESCRIPTION("Media Controller driver for mipi_dgram");
MODULE_AUTHOR("Recogni Corporation from NVidia sources");
MODULE_LICENSE("GPL v2");
