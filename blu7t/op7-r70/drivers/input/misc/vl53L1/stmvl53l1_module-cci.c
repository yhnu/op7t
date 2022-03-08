/*
* Copyright (c) 2016, STMicroelectronics - All Rights Reserved
*
* License terms: BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * @file stmvl53l1_module-i2c.c
 *
 *  implement STM VL53L1 module interface i2c wrapper + control
 *  using linux native i2c + gpio + reg api
 */
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <linux/version.h>

/*
 * power specific includes
 */
#include <linux/pwm.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/clk.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>

#include "stmvl53l1-i2c.h"
#include "stmvl53l1.h"
struct stmvl53l1_pinctrl_info {
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
};

struct stmvl53l1_pinctrl_info stmvl53l1_pinctrl;

extern int stmvl53l1_parse_tree(struct device *dev, struct i2c_data *i2c_data);
extern void stmvl53l1_release_gpios(struct i2c_data *i2c_data);

static int stmvl53l1_request_pinctrl(struct device *dev)
{
    struct stmvl53l1_pinctrl_info *device_pctrl = &stmvl53l1_pinctrl;
	device_pctrl->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(device_pctrl->pinctrl)) {
		vl53l1_errmsg("Pinctrl not available");
		device_pctrl->pinctrl = NULL;
		return 0;
	}
	device_pctrl->gpio_state_active =
		pinctrl_lookup_state(device_pctrl->pinctrl,
				"laser_default");
	if (IS_ERR_OR_NULL(device_pctrl->gpio_state_active)) {
		vl53l1_errmsg("Failed to get the active state pinctrl handle");
		device_pctrl->gpio_state_active = NULL;
		return -EINVAL;
	}
	device_pctrl->gpio_state_suspend
		= pinctrl_lookup_state(device_pctrl->pinctrl,
				"laser_suspend");
	if (IS_ERR_OR_NULL(device_pctrl->gpio_state_suspend)) {
		vl53l1_errmsg("Failed to get the suspend state pinctrl handle");
		device_pctrl->gpio_state_suspend = NULL;
		return -EINVAL;
	}
	return 0;
}

int stmvl53l1_enable_pinctrl(void)
{
    int rc = 0;

    if (stmvl53l1_pinctrl.pinctrl &&
        stmvl53l1_pinctrl.gpio_state_active) {
        rc = pinctrl_select_state(stmvl53l1_pinctrl.pinctrl,
            stmvl53l1_pinctrl.gpio_state_active);
        vl53l1_errmsg("enable pinctrl rc=%d\n", rc);
    }

    return rc;
}

int stmvl53l1_disable_pinctrl(void)
{
    int rc = 0;

    if (stmvl53l1_pinctrl.pinctrl &&
        stmvl53l1_pinctrl.gpio_state_suspend) {
        rc = pinctrl_select_state(stmvl53l1_pinctrl.pinctrl,
            stmvl53l1_pinctrl.gpio_state_suspend);
        vl53l1_errmsg("disable pinctrl rc=%d\n", rc);
    }

    return rc;

}

int stmvl53l1_release_pinctrl(struct device *dev)
{
    if (stmvl53l1_pinctrl.pinctrl)
        devm_pinctrl_put(stmvl53l1_pinctrl.pinctrl);
    stmvl53l1_pinctrl.pinctrl = NULL;

    return 0;
}

static int32_t stmvl53l1_probe_cci(
	struct platform_device *pdev)
{
    int rc = 0;
    struct stmvl53l1_data *vl53l1_data = NULL;
    struct i2c_data *i2c_data = NULL;

    vl53l1_dbgmsg("Enter %s : 0x%02x\n", pdev->name, pdev->id);


    vl53l1_data = kzalloc(sizeof(struct stmvl53l1_data), GFP_KERNEL);
    if (!vl53l1_data) {
        rc = -ENOMEM;
        return rc;
    }
    if (vl53l1_data) {
        vl53l1_data->client_object =
                kzalloc(sizeof(struct i2c_data), GFP_KERNEL);
        if (!vl53l1_data)
            goto done_freemem;
        i2c_data = (struct i2c_data *)vl53l1_data->client_object;
    }
    //i2c_data->client = client;
    i2c_data->vl53l1_data = vl53l1_data;
    i2c_data->irq = -1 ;
    rc = stmvl53l1_parse_tree(&(pdev->dev), i2c_data);
    if (rc)
        goto done_freemem;

    rc = stmvl53l1_request_pinctrl(&(pdev->dev));
    if (rc){
        vl53l1_errmsg("fail to request pinctrl,rc = %d", rc);
        goto done_freemem;
    }
    rc = stmvl53l1_enable_pinctrl();
    if (rc){
        vl53l1_errmsg("fail to enable pinctrl,rc = %d", rc);
        goto release_gpios;
    }

    platform_set_drvdata(pdev, vl53l1_data);
    //vl53l1_data->plat_dev = pdev;

    rc = stmvl53l1_setup(vl53l1_data);
    if (rc)
        goto release_gpios;
    vl53l1_dbgmsg("End\n");

    kref_init(&i2c_data->ref);

    return rc;

release_gpios:
    stmvl53l1_release_gpios(i2c_data);

done_freemem:
    kfree(vl53l1_data);
    kfree(i2c_data);

    return 0;
}

static int32_t stmvl53l1_remove_cci(struct platform_device *pdev)
{
    int rc = 0;
    struct stmvl53l1_data *data = platform_get_drvdata(pdev);
    struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;

    vl53l1_dbgmsg("Enter\n");
    mutex_lock(&data->work_mutex);
    /* main driver cleanup */
    stmvl53l1_cleanup(data);

    rc = stmvl53l1_disable_pinctrl();
    if (rc){
        vl53l1_errmsg("fatal,fail to disable pinctrl,rc = %d", rc);
    }
    rc = stmvl53l1_release_pinctrl(&(pdev->dev));
    if (rc){
        vl53l1_errmsg("fatal,fail to release pinctrl,rc = %d", rc);
    }

    /* release gpios */
    stmvl53l1_release_gpios(i2c_data);

    mutex_unlock(&data->work_mutex);

    stmvl53l1_put(data->client_object);

    vl53l1_dbgmsg("End\n");

    return 0;
}

static const struct of_device_id stmvl53l1_driver_dt_match[] = {
	{.compatible = "st,stmvl53l1"},
	{}
};

MODULE_DEVICE_TABLE(of, stmvl53l1_driver_dt_match);

static struct platform_driver stmvl53l1_platform_driver = {
	.probe = stmvl53l1_probe_cci,
	.driver = {
		.name = "st,stmvl53l1",
		.owner = THIS_MODULE,
		.of_match_table = stmvl53l1_driver_dt_match,
	},
	.remove = stmvl53l1_remove_cci,
};

int __init stmvl53l1_init_cci(void)
{
	int32_t rc = 0;
    vl53l1_dbgmsg("enter\n");
	rc = platform_driver_register(&stmvl53l1_platform_driver);
	if (rc < 0) {
        vl53l1_dbgmsg("platform_driver_register fail\n");
		return rc;
	}

	return rc;
}

void __exit stmvl53l1_exit_cci(void* obj)
{
    vl53l1_dbgmsg("enter\n");
	platform_driver_unregister(&stmvl53l1_platform_driver);
}

