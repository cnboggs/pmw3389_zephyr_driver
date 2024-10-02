//
// Created by jonasotto on 9/15/22.
//
// Updated by cnboggs
//
// Datasheet:
// https://gzhls.at/blob/ldb/a/0/f/6/3971c1b0ff98ce53e924c6ce3ffb16905172.pdf

#define DT_DRV_COMPAT pixart_pmw3389

#include "pmw3389.h"

#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>

LOG_MODULE_REGISTER(pmw3389, LOG_LEVEL_DBG);

int pmw3389_init(const struct device *dev)
{
	LOG_INF("Initializing PMW3389");
	LOG_INF("Initializing PMW3389");
    LOG_INF("Initializing PMW3389");
    return 0;
}

static const struct sensor_driver_api pmw3389_api = {};

#define PMW3389_INIT(n)                                                                            \
    DEVICE_DT_INST_DEFINE(n, &pmw3389_init, NULL, NULL, NULL,      \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &pmw3389_api);

DT_INST_FOREACH_STATUS_OKAY(PMW3389_INIT)
