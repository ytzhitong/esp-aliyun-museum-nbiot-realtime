/*
 * Copyright (C) 2015-2018 Alibaba Group Holding Limited
 */
#ifndef __APP_ENTRY_H__
#define __APP_ENTRY_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#define debug 0

#define NVS_KEY_WIFI_CONFIG "wifi_config"

#define net_mode     2  //0:wifi;1:lora;2:nb_iot.

#define publish_gap 60000000
#define waitnet_gap 10

typedef struct
{
    int argc;
    char **argv;
}app_main_paras_t;

int linkkit_main(void *paras);

void set_iotx_info();

void sensor_task(void *pvParameter);

#endif
