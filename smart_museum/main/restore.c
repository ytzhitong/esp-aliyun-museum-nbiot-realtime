/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS products only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

/* Reset to Factory
*/
#include <string.h>
#include <esp_log.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#include "nvs.h"
#include "restore.h"

#include "esp_spiffs.h"

#define STORAGE_NAMESPACE "storage"

static const char *TAG = "spiffs_exp";

void Spiffs_init(void)
{
    ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true
    };

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
}


esp_err_t read_PH_correct(void)
{
    nvs_handle my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    size_t required_size = sizeof(correct_para);  // value will default to 0, if not set yet in NVS
    err = nvs_get_blob(my_handle, "ph_correct", &PH_correct_para, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    if(PH_correct_para.flag!=0x55)
    {
    	PH_correct_para.flag=0x55;
    	PH_correct_para.zero=0;
    	PH_correct_para.slope=1;

    	err = nvs_set_blob(my_handle, "ph_correct", &PH_correct_para, required_size);
        if (err != ESP_OK) return err;
    }

    // Commit
    err = nvs_commit(my_handle);
    if (err != ESP_OK) return err;

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}

esp_err_t write_PH_correct(void)
{
    nvs_handle my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    size_t required_size = sizeof(correct_para);  // value will default to 0, if not set yet in NVS

	PH_correct_para.flag=0x55;

	err = nvs_set_blob(my_handle, "ph_correct", &PH_correct_para, required_size);
	if (err != ESP_OK) return err;

    // Commit
    err = nvs_commit(my_handle);
    if (err != ESP_OK) return err;

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}

esp_err_t read_devive_inf(void)
{
    nvs_handle my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    size_t required_size = sizeof(_decive_inf);  // value will default to 0, if not set yet in NVS
    err = nvs_get_blob(my_handle, "device_inf", &decive_inf, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    if(decive_inf.flag!=0x55)
    {
    	decive_inf.flag=0x55;
    	strcpy(decive_inf.name,"SUZHOU_NB_001");
    	strcpy(decive_inf.secret,"26f10ae448386c330c529103415b226e");
    	err = nvs_set_blob(my_handle, "device_inf", &decive_inf, required_size);
        if (err != ESP_OK) return err;
    }

    // Commit
    err = nvs_commit(my_handle);
    if (err != ESP_OK) return err;

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}

esp_err_t write_device_secret(void)
{
    nvs_handle my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    size_t required_size = sizeof(_decive_inf);  // value will default to 0, if not set yet in NVS

    decive_inf.flag=0x55;

	err = nvs_set_blob(my_handle, "device_inf", &decive_inf, required_size);
	if (err != ESP_OK) return err;

    // Commit
    err = nvs_commit(my_handle);
    if (err != ESP_OK) return err;

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}
