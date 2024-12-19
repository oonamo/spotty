#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"

#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatt_defs.h"
#include "esp_gatts_api.h"

#include "esp_err.h"
#include "esp_gap_bt_api.h"
#include "esp_log.h"

#include "esp_log_buffer.h"
#include "esp_spp_api.h"

#include "nvs.h"
#include "nvs_flash.h"

#define SPP_SERVER_NAME "SPOTTY_SERVICER"
#define DEVICE_NAME "SPOTTY"
#define BDA_SIZE 18

#define APP_ID 0x08c

#define TAG "spotty"

typedef struct
{
    bool connected;
} peer_device_t;

static peer_device_t m_dev = {
    .connected = false,
};

static uint8_t service_uuid[16] = {
    /* LSB
       <-------------------------------------------------------------------------------->
       MSB */
    // first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006,
    .max_interval = 0x0C80,
    .appearance = ESP_BLE_APPEARANCE_GENERIC_REMOTE,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT)};

static esp_ble_adv_params_t adv_param = {
    .adv_int_min = 0x0800,
    .adv_int_max = 0x0800,
    .adv_type = ADV_TYPE_IND,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static const esp_gatts_attr_db_t gatt_db[] = {};

static char *bda2str(uint8_t *bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < BDA_SIZE)
    {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x", p[0], p[1], p[2], p[3], p[4],
            p[5]);
    return str;
}

static void esp_gatts_cb(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                         esp_ble_gatts_cb_param_t *param)
{
    char bda_str[18] = {0};
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
    {
        ESP_LOGI(TAG, "REG_EVT");
        esp_err_t dev_name_ret = esp_ble_gap_set_device_name(DEVICE_NAME);
        if (dev_name_ret)
        {
            ESP_LOGE(TAG, "set device name failed, code: %x", dev_name_ret);
        }

        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret)
        {
            ESP_LOGE(TAG, "config adv data failed, code: %x", ret);
        }
        break;
    }
    case ESP_GATTS_READ_EVT:
        ESP_LOGI(TAG, "READ EVENT");
        break;
    case ESP_GATTS_CONNECT_EVT:
        m_dev.connected = true;
        ESP_LOGI(TAG, "Device connected!, conn_id: %d", param->connect.conn_id);
        ESP_LOGI(TAG, "str_bda: %s",
                 bda2str(param->connect.remote_bda, bda_str, 18));
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda,
               sizeof(esp_bd_addr_t));
        conn_params.latency = 0;
        conn_params.max_int = 0x20; // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10; // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;  // timeout = 400*10ms = 4000ms
        // start sent the update connection parameters to the peer device.
        esp_ble_gap_update_conn_params(&conn_params);
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        m_dev.connected = false;
        ESP_LOGW(TAG, "Peer device was disconnected, reason: %d",
                 param->disconnect.reason);
        esp_ble_gap_start_advertising(&adv_param);
        break;
    default:
        ESP_LOGI(TAG, "GATTS event: %d", event);
        break;
    }
}

static esp_err_t bluedroid_init()
{
    esp_err_t ret;
    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(TAG, "%s init bluetooth failed: %s", __func__,
                 esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(TAG, "%s enable bluetooth failed: %s", __func__,
                 esp_err_to_name(ret));
        return ret;
    }

    return ret;
}

static esp_err_t controller_init()
{
    esp_err_t ret;
    esp_bt_controller_config_t bt_cntrl_cfg =
        BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cntrl_cfg);
    if (ret)
    {
        ESP_LOGE(TAG, "%s failed to init contoller", __func__);
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(TAG, "%s failed to enable controller with BLE", __func__);
        return ret;
    }
    return ret;
}

void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "Starting advertisment");
        if (!m_dev.connected)
        {
            esp_ble_gap_start_advertising(&adv_param);
            m_dev.connected = true;
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(TAG, "Advertising task failed: %s",
                     esp_err_to_name(param->adv_start_cmpl.status));
        }
        else
        {
            ESP_LOGI(TAG, "started advertising");
        }
        break;
    default:
        ESP_LOGI(TAG, "Gap event: %d", event);
        break;
    }
}
void app_main(void)
{
    esp_err_t ret;
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    ret = controller_init();
    if (ret)
    {
        ESP_LOGE(TAG, "%s could not init controller", __func__);
        return;
    }

    ret = bluedroid_init();
    if (ret)
    {
        ESP_LOGE(TAG, "%s could not init bluedroid", __func__);
        return;
    }

    ret = esp_ble_gatts_register_callback(esp_gatts_cb);
    if (ret)
    {
        ESP_LOGE(TAG, "%s could not register gatts cb, code: %x", __func__,
                 ret);
        return;
    }
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret)
    {
        ESP_LOGE(TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(APP_ID);
    if (ret)
    {
        ESP_LOGE(TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret)
    {
        ESP_LOGE(TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
}
