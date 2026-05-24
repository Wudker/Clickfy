#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_pm.h"
#include "esp_sleep.h"

// --- KONFIGURACJA ---
#define TAG "Clickfy_2.0"
#define PLAY_PIN 7
#define FORWARD_PIN 3
#define PREVIOUS_PIN 18
#define LED_PIN 6
#define BATTERY_PIN ADC1_CHANNEL_4

#define SLEEP_TIMEOUT_MS 15000

// Kody multimedialne HID Consumer Control
#define KEY_PLAY_PAUSE 0x00CD
#define KEY_NEXT_TRACK 0x00B5
#define KEY_PREV_TRACK 0x00B6

static uint8_t ble_addr_type;
static volatile uint16_t current_conn_handle = 0;
static uint16_t input_report_handle;
static uint16_t battery_handle;

// --- PROTOTYPY ---
void ble_app_advertise(void);
static int hid_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int ble_svc_battery_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
void on_sync(void);

// --- MAPA RAPORTÓW HID ---
static const uint8_t hid_report_map[] = {
    0x05, 0x0C, 0x09, 0x01, 0xA1, 0x01, 0x85, 0x01, 0x19, 0x00, 0x2A, 0x3C, 0x02,
    0x15, 0x00, 0x26, 0x3C, 0x02, 0x95, 0x01, 0x75, 0x10, 0x81, 0x00, 0xC0};

// --- USŁUGI GATT ---
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = BLE_UUID16_DECLARE(0x1812),
     .characteristics = (struct ble_gatt_chr_def[]){
         {.uuid = BLE_UUID16_DECLARE(0x2A4A), .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC, .access_cb = hid_access_cb, .arg = (void *)1},
         {.uuid = BLE_UUID16_DECLARE(0x2A4B), .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC, .access_cb = hid_access_cb, .arg = (void *)2},
         {.uuid = BLE_UUID16_DECLARE(0x2A4D), .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_NOTIFY, .access_cb = hid_access_cb, .arg = (void *)3, .val_handle = &input_report_handle},
         {0}}},
    {.type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = BLE_UUID16_DECLARE(0x180F), .characteristics = (struct ble_gatt_chr_def[]){{.uuid = BLE_UUID16_DECLARE(0x2A19), .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_NOTIFY, .access_cb = ble_svc_battery_access, .val_handle = &battery_handle}, {0}}},
    {0}};

// --- OBSŁUGA BATERII ---
float read_battery_level()
{
    int raw = adc1_get_raw(BATTERY_PIN);
    float voltage = (raw / 4095.0f) * 3.3f * 2.0f * 0.906f;
    float percentage = ((voltage - 3.3f) / (4.2f - 3.3f)) * 100.0f;
    return (percentage > 100) ? 100 : (percentage < 0 ? 0 : percentage);
}

static int ble_svc_battery_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint8_t level = (uint8_t)read_battery_level();
    os_mbuf_append(ctxt->om, &level, sizeof(level));
    return 0;
}

static int hid_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int type = (int)arg;
    if (type == 1)
    { // HID Info
        uint8_t info[] = {0x11, 0x01, 0x00, 0x01};
        os_mbuf_append(ctxt->om, info, sizeof(info));
    }
    else if (type == 2)
    { // Report Map
        os_mbuf_append(ctxt->om, hid_report_map, sizeof(hid_report_map));
    }
    else if (type == 3)
    { // Report
        uint8_t report[] = {0x01, 0x00, 0x00};
        os_mbuf_append(ctxt->om, report, sizeof(report));
    }
    return 0;
}

// --- STEROWANIE ---
void send_media_command(uint16_t command_code)
{
    if (current_conn_handle == 0)
        return;
    uint8_t press[] = {0x01, command_code & 0xFF, (command_code >> 8) & 0xFF};
    uint8_t release[] = {0x01, 0x00, 0x00};
    ble_gattc_notify_custom(current_conn_handle, input_report_handle, ble_hs_mbuf_from_flat(press, 3));
    vTaskDelay(pdMS_TO_TICKS(50));
    ble_gattc_notify_custom(current_conn_handle, input_report_handle, ble_hs_mbuf_from_flat(release, 3));
}

static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0)
        {
            current_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "Polaczono!");
        }
        else
        {
            ble_app_advertise();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        current_conn_handle = 0;
        ESP_LOGI(TAG, "Rozlaczono");
        ble_app_advertise();
        break;
    }
    return 0;
}

void ble_app_advertise(void)
{
    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)TAG;
    fields.name_len = strlen(TAG);
    fields.name_is_complete = 1;
    fields.appearance = 0x03C1;
    fields.uuids16 = (ble_uuid16_t[]){BLE_UUID16_INIT(0x1812)};
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    ble_gap_adv_set_fields(&fields);
    struct ble_gap_adv_params adv_params = {.conn_mode = BLE_GAP_CONN_MODE_UND, .disc_mode = BLE_GAP_DISC_MODE_GEN};
    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}

void on_sync(void)
{
    ble_hs_id_infer_auto(0, &ble_addr_type);
    ble_app_advertise();
}

void IRAM_ATTR gpio_isr_handler(void *arg) {}

void hardware_task(void *param)
{
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    gpio_config_t io_conf = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pin_bit_mask = (1ULL << PLAY_PIN) | (1ULL << FORWARD_PIN) | (1ULL << PREVIOUS_PIN)};
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PLAY_PIN, gpio_isr_handler, NULL);
    gpio_isr_handler_add(FORWARD_PIN, gpio_isr_handler, NULL);
    gpio_isr_handler_add(PREVIOUS_PIN, gpio_isr_handler, NULL);

    TickType_t last_activity = xTaskGetTickCount();
    bool in_sleep_mode = false;

    while (1)
    {
        bool pressed = false;

        if (gpio_get_level(PLAY_PIN) == 0)
        {
            send_media_command(KEY_PLAY_PAUSE);
            while (gpio_get_level(PLAY_PIN) == 0)
                vTaskDelay(pdMS_TO_TICKS(10));
            pressed = true;
            in_sleep_mode = false;
        }
        else if (gpio_get_level(FORWARD_PIN) == 0)
        {
            send_media_command(KEY_NEXT_TRACK);
            while (gpio_get_level(FORWARD_PIN) == 0)
                vTaskDelay(pdMS_TO_TICKS(10));
            pressed = true;
            in_sleep_mode = false;
        }
        else if (gpio_get_level(PREVIOUS_PIN) == 0)
        {
            send_media_command(KEY_PREV_TRACK);
            while (gpio_get_level(PREVIOUS_PIN) == 0)
                vTaskDelay(pdMS_TO_TICKS(10));
            pressed = true;
            in_sleep_mode = false;
        }

        if (pressed)
            last_activity = xTaskGetTickCount();

        gpio_set_level(LED_PIN, (current_conn_handle == 0));

        // Wejście w tryb uśpienia po SLEEP_TIMEOUT_MS bez aktywności
        if (xTaskGetTickCount() - last_activity > pdMS_TO_TICKS(SLEEP_TIMEOUT_MS) && !in_sleep_mode)
        {
            gpio_set_level(LED_PIN, 0);
            in_sleep_mode = true;
            ESP_LOGI(TAG, "Wchodzę w tryb low-power (tickless idle)");
        }

        // Wyjście z trybu uśpienia gdy zmieni się stan przycisku
        if (in_sleep_mode && (gpio_get_level(PLAY_PIN) == 0 || gpio_get_level(FORWARD_PIN) == 0 || gpio_get_level(PREVIOUS_PIN) == 0))
        {
            in_sleep_mode = false;
            ESP_LOGI(TAG, "Wybudzam się z trybu low-power");
            last_activity = xTaskGetTickCount();
        }

        vTaskDelay(pdMS_TO_TICKS(in_sleep_mode ? 200 : 100));
    }
}

void host_task(void *param) { nimble_port_run(); }

void app_main()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    else
    {
        ESP_ERROR_CHECK(ret);
    }

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(BATTERY_PIN, ADC_ATTEN_DB_11);

    ESP_ERROR_CHECK(esp_nimble_hci_and_controller_init());

    nimble_port_init();

    ble_svc_gap_init();
    ble_svc_gatt_init();

    ble_svc_gap_device_name_set(TAG);

    ble_hs_cfg.sync_cb = on_sync;

    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    int rc;

    rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gatts_count_cfg failed: %d", rc);
        return;
    }

    rc = ble_gatts_add_svcs(gatt_svcs);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gatts_add_svcs failed: %d", rc);
        return;
    }

    nimble_port_freertos_init(host_task);

    xTaskCreate(hardware_task, "hw", 4096, NULL, 5, NULL);
}
