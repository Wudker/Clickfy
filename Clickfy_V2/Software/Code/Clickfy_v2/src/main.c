#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_adc_cal.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_nimble_hci.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_hs_mbuf.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#define TAG "Clickfy_2.0"
#define DEVICE_NAME "Clickfy 2.0"
#define MANUFACTURER_NAME "Blough"

#define PLAY_PIN GPIO_NUM_7
#define FORWARD_PIN GPIO_NUM_3
#define PREVIOUS_PIN GPIO_NUM_18
#define LED_PIN GPIO_NUM_6
#define BATTERY_CHANNEL ADC1_CHANNEL_4

#define BUTTON_MASK ((1ULL << PLAY_PIN) | (1ULL << FORWARD_PIN) | (1ULL << PREVIOUS_PIN))

/* Fast advertising is used after boot and disconnect. If the phone does not
 * connect in this time, Bluetooth is stopped before entering light sleep. */
#define FAST_ADV_DURATION_MS 30000
#define CONNECTED_IDLE_TIMEOUT_MS 15000
#define BUTTON_DEBOUNCE_MS 25
#define REPORT_RELEASE_DELAY_MS 25
#define BATTERY_CHECK_INTERVAL_MS (5 * 60 * 1000)

#define BATTERY_DIVIDER_RATIO 2.0f
#define BATTERY_CALIBRATION 1.0f
#define BATTERY_DEFAULT_VREF_MV 1100

#define REPORT_ID_CONSUMER_CONTROL 1
#define REPORT_TYPE_INPUT 1

#define KEY_PLAY_PAUSE 0x00CD
#define KEY_NEXT_TRACK 0x00B5
#define KEY_PREV_TRACK 0x00B6

#define RTC_PENDING_MAGIC_VALUE 0x434C4B59UL
#define COMMAND_QUEUE_LENGTH 4

typedef enum {
    HID_ACCESS_INFO = 1,
    HID_ACCESS_REPORT_MAP,
    HID_ACCESS_INPUT_REPORT,
    HID_ACCESS_PROTOCOL_MODE,
    HID_ACCESS_CONTROL_POINT,
} hid_access_type_t;

static uint8_t ble_addr_type;
static volatile uint16_t current_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static volatile bool input_notify_enabled;
static volatile bool battery_notify_enabled;
static volatile bool ble_stack_synced;
static volatile bool ble_stack_active;
static volatile bool sleep_requested;
static volatile bool sleep_after_disconnect;
static volatile bool suppress_wake_button_until_release;

static uint16_t input_report_handle;
static uint16_t battery_handle;
static uint8_t protocol_mode = 1;
static uint8_t control_point;
static uint8_t input_report_value[2];
static volatile uint8_t cached_battery_level = 100;

static QueueHandle_t command_queue;
static SemaphoreHandle_t ble_host_stopped_sem;
static TaskHandle_t clickfy_task_handle;
static esp_adc_cal_characteristics_t adc_characteristics;
static int64_t boot_started_us;
static volatile TickType_t last_activity_tick;

/* RTC_NOINIT survives esp_restart(). A magic value prevents random power-on
 * contents from being interpreted as a media command. */
RTC_NOINIT_ATTR static uint32_t rtc_pending_magic;
RTC_NOINIT_ATTR static uint16_t rtc_pending_command;

void ble_store_config_init(void);

static void ble_app_advertise(void);
static int ble_gap_event(struct ble_gap_event *event, void *arg);
static void notify_clickfy_task(void);

static const uint8_t hid_report_map[] = {
    0x05, 0x0C,       /* Usage Page (Consumer) */
    0x09, 0x01,       /* Usage (Consumer Control) */
    0xA1, 0x01,       /* Collection (Application) */
    0x85, REPORT_ID_CONSUMER_CONTROL,
    0x19, 0x00,       /* Usage Minimum (Unassigned) */
    0x2A, 0x3C, 0x02, /* Usage Maximum (AC Format) */
    0x15, 0x00,       /* Logical Minimum (0) */
    0x26, 0x3C, 0x02, /* Logical Maximum (572) */
    0x95, 0x01,       /* Report Count (1) */
    0x75, 0x10,       /* Report Size (16) */
    0x81, 0x00,       /* Input (Data, Array, Absolute) */
    0xC0              /* End Collection */
};

static int append_to_mbuf(struct os_mbuf *om, const void *data, uint16_t length)
{
    return os_mbuf_append(om, data, length) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int report_reference_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                      struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_DSC) {
        return BLE_ATT_ERR_READ_NOT_PERMITTED;
    }

    const uint8_t report_reference[] = {
        REPORT_ID_CONSUMER_CONTROL,
        REPORT_TYPE_INPUT,
    };
    return append_to_mbuf(ctxt->om, report_reference, sizeof(report_reference));
}

static int hid_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;

    const hid_access_type_t type = (hid_access_type_t)(uintptr_t)arg;
    uint8_t value;
    uint16_t copied;
    int rc;

    switch (type) {
    case HID_ACCESS_INFO: {
        const uint8_t hid_info[] = {0x11, 0x01, 0x00, 0x01};
        return append_to_mbuf(ctxt->om, hid_info, sizeof(hid_info));
    }

    case HID_ACCESS_REPORT_MAP:
        return append_to_mbuf(ctxt->om, hid_report_map, sizeof(hid_report_map));

    case HID_ACCESS_INPUT_REPORT:
        return append_to_mbuf(ctxt->om, input_report_value, sizeof(input_report_value));

    case HID_ACCESS_PROTOCOL_MODE:
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            return append_to_mbuf(ctxt->om, &protocol_mode, sizeof(protocol_mode));
        }
        if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        rc = ble_hs_mbuf_to_flat(ctxt->om, &value, sizeof(value), &copied);
        if (rc != 0 || copied != sizeof(value) || value > 1) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        protocol_mode = value;
        return 0;

    case HID_ACCESS_CONTROL_POINT:
        if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
            return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
        }
        rc = ble_hs_mbuf_to_flat(ctxt->om, &value, sizeof(value), &copied);
        if (rc != 0 || copied != sizeof(value) || value > 1) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        control_point = value;
        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

static int battery_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        return BLE_ATT_ERR_READ_NOT_PERMITTED;
    }

    const uint8_t level = cached_battery_level;
    return append_to_mbuf(ctxt->om, &level, sizeof(level));
}

static int manufacturer_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                  struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        return BLE_ATT_ERR_READ_NOT_PERMITTED;
    }
    return append_to_mbuf(ctxt->om, MANUFACTURER_NAME, strlen(MANUFACTURER_NAME));
}

static struct ble_gatt_dsc_def input_report_descriptors[] = {
    {
        .uuid = BLE_UUID16_DECLARE(0x2908),
        .att_flags = BLE_ATT_F_READ | BLE_ATT_F_READ_ENC,
        .access_cb = report_reference_access_cb,
    },
    {0},
};

static const struct ble_gatt_chr_def hid_characteristics[] = {
    {
        .uuid = BLE_UUID16_DECLARE(0x2A4A),
        .access_cb = hid_access_cb,
        .arg = (void *)(uintptr_t)HID_ACCESS_INFO,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC,
    },
    {
        .uuid = BLE_UUID16_DECLARE(0x2A4B),
        .access_cb = hid_access_cb,
        .arg = (void *)(uintptr_t)HID_ACCESS_REPORT_MAP,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC,
    },
    {
        .uuid = BLE_UUID16_DECLARE(0x2A4D),
        .access_cb = hid_access_cb,
        .arg = (void *)(uintptr_t)HID_ACCESS_INPUT_REPORT,
        .descriptors = input_report_descriptors,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &input_report_handle,
    },
    {
        .uuid = BLE_UUID16_DECLARE(0x2A4E),
        .access_cb = hid_access_cb,
        .arg = (void *)(uintptr_t)HID_ACCESS_PROTOCOL_MODE,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC |
                 BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_WRITE_ENC,
    },
    {
        .uuid = BLE_UUID16_DECLARE(0x2A4C),
        .access_cb = hid_access_cb,
        .arg = (void *)(uintptr_t)HID_ACCESS_CONTROL_POINT,
        .flags = BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_WRITE_ENC,
    },
    {0},
};

static const struct ble_gatt_chr_def battery_characteristics[] = {
    {
        .uuid = BLE_UUID16_DECLARE(0x2A19),
        .access_cb = battery_access_cb,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &battery_handle,
    },
    {0},
};

static const struct ble_gatt_chr_def device_information_characteristics[] = {
    {
        .uuid = BLE_UUID16_DECLARE(0x2A29),
        .access_cb = manufacturer_access_cb,
        .flags = BLE_GATT_CHR_F_READ,
    },
    {0},
};

static const struct ble_gatt_svc_def gatt_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x1812),
        .characteristics = hid_characteristics,
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180F),
        .characteristics = battery_characteristics,
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180A),
        .characteristics = device_information_characteristics,
    },
    {0},
};

static bool command_is_valid(uint16_t command)
{
    return command == KEY_PLAY_PAUSE ||
           command == KEY_NEXT_TRACK ||
           command == KEY_PREV_TRACK;
}

static uint16_t command_from_button_state(uint64_t wake_mask)
{
    if ((wake_mask & (1ULL << PLAY_PIN)) || gpio_get_level(PLAY_PIN) == 0) {
        return KEY_PLAY_PAUSE;
    }
    if ((wake_mask & (1ULL << FORWARD_PIN)) || gpio_get_level(FORWARD_PIN) == 0) {
        return KEY_NEXT_TRACK;
    }
    if ((wake_mask & (1ULL << PREVIOUS_PIN)) || gpio_get_level(PREVIOUS_PIN) == 0) {
        return KEY_PREV_TRACK;
    }
    return 0;
}

static bool any_button_pressed(void)
{
    return gpio_get_level(PLAY_PIN) == 0 ||
           gpio_get_level(FORWARD_PIN) == 0 ||
           gpio_get_level(PREVIOUS_PIN) == 0;
}

static void queue_media_command(uint16_t command)
{
    if (!command_is_valid(command)) {
        return;
    }

    if (xQueueSend(command_queue, &command, 0) != pdTRUE) {
        uint16_t discarded;
        xQueueReceive(command_queue, &discarded, 0);
        xQueueSend(command_queue, &command, 0);
        ESP_LOGW(TAG, "Kolejka polecen byla pelna; usunieto najstarsze polecenie");
    }
}

static bool send_media_command(uint16_t command)
{
    const uint16_t conn_handle = current_conn_handle;
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE || !input_notify_enabled) {
        return false;
    }

    const uint8_t press[] = {
        (uint8_t)(command & 0xFF),
        (uint8_t)(command >> 8),
    };
    const uint8_t release[] = {0x00, 0x00};

    struct os_mbuf *om = ble_hs_mbuf_from_flat(press, sizeof(press));
    if (om == NULL) {
        ESP_LOGE(TAG, "Brak bufora dla raportu HID");
        return false;
    }

    int rc = ble_gattc_notify_custom(conn_handle, input_report_handle, om);
    if (rc != 0) {
        ESP_LOGW(TAG, "Nie wyslano raportu HID; rc=%d", rc);
        return false;
    }

    memcpy(input_report_value, press, sizeof(input_report_value));
    vTaskDelay(pdMS_TO_TICKS(REPORT_RELEASE_DELAY_MS));

    om = ble_hs_mbuf_from_flat(release, sizeof(release));
    if (om != NULL) {
        rc = ble_gattc_notify_custom(conn_handle, input_report_handle, om);
        if (rc != 0) {
            ESP_LOGW(TAG, "Nie wyslano zwolnienia klawisza; rc=%d", rc);
        }
    }
    memcpy(input_report_value, release, sizeof(input_report_value));

    ESP_LOGI(TAG, "Wyslano polecenie HID 0x%04X", command);
    return true;
}

static void send_queued_commands(void)
{
    uint16_t command;

    while (current_conn_handle != BLE_HS_CONN_HANDLE_NONE &&
           input_notify_enabled &&
           xQueuePeek(command_queue, &command, 0) == pdTRUE) {
        if (!send_media_command(command)) {
            break;
        }
        xQueueReceive(command_queue, &command, 0);
    }
}

static uint8_t battery_percent_from_mv(uint32_t millivolts)
{
    static const struct {
        uint16_t millivolts;
        uint8_t percent;
    } curve[] = {
        {4200, 100},
        {4100, 90},
        {4000, 80},
        {3900, 65},
        {3800, 45},
        {3700, 25},
        {3600, 10},
        {3400, 3},
        {3300, 0},
    };

    if (millivolts >= curve[0].millivolts) {
        return 100;
    }
    if (millivolts <= curve[sizeof(curve) / sizeof(curve[0]) - 1].millivolts) {
        return 0;
    }

    for (size_t i = 1; i < sizeof(curve) / sizeof(curve[0]); ++i) {
        if (millivolts >= curve[i].millivolts) {
            const uint32_t voltage_span = curve[i - 1].millivolts - curve[i].millivolts;
            const uint32_t percent_span = curve[i - 1].percent - curve[i].percent;
            return curve[i].percent +
                   (uint8_t)(((millivolts - curve[i].millivolts) * percent_span) /
                             voltage_span);
        }
    }

    return 0;
}

static uint8_t read_battery_level(void)
{
    uint32_t raw_sum = 0;
    const uint32_t samples = 32;

    /* The 200k/200k divider has high output impedance. Several conversions
     * allow the ADC sample capacitor to settle and reduce random noise. */
    (void)adc1_get_raw(BATTERY_CHANNEL);
    for (uint32_t i = 0; i < samples; ++i) {
        raw_sum += adc1_get_raw(BATTERY_CHANNEL);
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    const uint32_t raw_average = raw_sum / samples;
    const uint32_t adc_mv = esp_adc_cal_raw_to_voltage(raw_average, &adc_characteristics);
    const uint32_t battery_mv =
        (uint32_t)(adc_mv * BATTERY_DIVIDER_RATIO * BATTERY_CALIBRATION);
    const uint8_t percentage = battery_percent_from_mv(battery_mv);

    ESP_LOGI(TAG, "Bateria: %lu mV, %u%%",
             (unsigned long)battery_mv, percentage);
    return percentage;
}

static void update_battery_level(void)
{
    const uint8_t previous = cached_battery_level;
    cached_battery_level = read_battery_level();

    if (battery_notify_enabled &&
        current_conn_handle != BLE_HS_CONN_HANDLE_NONE &&
        cached_battery_level != previous) {
        const uint8_t level = cached_battery_level;
        struct os_mbuf *om = ble_hs_mbuf_from_flat(&level, sizeof(level));
        if (om != NULL) {
            const int rc =
                ble_gattc_notify_custom(current_conn_handle, battery_handle, om);
            if (rc != 0) {
                ESP_LOGW(TAG, "Nie wyslano poziomu baterii; rc=%d", rc);
            }
        }
    }
}

static void pulse_low_battery_led(TickType_t *last_blink)
{
    const uint8_t level = cached_battery_level;
    if (level > 20) {
        gpio_set_level(LED_PIN, 0);
        return;
    }

    const TickType_t now = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(level <= 5 ? 2000 : 10000);
    if (now - *last_blink < interval) {
        return;
    }

    *last_blink = now;
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(level <= 5 ? 50 : 20));
    gpio_set_level(LED_PIN, 0);
}

static void notify_clickfy_task(void)
{
    if (clickfy_task_handle != NULL) {
        xTaskNotifyGive(clickfy_task_handle);
    }
}

static void mark_activity(void)
{
    last_activity_tick = xTaskGetTickCount();
    sleep_requested = false;
    sleep_after_disconnect = false;
}

static void IRAM_ATTR button_isr_handler(void *arg)
{
    (void)arg;
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (clickfy_task_handle != NULL) {
        vTaskNotifyGiveFromISR(clickfy_task_handle, &higher_priority_task_woken);
    }
    if (higher_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void configure_hardware(void)
{
    gpio_config_t output_config = {
        .pin_bit_mask = 1ULL << LED_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&output_config));
    gpio_set_level(LED_PIN, 0);

    gpio_config_t button_config = {
        .pin_bit_mask = BUTTON_MASK,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&button_config));

    esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }
    ESP_ERROR_CHECK(gpio_isr_handler_add(PLAY_PIN, button_isr_handler, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(FORWARD_PIN, button_isr_handler, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PREVIOUS_PIN, button_isr_handler, NULL));

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(BATTERY_CHANNEL, ADC_ATTEN_DB_11);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12,
                             BATTERY_DEFAULT_VREF_MV, &adc_characteristics);
}

static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    struct ble_gap_conn_desc desc;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            current_conn_handle = event->connect.conn_handle;
            input_notify_enabled = false;
            battery_notify_enabled = false;
            mark_activity();

            const int64_t connected_after_ms =
                (esp_timer_get_time() - boot_started_us) / 1000;
            ESP_LOGI(TAG, "Polaczono po %lld ms od startu",
                     (long long)connected_after_ms);

            rc = ble_gap_security_initiate(current_conn_handle);
            if (rc != 0 && rc != BLE_HS_EALREADY) {
                ESP_LOGW(TAG, "Nie rozpoczeto zabezpieczania polaczenia; rc=%d", rc);
            }

            /* Up to 250 ms effective idle interval keeps media keys responsive
             * while allowing the peripheral to skip four connection events. */
            const struct ble_gap_upd_params low_power_params = {
                .itvl_min = 24, /* 30 ms */
                .itvl_max = 40, /* 50 ms */
                .latency = 4,
                .supervision_timeout = 500, /* 5 s */
                .min_ce_len = 0,
                .max_ce_len = 0,
            };
            rc = ble_gap_update_params(current_conn_handle, &low_power_params);
            if (rc != 0) {
                ESP_LOGW(TAG, "Telefon odrzucil parametry low-power; rc=%d", rc);
            }
            notify_clickfy_task();
        } else {
            ESP_LOGW(TAG, "Proba polaczenia nieudana; status=%d",
                     event->connect.status);
            ble_app_advertise();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT: {
        const bool should_sleep = sleep_after_disconnect;
        sleep_after_disconnect = false;
        current_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        input_notify_enabled = false;
        battery_notify_enabled = false;
        ESP_LOGI(TAG, "Rozlaczono; powod=%d", event->disconnect.reason);
        if (should_sleep) {
            sleep_requested = true;
        } else {
            ble_app_advertise();
        }
        notify_clickfy_task();
        return 0;
    }

    case BLE_GAP_EVENT_ADV_COMPLETE:
        if (event->adv_complete.reason == BLE_HS_ETIMEOUT) {
            ESP_LOGI(TAG, "Telefon nie polaczyl sie w oknie szybkiego reklamowania");
            sleep_requested = true;
            notify_clickfy_task();
        }
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        if (event->enc_change.status == 0) {
            mark_activity();
            ESP_LOGI(TAG, "Polaczenie zaszyfrowane / bonding przywrocony");
        } else {
            ESP_LOGW(TAG, "Zmiana szyfrowania nieudana; status=%d",
                     event->enc_change.status);
        }
        notify_clickfy_task();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == input_report_handle) {
            input_notify_enabled = event->subscribe.cur_notify;
            if (input_notify_enabled) {
                mark_activity();
            }
            ESP_LOGI(TAG, "Raport HID notifications: %s",
                     input_notify_enabled ? "wlaczone" : "wylaczone");
        } else if (event->subscribe.attr_handle == battery_handle) {
            battery_notify_enabled = event->subscribe.cur_notify;
        }
        notify_clickfy_task();
        return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING:
        rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        if (rc == 0) {
            ble_store_util_delete_peer(&desc.peer_id_addr);
        }
        return BLE_GAP_REPEAT_PAIRING_RETRY;

    default:
        return 0;
    }
}

static void ble_app_advertise(void)
{
    if (!ble_stack_synced ||
        current_conn_handle != BLE_HS_CONN_HANDLE_NONE ||
        ble_gap_adv_active()) {
        return;
    }

    const ble_uuid16_t service_uuids[] = {
        BLE_UUID16_INIT(0x1812),
        BLE_UUID16_INIT(0x180F),
    };
    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)DEVICE_NAME;
    fields.name_len = strlen(DEVICE_NAME);
    fields.name_is_complete = 1;
    fields.appearance = 0x03C1;
    fields.uuids16 = (ble_uuid16_t *)service_uuids;
    fields.num_uuids16 = sizeof(service_uuids) / sizeof(service_uuids[0]);
    fields.uuids16_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Nie ustawiono danych reklamowych; rc=%d", rc);
        return;
    }

    struct ble_gap_adv_params params = {0};
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    params.itvl_min = BLE_GAP_ADV_FAST_INTERVAL1_MIN;
    params.itvl_max = BLE_GAP_ADV_FAST_INTERVAL1_MAX;

    sleep_requested = false;
    rc = ble_gap_adv_start(ble_addr_type, NULL, FAST_ADV_DURATION_MS,
                           &params, ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Nie uruchomiono reklamowania; rc=%d", rc);
        return;
    }

    ESP_LOGI(TAG, "Szybkie reklamowanie BLE przez %d s",
             FAST_ADV_DURATION_MS / 1000);
}

static void on_ble_reset(int reason)
{
    ESP_LOGE(TAG, "Reset stosu NimBLE; powod=%d", reason);
    current_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    input_notify_enabled = false;
    battery_notify_enabled = false;
    ble_stack_synced = false;
}

static void on_ble_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "Brak poprawnego adresu BLE; rc=%d", rc);
        return;
    }

    rc = ble_hs_id_infer_auto(0, &ble_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Nie ustalono typu adresu BLE; rc=%d", rc);
        return;
    }

    ble_stack_synced = true;
    ble_app_advertise();
}

static void ble_host_task(void *param)
{
    (void)param;
    ESP_LOGI(TAG, "Uruchomiono zadanie NimBLE");
    nimble_port_run();

    xSemaphoreGive(ble_host_stopped_sem);
    nimble_port_freertos_deinit();
}

static bool start_ble_stack(void)
{
    esp_err_t err = esp_nimble_hci_and_controller_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Nie uruchomiono kontrolera BLE: %s", esp_err_to_name(err));
        return false;
    }

    nimble_port_init();
    ble_stack_active = true;
    ble_stack_synced = false;
    current_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    input_notify_enabled = false;
    battery_notify_enabled = false;

    ble_hs_cfg.reset_cb = on_ble_reset;
    ble_hs_cfg.sync_cb = on_ble_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_svc_gap_device_name_set(DEVICE_NAME);
    if (rc != 0) {
        ESP_LOGE(TAG, "Nie ustawiono nazwy BLE; rc=%d", rc);
        return false;
    }

    rc = ble_gatts_count_cfg(gatt_services);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg nie powiodlo sie; rc=%d", rc);
        return false;
    }

    rc = ble_gatts_add_svcs(gatt_services);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs nie powiodlo sie; rc=%d", rc);
        return false;
    }

    ble_store_config_init();
    nimble_port_freertos_init(ble_host_task);
    return true;
}

static bool stop_ble_stack(void)
{
    if (!ble_stack_active) {
        return true;
    }

    ble_stack_synced = false;
    if (ble_gap_adv_active()) {
        ble_gap_adv_stop();
    }

    const int rc = nimble_port_stop();
    if (rc != 0) {
        ESP_LOGE(TAG, "Nie zatrzymano hosta NimBLE; rc=%d", rc);
        return false;
    }

    if (xSemaphoreTake(ble_host_stopped_sem, pdMS_TO_TICKS(2000)) != pdTRUE) {
        ESP_LOGE(TAG, "Przekroczono czas zatrzymywania zadania NimBLE");
        return false;
    }

    nimble_port_deinit();
    const esp_err_t err = esp_nimble_hci_and_controller_deinit();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Nie zatrzymano kontrolera BLE: %s", esp_err_to_name(err));
        return false;
    }

    ble_stack_active = false;
    current_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    input_notify_enabled = false;
    battery_notify_enabled = false;
    return true;
}

static void restart_fast_advertising(void)
{
    if (!ble_stack_synced || current_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        return;
    }

    sleep_requested = false;
    if (ble_gap_adv_active()) {
        ble_gap_adv_stop();
    }
    ble_app_advertise();
}

static void enter_disconnected_light_sleep(void)
{
    if (current_conn_handle != BLE_HS_CONN_HANDLE_NONE || any_button_pressed()) {
        sleep_requested = false;
        restart_fast_advertising();
        return;
    }

    ESP_LOGI(TAG, "Zatrzymywanie BLE przed light-sleep");
    gpio_set_level(LED_PIN, 0);
    if (!stop_ble_stack()) {
        ESP_LOGE(TAG, "Bezpieczne uspienie nieudane; restart");
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_restart();
    }

    xQueueReset(command_queue);
    gpio_intr_disable(PLAY_PIN);
    gpio_intr_disable(FORWARD_PIN);
    gpio_intr_disable(PREVIOUS_PIN);

    ESP_ERROR_CHECK(gpio_wakeup_enable(PLAY_PIN, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(gpio_wakeup_enable(FORWARD_PIN, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(gpio_wakeup_enable(PREVIOUS_PIN, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());

    ESP_LOGI(TAG, "Light-sleep: oczekiwanie na dowolny przycisk");
    vTaskDelay(pdMS_TO_TICKS(30));
    const esp_err_t sleep_result = esp_light_sleep_start();
    if (sleep_result != ESP_OK) {
        ESP_LOGE(TAG, "Light-sleep nie uruchomil sie: %s",
                 esp_err_to_name(sleep_result));
    }

    const uint64_t wake_mask = esp_sleep_get_gpio_wakeup_status();
    const uint16_t command = command_from_button_state(wake_mask);
    if (command_is_valid(command)) {
        rtc_pending_command = command;
        rtc_pending_magic = RTC_PENDING_MAGIC_VALUE;
    } else {
        rtc_pending_command = 0;
        rtc_pending_magic = 0;
    }

    /* A clean reboot is faster and more robust than rebuilding a previously
     * deinitialized HID host in-place. RTC_NOINIT carries the wake command. */
    esp_restart();
}

static void handle_button_press(void)
{
    if (suppress_wake_button_until_release) {
        if (any_button_pressed()) {
            return;
        }
        suppress_wake_button_until_release = false;
    }

    if (!any_button_pressed()) {
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
    const uint16_t command = command_from_button_state(0);
    if (!command_is_valid(command)) {
        return;
    }

    mark_activity();
    queue_media_command(command);
    if (current_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        restart_fast_advertising();
    }
    send_queued_commands();

    while (any_button_pressed()) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
    (void)ulTaskNotifyTake(pdTRUE, 0);
}

static TickType_t next_task_wait_time(void)
{
    if (suppress_wake_button_until_release) {
        return pdMS_TO_TICKS(25);
    }

    if (current_conn_handle != BLE_HS_CONN_HANDLE_NONE &&
        !sleep_after_disconnect) {
        const TickType_t timeout = pdMS_TO_TICKS(CONNECTED_IDLE_TIMEOUT_MS);
        const TickType_t elapsed = xTaskGetTickCount() - last_activity_tick;
        if (elapsed >= timeout) {
            return 1;
        }

        const TickType_t remaining = timeout - elapsed;
        const TickType_t regular_wakeup = pdMS_TO_TICKS(5000);
        return remaining < regular_wakeup ? remaining : regular_wakeup;
    }

    return pdMS_TO_TICKS(5000);
}

static void request_connected_idle_sleep(void)
{
    const uint16_t conn_handle = current_conn_handle;
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE ||
        sleep_after_disconnect ||
        any_button_pressed()) {
        return;
    }

    const TickType_t elapsed = xTaskGetTickCount() - last_activity_tick;
    if (elapsed < pdMS_TO_TICKS(CONNECTED_IDLE_TIMEOUT_MS)) {
        return;
    }

    sleep_after_disconnect = true;
    ESP_LOGI(TAG, "Brak aktywnosci przez %d s; rozlaczanie przed light-sleep",
             CONNECTED_IDLE_TIMEOUT_MS / 1000);
    const int rc = ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    if (rc != 0) {
        sleep_after_disconnect = false;
        ESP_LOGW(TAG, "Nie rozpoczeto rozlaczania przed snem; rc=%d", rc);
    }
}

static void clickfy_task(void *param)
{
    (void)param;
    TickType_t last_battery_check = xTaskGetTickCount();
    TickType_t last_led_blink = xTaskGetTickCount();

    while (true) {
        ulTaskNotifyTake(pdTRUE, next_task_wait_time());

        handle_button_press();
        send_queued_commands();
        request_connected_idle_sleep();

        const TickType_t now = xTaskGetTickCount();
        if (now - last_battery_check >= pdMS_TO_TICKS(BATTERY_CHECK_INTERVAL_MS)) {
            last_battery_check = now;
            update_battery_level();
        }
        pulse_low_battery_led(&last_led_blink);

        if (sleep_requested &&
            current_conn_handle == BLE_HS_CONN_HANDLE_NONE &&
            !any_button_pressed()) {
            enter_disconnected_light_sleep();
        }
    }
}

static void initialize_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

static void configure_power_management(void)
{
#if CONFIG_PM_ENABLE
    /* ESP-IDF 4.4 keeps a no-light-sleep lock while Bluetooth is enabled.
     * DFS still saves active power; manual light sleep is entered only after
     * the controller has been fully stopped. */
    const esp_pm_config_esp32c3_t pm_config = {
        .max_freq_mhz = 80,
        .min_freq_mhz = 40,
        .light_sleep_enable = false,
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
#endif
}

void app_main(void)
{
    boot_started_us = esp_timer_get_time();
    last_activity_tick = xTaskGetTickCount();
    initialize_nvs();
    configure_power_management();
    configure_hardware();

    command_queue = xQueueCreate(COMMAND_QUEUE_LENGTH, sizeof(uint16_t));
    ble_host_stopped_sem = xSemaphoreCreateBinary();
    if (command_queue == NULL || ble_host_stopped_sem == NULL) {
        ESP_LOGE(TAG, "Nie utworzono obiektow FreeRTOS");
        return;
    }

    cached_battery_level = read_battery_level();

    if (esp_reset_reason() == ESP_RST_SW &&
        rtc_pending_magic == RTC_PENDING_MAGIC_VALUE &&
        command_is_valid(rtc_pending_command)) {
        queue_media_command(rtc_pending_command);
        suppress_wake_button_until_release = true;
        ESP_LOGI(TAG, "Zapamietano polecenie z przycisku wybudzajacego");
    }
    rtc_pending_magic = 0;
    rtc_pending_command = 0;

    BaseType_t task_result =
        xTaskCreate(clickfy_task, "clickfy", 4096, NULL, 5, &clickfy_task_handle);
    if (task_result != pdPASS) {
        ESP_LOGE(TAG, "Nie utworzono zadania Clickfy");
        return;
    }

    if (!start_ble_stack()) {
        ESP_LOGE(TAG, "Uruchomienie BLE nie powiodlo sie");
    }
}