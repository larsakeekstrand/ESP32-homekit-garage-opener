//
// ota_update.c — periodically poll GitHub Releases and OTA-update if newer.
//
#include <string.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_http_client.h>
#include <esp_https_ota.h>
#include <esp_crt_bundle.h>
#include <esp_ota_ops.h>
#include <esp_system.h>

#include "ota_update.h"

static const char *TAG = "OTA";

#define OTA_BASE_URL      CONFIG_OTA_GITHUB_BASE_URL
#define VERSION_URL       OTA_BASE_URL "/releases/latest/download/version.txt"
#define FIRMWARE_URL      OTA_BASE_URL "/releases/latest/download/garage.bin"
#define POLL_INTERVAL_MS      (CONFIG_OTA_POLL_INTERVAL_SEC * 1000)
#define STABLE_BOOT_DELAY_MS  30000  /* survive this long → confirm image (network-independent) */

/* Parse up to major.minor.patch from a version string. A non-numeric/missing
 * component (e.g. the "-dev" suffix) parses as 0. */
static void parse_ver(const char *s, int out[3]) {
    out[0] = out[1] = out[2] = 0;
    sscanf(s, "%d.%d.%d", &out[0], &out[1], &out[2]);
}

/* Return >0 if a is newer than b, <0 if older, 0 if equal. */
static int ota_semver_cmp(const char *a, const char *b) {
    int va[3], vb[3];
    parse_ver(a, va);
    parse_ver(b, vb);
    for (int i = 0; i < 3; i++) {
        if (va[i] != vb[i]) return (va[i] > vb[i]) ? 1 : -1;
    }
    return 0;
}

typedef struct {
    char  *buf;
    int    len;
    int    cap;
} ota_resp_t;

static esp_err_t ota_http_event(esp_http_client_event_t *evt) {
    ota_resp_t *r = (ota_resp_t *) evt->user_data;
    switch (evt->event_id) {
        case HTTP_EVENT_REDIRECT:
            /* discard anything captured before the redirect */
            if (r) { r->len = 0; if (r->cap > 0) r->buf[0] = '\0'; }
            break;
        case HTTP_EVENT_ON_DATA:
            if (r && evt->data_len > 0) {
                int space = r->cap - 1 - r->len;
                int n = (evt->data_len < space) ? evt->data_len : space;
                if (n > 0) {
                    memcpy(r->buf + r->len, evt->data, n);
                    r->len += n;
                    r->buf[r->len] = '\0';
                }
            }
            break;
        default:
            break;
    }
    return ESP_OK;
}

/* Fetch the latest release's version.txt into buf (NUL-terminated, trimmed),
 * following redirects and requiring HTTP 200. Returns ESP_OK on success. */
static esp_err_t fetch_latest_version(char *buf, size_t buf_len) {
    if (buf_len == 0) return ESP_FAIL;
    buf[0] = '\0';
    ota_resp_t resp = { .buf = buf, .len = 0, .cap = (int) buf_len };
    esp_http_client_config_t config = {
        .url = VERSION_URL,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 15000,
        .event_handler = ota_http_event,
        .user_data = &resp,
        /* GitHub redirects to a long signed CDN URL; the default 512 B HTTP
         * buffers can't hold that request line / response headers. */
        .buffer_size = 4096,
        .buffer_size_tx = 4096,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) return ESP_FAIL;

    esp_err_t err = esp_http_client_perform(client);   /* follows 3xx redirects */
    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "version fetch failed: %s", esp_err_to_name(err));
        return err;
    }
    if (status != 200) {
        ESP_LOGE(TAG, "version fetch HTTP status %d", status);
        return ESP_FAIL;
    }
    int n = resp.len;
    while (n > 0 && (buf[n-1] == '\n' || buf[n-1] == '\r' || buf[n-1] == ' ' || buf[n-1] == '\t')) {
        buf[--n] = '\0';
    }
    return (n > 0) ? ESP_OK : ESP_FAIL;
}

/* Download and apply the latest firmware; reboots on success. */
static void do_ota(void) {
    ESP_LOGI(TAG, "Starting OTA from %s", FIRMWARE_URL);
    esp_http_client_config_t http_config = {
        .url = FIRMWARE_URL,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 30000,
        .keep_alive_enable = true,
        /* GitHub redirects to a long signed CDN URL; enlarge the HTTP buffers
         * so the redirected request line / headers fit (default is 512 B). */
        .buffer_size = 4096,
        .buffer_size_tx = 4096,
    };
    esp_https_ota_config_t ota_config = {
        .http_config = &http_config,
    };
    esp_err_t err = esp_https_ota(&ota_config);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "OTA successful; rebooting into new firmware");
        esp_restart();
    } else {
        ESP_LOGE(TAG, "OTA failed: %s", esp_err_to_name(err));
    }
}

static void ota_task(void *arg) {
    /* Confirm a healthy boot independent of Wi-Fi/provisioning, so rollback
     * only catches crash/boot-loops — not transient network states. */
    vTaskDelay(STABLE_BOOT_DELAY_MS / portTICK_PERIOD_MS);
    ota_update_mark_valid();

    while (1) {
        char remote[32];
        if (fetch_latest_version(remote, sizeof(remote)) == ESP_OK) {
            if (ota_semver_cmp(remote, FW_VERSION) > 0) {
                ESP_LOGI(TAG, "Update available: running %s, latest %s", FW_VERSION, remote);
                do_ota();
            } else {
                ESP_LOGI(TAG, "Up to date (running %s, latest %s)", FW_VERSION, remote);
            }
        } else {
            ESP_LOGW(TAG, "Version check failed; will retry");
        }
        vTaskDelay(POLL_INTERVAL_MS / portTICK_PERIOD_MS);
    }
}

void ota_update_start(void) {
    xTaskCreate(ota_task, "ota_update", 12 * 1024, NULL, 2, NULL);
}

void ota_update_mark_valid(void) {
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(running, &state) == ESP_OK &&
        state == ESP_OTA_IMG_PENDING_VERIFY) {
        ESP_LOGI(TAG, "Marking running image valid (cancelling rollback)");
        esp_ota_mark_app_valid_cancel_rollback();
    }
}
