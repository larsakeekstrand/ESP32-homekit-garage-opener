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
#define POLL_INTERVAL_MS  (CONFIG_OTA_POLL_INTERVAL_SEC * 1000)
#define FIRST_CHECK_DELAY_MS 60000   /* let Wi-Fi/HAP settle and mark-valid run first */

/* Parse up to major.minor.patch from a version string. A non-numeric/missing
 * component (e.g. the "-dev" suffix) parses as 0. */
static void parse_ver(const char *s, int out[3]) {
    out[0] = out[1] = out[2] = 0;
    sscanf(s, "%d.%d.%d", &out[0], &out[1], &out[2]);
}

/* Return >0 if a is newer than b, <0 if older, 0 if equal. */
int ota_semver_cmp(const char *a, const char *b) {
    int va[3], vb[3];
    parse_ver(a, va);
    parse_ver(b, vb);
    for (int i = 0; i < 3; i++) {
        if (va[i] != vb[i]) return (va[i] > vb[i]) ? 1 : -1;
    }
    return 0;
}

/* Fetch the latest release's version.txt into buf (NUL-terminated, trimmed).
 * Returns ESP_OK on success. */
static esp_err_t fetch_latest_version(char *buf, size_t buf_len) {
    esp_http_client_config_t config = {
        .url = VERSION_URL,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 15000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) return ESP_FAIL;

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open %s: %s", VERSION_URL, esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return err;
    }
    esp_http_client_fetch_headers(client);
    int read = esp_http_client_read_response(client, buf, buf_len - 1);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    if (read <= 0) return ESP_FAIL;

    buf[read] = '\0';
    /* trim trailing whitespace/newline */
    while (read > 0 && (buf[read - 1] == '\n' || buf[read - 1] == '\r' || buf[read - 1] == ' ')) {
        buf[--read] = '\0';
    }
    return ESP_OK;
}

/* Download and apply the latest firmware; reboots on success. */
static void do_ota(void) {
    ESP_LOGI(TAG, "Starting OTA from %s", FIRMWARE_URL);
    esp_http_client_config_t http_config = {
        .url = FIRMWARE_URL,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 30000,
        .keep_alive_enable = true,
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
    vTaskDelay(FIRST_CHECK_DELAY_MS / portTICK_PERIOD_MS);
    while (1) {
        char remote[32];
        if (fetch_latest_version(remote, sizeof(remote)) == ESP_OK) {
            if (ota_semver_cmp(remote, FW_VERSION) > 0) {
                ESP_LOGI(TAG, "Update available: running %s, latest %s", FW_VERSION, remote);
                do_ota();   /* reboots on success; falls through to retry on failure */
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
    xTaskCreate(ota_task, "ota_update", 8 * 1024, NULL, 2, NULL);
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
