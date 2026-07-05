//
// ota_upload.c — receive a streamed firmware image and flash the inactive OTA slot.
//
#include <string.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <esp_app_desc.h>
#include <esp_image_format.h>

#include "ota_upload.h"

static const char *TAG = "OTA-Upload";

/* The app descriptor sits right after the image header + first segment header. */
#define APP_DESC_OFFSET (sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t))
#define HEADER_CAPTURE  (APP_DESC_OFFSET + sizeof(esp_app_desc_t))
#define EXPECT_PROJECT  "garage"

static bool                 s_busy;
static esp_ota_handle_t     s_handle;
static const esp_partition_t *s_part;
static uint8_t              s_hdr[HEADER_CAPTURE];
static size_t               s_hdr_len;
static bool                 s_hdr_checked;

bool ota_upload_in_progress(void) { return s_busy; }

esp_err_t ota_upload_begin(void) {
  if (s_busy) return ESP_ERR_INVALID_STATE;
  s_part = esp_ota_get_next_update_partition(NULL);
  if (!s_part) { ESP_LOGE(TAG, "no OTA partition available"); return ESP_FAIL; }
  esp_err_t e = esp_ota_begin(s_part, OTA_SIZE_UNKNOWN, &s_handle);
  if (e != ESP_OK) { ESP_LOGE(TAG, "esp_ota_begin: %s", esp_err_to_name(e)); return e; }
  s_hdr_len = 0;
  s_hdr_checked = false;
  s_busy = true;
  ESP_LOGI(TAG, "OTA upload begin -> partition '%s'", s_part->label);
  return ESP_OK;
}

//
// Validate the captured app descriptor: magic word + project name.
//
static esp_err_t check_header(void) {
  const esp_app_desc_t *d = (const esp_app_desc_t *)(s_hdr + APP_DESC_OFFSET);
  if (d->magic_word != ESP_APP_DESC_MAGIC_WORD) {
    ESP_LOGE(TAG, "bad app-desc magic 0x%08x", (unsigned)d->magic_word);
    return ESP_ERR_INVALID_ARG;
  }
  if (strncmp(d->project_name, EXPECT_PROJECT, sizeof(d->project_name)) != 0) {
    ESP_LOGE(TAG, "wrong project_name '%.*s' (expected '%s')",
             (int)sizeof(d->project_name), d->project_name, EXPECT_PROJECT);
    return ESP_ERR_INVALID_ARG;
  }
  ESP_LOGI(TAG, "image accepted: project='%s' version='%.*s'",
           EXPECT_PROJECT, (int)sizeof(d->version), d->version);
  s_hdr_checked = true;
  return ESP_OK;
}

esp_err_t ota_upload_write(const void *data, size_t len) {
  if (!s_busy) return ESP_ERR_INVALID_STATE;

  /* Capture the leading header bytes for the app-desc guard (a copy — the full
   * chunk is still written below). Validate as soon as we have enough. */
  if (s_hdr_len < HEADER_CAPTURE) {
    size_t n = HEADER_CAPTURE - s_hdr_len;
    if (n > len) n = len;
    memcpy(s_hdr + s_hdr_len, data, n);
    s_hdr_len += n;
    if (s_hdr_len >= HEADER_CAPTURE && !s_hdr_checked) {
      esp_err_t e = check_header();
      if (e != ESP_OK) return e;
    }
  }
  return esp_ota_write(s_handle, data, len);
}

esp_err_t ota_upload_end(void) {
  if (!s_busy) return ESP_ERR_INVALID_STATE;
  if (!s_hdr_checked) {          /* image too small to even contain a descriptor */
    ESP_LOGE(TAG, "image too small / descriptor never validated");
    ota_upload_abort();
    return ESP_ERR_INVALID_SIZE;
  }
  esp_err_t e = esp_ota_end(s_handle);
  if (e != ESP_OK) { ESP_LOGE(TAG, "esp_ota_end: %s", esp_err_to_name(e)); s_busy = false; return e; }
  e = esp_ota_set_boot_partition(s_part);
  if (e != ESP_OK) ESP_LOGE(TAG, "esp_ota_set_boot_partition: %s", esp_err_to_name(e));
  s_busy = false;
  return e;
}

void ota_upload_abort(void) {
  if (s_busy) { esp_ota_abort(s_handle); s_busy = false; ESP_LOGW(TAG, "OTA upload aborted"); }
}
