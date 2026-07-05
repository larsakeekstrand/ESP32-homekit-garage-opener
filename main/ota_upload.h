//
// ota_upload.h — receive a streamed firmware image and flash the inactive OTA slot.
//
#pragma once

#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

/* Begin an upload: pick the next OTA partition and open an esp_ota session.
 * Returns ESP_ERR_INVALID_STATE if one is already in progress. */
esp_err_t ota_upload_begin(void);

/* Feed the next chunk of the image. Once enough header bytes have arrived it
 * validates the app descriptor (magic + project_name == "garage") and returns
 * ESP_ERR_INVALID_ARG on mismatch. */
esp_err_t ota_upload_write(const void *data, size_t len);

/* Finalize: validate the image (esp_ota_end) and set it as the boot partition.
 * The caller reboots afterwards. */
esp_err_t ota_upload_end(void);

/* Abort an in-progress upload (esp_ota_abort) and clear the busy state. */
void ota_upload_abort(void);

/* True while an upload session is open. */
bool ota_upload_in_progress(void);
