//
// ota_update.h — self-polling GitHub-release OTA updater.
//
#pragma once

/* Spawn the background task that periodically checks GitHub Releases and
 * applies a newer firmware via HTTPS OTA. */
void ota_update_start(void);

/* Confirm the currently-running image after a healthy boot, cancelling any
 * pending rollback. Safe to call when rollback is not pending (no-op). */
void ota_update_mark_valid(void);
