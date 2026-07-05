//
// door_control.c — door state machine, relay, reed sensor, failsafe timer.
//
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "door_control.h"
#include "hap_garage.h"
#include "board.h"
#include "settings.h"

static const char *TAG = "HAP Garage";

static QueueHandle_t gpio_evt_queue = NULL;
static uint8_t door_state = HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_UNKNOWN;
static esp_timer_handle_t oneshot_timer;

/* Auto-close fault detection + backstops. */
#define AUTO_CLOSE_MAX_FAILS    3       /* consecutive failed closes → latch off */
#define AUTO_CLOSE_MAX_PER_HOUR 10      /* hard cap on auto-close pulses per hour */
#define AUTO_CLOSE_MIN_GAP_US   60000000LL  /* >= 60 s between auto-close pulses */

static int64_t  door_open_since_us;        /* set when OPEN, 0 when CLOSED */
static bool     left_open_tripped;
/* RAM-only: a reboot re-arms auto-close; if the sensor is still stuck it simply re-latches after 3 more attempts. */
static bool     auto_close_faulted;
static uint32_t auto_close_fail_count;
static int64_t  auto_close_verify_deadline_us;  /* 0 = no verify pending */
static int64_t  last_auto_close_pulse_us;
static int64_t  hour_window_start_us;
static uint32_t attempts_this_hour;

uint8_t door_get_state(void) { return door_state; }

/* Door-move / auto-close-verify window in microseconds, from settings. */
static int64_t door_move_us(void) {
  return (int64_t)settings_get_door_move_sec() * 1000000LL;
}

//
// Set the logical door state and maintain derived state: the open-since
// timestamp, the auto-close fault latch (auto-clears on a genuine CLOSED),
// and the left-open alert reset.
//
static void door_set_state(uint8_t s) {
  if (s == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN) {
    if (door_open_since_us == 0) door_open_since_us = esp_timer_get_time();
  } else if (s == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED) {
    door_open_since_us = 0;
    auto_close_fail_count = 0;               /* genuine close → failures are no longer consecutive */
    auto_close_verify_deadline_us = 0;       /* cancel any pending auto-close verification */
    if (auto_close_faulted) {                /* was latched → clear the fault + indicator */
      auto_close_faulted = false;
      hap_garage_send_status_fault(0);
      ESP_LOGI(TAG, "Auto-close fault cleared (door reported CLOSED)");
    }
    if (left_open_tripped) {
      left_open_tripped = false;
      hap_garage_send_left_open(false);
    }
  }
  door_state = s;
}

/* Reed switch: with this install's magnet/contact polarity the pin reads
 * low when the door is closed and high when open, which maps straight onto
 * DOOR_CLOSED (0) / DOOR_OPEN (1) — no inversion. */
static uint8_t read_door_sensor(void) { return gpio_get_level(DOOR_SENSOR_GPIO); }

static void kick_door_relay(void) {
  gpio_set_level(DOOR_RELAY_GPIO, 1);
  vTaskDelay(pdMS_TO_TICKS(settings_get_relay_pulse_ms()));
  gpio_set_level(DOOR_RELAY_GPIO, 0);
}

static void timer_callback(void *args) {
  ESP_LOGI(TAG, "Timer expired. Setting state according to sensor.");
  if (read_door_sensor() == DOOR_OPEN) {
    hap_garage_send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN);
    hap_garage_send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN);
    door_set_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN);
  } else {
    hap_garage_send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED);
    hap_garage_send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
    door_set_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
  }
}

static const esp_timer_create_args_t oneshot_timer_args = {
  .callback = &timer_callback, .name = "one-shot"
};

static void IRAM_ATTR gpio_isr_handler(void *arg) {
  uint32_t gpio_num = (uint32_t) arg;
  xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void door_handle_homekit_request(void) {
  if (read_door_sensor() == DOOR_OPEN ||
      door_state == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING) {
    kick_door_relay();
    door_set_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING);
    ESP_LOGI(TAG, "Received request to close door. Closing.");
  } else {
    kick_door_relay();
    door_set_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING);
    ESP_LOGI(TAG, "Received request to open door. Opening.");
  }
  esp_timer_start_once(oneshot_timer, door_move_us());
}

void door_control_init(void) {
  gpio_config_t io_conf = {0};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
  gpio_config(&io_conf);

  gpio_set_level(RUNNING_LED_GPIO, 1);
  gpio_set_level(DOOR_LED_GPIO, 0);
  gpio_set_level(DOOR_RELAY_GPIO, 0);

  io_conf.intr_type = GPIO_INTR_ANYEDGE;
  io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = 1;
  gpio_config(&io_conf);

  gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  gpio_isr_handler_add(DOOR_SENSOR_GPIO, gpio_isr_handler, (void *) DOOR_SENSOR_GPIO);

  esp_timer_create(&oneshot_timer_args, &oneshot_timer);

  uint8_t sensor = read_door_sensor();
  ESP_LOGI(TAG, "Initial door state is: (%s)", sensor == DOOR_OPEN ? "open" : "closed");
  if (sensor == DOOR_OPEN) {
    door_set_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN);
    hap_garage_send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN);
    hap_garage_send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN);
    gpio_set_level(DOOR_LED_GPIO, 1);
  } else {
    door_set_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
    hap_garage_send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
    hap_garage_send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED);
  }
}

/* Ignore reed edges that arrive within this window of the last accepted edge. */
#define DOOR_DEBOUNCE_US 50000

//
// Public getters for the web/HAP layers.
//
bool door_auto_close_faulted(void) { return auto_close_faulted; }

void door_auto_close_fault_reset(void) {
  auto_close_faulted = false;
  auto_close_fail_count = 0;
  auto_close_verify_deadline_us = 0;
  hap_garage_send_status_fault(0);
  ESP_LOGI(TAG, "Auto-close fault reset (manual)");
}

int64_t door_open_duration_us(void) {
  return door_open_since_us ? (esp_timer_get_time() - door_open_since_us) : 0;
}

//
// Periodic evaluator (~1 Hz from the event loop). Runs auto-close, the left-open
// alert, and auto-close fault detection. Single-threaded within the loop, so it
// shares no locks with the reed ISR.
//
static void door_control_tick(void) {
  int64_t now = esp_timer_get_time();

  /* Verify a pending auto-close: did the door actually reach CLOSED? */
  if (auto_close_verify_deadline_us != 0 && now >= auto_close_verify_deadline_us) {
    auto_close_verify_deadline_us = 0;
    if (read_door_sensor() == DOOR_CLOSED) {
      auto_close_fail_count = 0;               /* success */
    } else {
      auto_close_fail_count++;
      settings_incr_fault();
      ESP_LOGE(TAG, "Auto-close attempt failed (%u/%u) — sensor still OPEN",
               auto_close_fail_count, AUTO_CLOSE_MAX_FAILS);
      if (auto_close_fail_count >= AUTO_CLOSE_MAX_FAILS && !auto_close_faulted) {
        auto_close_faulted = true;
        ESP_LOGE(TAG, "Auto-close LATCHED OFF — reed sensor may be stuck");
        hap_garage_send_status_fault(1);
      }
    }
  }

  /* Coarse periodic flush so counters survive power loss (~5 min). */
  static int64_t last_flush_us = 0;
  if (now - last_flush_us > 300000000LL) { last_flush_us = now; settings_flush(); }

  if (door_open_since_us == 0) return;         /* door closed → nothing to time */
  int64_t open_us = now - door_open_since_us;

  /* Left-open alert — fires regardless of lockout. */
  uint32_t left = settings_get_left_open_sec();
  if (left > 0 && !left_open_tripped && open_us >= (int64_t)left * 1000000LL) {
    left_open_tripped = true;
    settings_incr_left_open();
    ESP_LOGW(TAG, "Door left open > %u s — tripping alert", left);
    hap_garage_send_left_open(true);
  }

  /* Auto-close — suspended by lockout (absolute) and by the fault latch. */
  if (settings_get_lockout() || auto_close_faulted) return;
  uint32_t ac = settings_get_auto_close_sec();
  if (ac == 0 || open_us < (int64_t)ac * 1000000LL) return;
  if (auto_close_verify_deadline_us != 0) return;          /* attempt in flight */

  /* Backstops: min gap between pulses + hourly cap. */
  if (now - last_auto_close_pulse_us < AUTO_CLOSE_MIN_GAP_US) return;
  if (now - hour_window_start_us > 3600000000LL) { hour_window_start_us = now; attempts_this_hour = 0; }
  if (attempts_this_hour >= AUTO_CLOSE_MAX_PER_HOUR) return;

  if (read_door_sensor() != DOOR_OPEN) return;             /* nothing to close */

  ESP_LOGI(TAG, "Auto-closing door after %lld s open", open_us / 1000000LL);
  last_auto_close_pulse_us = now;
  attempts_this_hour++;
  settings_incr_auto_close();
  kick_door_relay();
  door_set_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING);
  hap_garage_send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING);
  hap_garage_send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED);
  esp_timer_start_once(oneshot_timer, door_move_us());
  auto_close_verify_deadline_us = now + door_move_us();
}

void door_control_run_loop(void) {
  uint32_t io_num;
  uint8_t last_sensor = read_door_sensor();
  int64_t last_change_us = 0;

  while (1) {
    if (xQueueReceive(gpio_evt_queue, &io_num, pdMS_TO_TICKS(1000)) == pdFALSE) {
      door_control_tick();   /* periodic: auto-close / left-open / fault checks */
      continue;
    }

    int64_t now = esp_timer_get_time();
    if (now - last_change_us < DOOR_DEBOUNCE_US) {
      continue;  // debounce: too soon after the last accepted edge
    }

    uint8_t sensor = read_door_sensor();
    if (sensor == last_sensor) {
      continue;  // spurious edge, no real change
    }
    last_sensor = sensor;
    last_change_us = now;

    ESP_LOGI(TAG, "Door sensor %s (state was %s)",
             sensor == DOOR_OPEN ? "open" : "closed",
             door_get_state() == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN ? "open" : "not-open");

    if (sensor == DOOR_OPEN) {
      settings_incr_cycle();
      gpio_set_level(DOOR_LED_GPIO, 1);
      hap_garage_send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN);
      hap_garage_send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN);
      door_set_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN);
    } else {
      gpio_set_level(DOOR_LED_GPIO, 0);
      hap_garage_send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED);
      hap_garage_send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
      door_set_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
    }
  }
}
