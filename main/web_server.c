//
// web_server.c — authenticated local status/config web page (HTTP :80, LAN-only).
//
#include <string.h>
#include <stdlib.h>
#include <esp_log.h>
#include <esp_http_server.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <esp_mac.h>
#include <esp_system.h>
#include <esp_timer.h>
#include "mbedtls/base64.h"

#include "web_server.h"
#include "settings.h"
#include "door_control.h"
#include "hap_garage.h"
#include "dht_sensor.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "ota_upload.h"

static const char *TAG = "WebServer";
static httpd_handle_t server = NULL;

// ---- static pages --------------------------------------------------------

// First-run "create password" page (served while no password is set).
static const char SETUP_HTML[] =
"<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'>"
"<title>Garage setup</title>"
"<style>body{font-family:system-ui;max-width:32rem;margin:2rem auto;padding:0 1rem}"
"input,button{font-size:1rem;padding:.5rem;margin:.25rem 0;width:100%;box-sizing:border-box}</style>"
"<h1>Set admin password</h1>"
"<p>This device has no password yet. Set one now (min 4 characters). "
"Do this promptly — anyone on this network can set it until you do.</p>"
"<input id=pw type=password placeholder='New password'>"
"<button onclick=\"fetch('/api/password',{method:'POST',body:'password='+encodeURIComponent(pw.value)})"
".then(r=>r.ok?location.reload():r.text().then(t=>alert(t)))\">Save</button>";

// Main status/config page. Values are filled by JS from /api/status.
static const char PAGE_HTML[] =
"<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'>"
"<title>Garage</title>"
"<style>body{font-family:system-ui;max-width:36rem;margin:1.5rem auto;padding:0 1rem}"
"h1{font-size:1.4rem}.card{border:1px solid #ccc;border-radius:8px;padding:1rem;margin:1rem 0}"
"tr td:first-child{color:#666;padding-right:1rem}#fault{display:none;background:#c0392b;color:#fff;"
"padding:1rem;border-radius:8px}input,button{font-size:1rem;padding:.4rem}</style>"
"<h1>Garage Opener</h1>"
"<div id=fault>⚠ Auto-close FAULTED — reed sensor may be stuck. "
"<button onclick=\"post('/api/fault-reset','')\">Reset</button></div>"
"<div class=card><h3>Status</h3><table>"
"<tr><td>Door</td><td id=door>…</td></tr>"
"<tr><td>Open for</td><td id=openfor>…</td></tr>"
"<tr><td>Lockout</td><td><input type=checkbox id=lockout onchange=\"post('/api/lockout','on='+(this.checked?1:0))\"></td></tr>"
"<tr><td>Wi-Fi</td><td id=wifi>…</td></tr>"
"<tr><td>Temp</td><td id=temp>…</td></tr>"
"<tr><td>Humidity</td><td id=hum>…</td></tr>"
"<tr><td>Uptime</td><td id=uptime>…</td></tr></table></div>"
"<div class=card><h3>System</h3><table>"
"<tr><td>Firmware</td><td id=fw>…</td></tr>"
"<tr><td>Free heap</td><td id=heap>…</td></tr>"
"<tr><td>MAC</td><td id=mac>…</td></tr>"
"<tr><td>IP</td><td id=ip>…</td></tr>"
"<tr><td>Reset reason</td><td id=rst>…</td></tr>"
"<tr><td>Auto-update</td><td><input type=checkbox id=autoota onchange=\"post('/api/auto-ota','on='+(this.checked?1:0))\"> <span id=autoota_l></span></td></tr></table></div>"
"<div class=card><h3>Counters</h3><table>"
"<tr><td>Door cycles</td><td id=cycles>…</td></tr>"
"<tr><td>Auto-closes</td><td id=ac>…</td></tr>"
"<tr><td>Left-open trips</td><td id=lo>…</td></tr>"
"<tr><td>Faults</td><td id=flt>…</td></tr></table></div>"
"<div class=card><h3>Settings</h3>"
"<p>Auto-close after <input id=ac_sec type=number min=0 max=3600 style=width:6rem> s (0 = off)</p>"
"<p>Left-open alert after <input id=lo_sec type=number min=0 max=3600 style=width:6rem> s (0 = off)</p>"
"<p id=hint style=color:#c0392b></p>"
"<button onclick=\"post('/api/settings','auto_close_sec='+ac_sec.value+'&left_open_sec='+lo_sec.value)\">Save</button>"
"</div>"
"<script>"
"function post(u,b){return fetch(u,{method:'POST',body:b}).then(r=>{if(!r.ok)r.text().then(t=>alert(t));setTimeout(refresh,300)})}"
"function fmt(s){s=Math.floor(s);let h=Math.floor(s/3600),m=Math.floor(s%3600/60);return h+'h '+m+'m '+(s%60)+'s'}"
"function refresh(){fetch('/api/status').then(r=>r.json()).then(d=>{"
"door.textContent=d.door_state;openfor.textContent=d.door_open_seconds?fmt(d.door_open_seconds):'—';"
"lockout.checked=d.lockout;wifi.textContent=d.rssi+' dBm ('+d.rssi_quality+')';"
"temp.textContent=d.temperature.toFixed(1)+' °C';hum.textContent=d.humidity.toFixed(0)+' %';"
"uptime.textContent=fmt(d.uptime_seconds);fw.textContent=d.fw_version;heap.textContent=d.free_heap+' B';"
"mac.textContent=d.mac;ip.textContent=d.ip;rst.textContent=d.reset_reason;"
"autoota.checked=!d.auto_ota_paused;autoota_l.textContent=d.auto_ota_paused?'PAUSED':'on';"
"cycles.textContent=d.cycles;ac.textContent=d.auto_closes;lo.textContent=d.left_open_trips;flt.textContent=d.faults;"
"fault.style.display=d.auto_close_faulted?'block':'none';"
"if(document.activeElement!=ac_sec)ac_sec.value=d.auto_close_sec;"
"if(document.activeElement!=lo_sec)lo_sec.value=d.left_open_sec;"
"hint.textContent=(d.left_open_sec&&d.auto_close_sec&&d.left_open_sec>=d.auto_close_sec)?"
"'Left-open >= auto-close: the alert may never fire.':'';"
"})}refresh();setInterval(refresh,3000);</script>";

// ---- helpers -------------------------------------------------------------

static const char *reset_reason_str(void) {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   return "power-on";
    case ESP_RST_SW:        return "software";
    case ESP_RST_PANIC:     return "panic";
    case ESP_RST_INT_WDT:   return "int-wdt";
    case ESP_RST_TASK_WDT:  return "task-wdt";
    case ESP_RST_BROWNOUT:  return "brownout";
    case ESP_RST_DEEPSLEEP: return "deep-sleep";
    default:                return "other";
  }
}

static const char *door_state_str(uint8_t s) {
  switch (s) {
    case HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN:    return "open";
    case HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED:  return "closed";
    case HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING: return "opening";
    case HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING: return "closing";
    case HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_STOPPED: return "stopped";
    default:                                                 return "unknown";
  }
}

// Basic Auth. Returns true if authorized. (TOFU: caller checks password_is_set first.)
static bool check_auth(httpd_req_t *req) {
  char hdr[160];
  if (httpd_req_get_hdr_value_str(req, "Authorization", hdr, sizeof(hdr)) != ESP_OK) return false;
  if (strncmp(hdr, "Basic ", 6) != 0) return false;
  unsigned char dec[120];
  size_t olen = 0;
  if (mbedtls_base64_decode(dec, sizeof(dec) - 1, &olen,
                            (const unsigned char *)hdr + 6, strlen(hdr + 6)) != 0) return false;
  dec[olen] = '\0';
  char *colon = strchr((char *)dec, ':');
  if (!colon) return false;
  *colon = '\0';
  if (strcmp((char *)dec, "admin") != 0) return false;
  return settings_password_check(colon + 1);
}

// Enforce auth; on failure send 401 and return ESP_FAIL.
static esp_err_t require_auth(httpd_req_t *req) {
  if (settings_password_is_set() && check_auth(req)) return ESP_OK;
  if (!settings_password_is_set()) return ESP_OK;   /* TOFU window: allow */
  httpd_resp_set_status(req, "401 Unauthorized");
  httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"Garage\"");
  httpd_resp_send(req, "Authentication required", HTTPD_RESP_USE_STRLEN);
  return ESP_FAIL;
}

// Read a request body into buf (NUL-terminated). Returns length or -1.
static int read_body(httpd_req_t *req, char *buf, size_t len) {
  int total = req->content_len;
  if (total >= (int)len) total = len - 1;
  int off = 0;
  while (off < total) {
    int r = httpd_req_recv(req, buf + off, total - off);
    if (r <= 0) return -1;
    off += r;
  }
  buf[off] = '\0';
  return off;
}

// ---- handlers ------------------------------------------------------------

static esp_err_t root_get(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  if (!settings_password_is_set()) {
    httpd_resp_send(req, SETUP_HTML, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
  }
  if (require_auth(req) != ESP_OK) return ESP_OK;
  httpd_resp_send(req, PAGE_HTML, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

static esp_err_t status_get(httpd_req_t *req) {
  if (require_auth(req) != ESP_OK) return ESP_OK;

  float t = 0, h = 0;
  dht_sensor_get(&t, &h);

  int rssi = 0;
  const char *q = "n/a";
  wifi_ap_record_t ap;
  if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
    rssi = ap.rssi;
    q = rssi >= -55 ? "excellent" : rssi >= -67 ? "good" : rssi >= -75 ? "fair" : "weak";
  }

  esp_netif_ip_info_t ip = {0};
  esp_netif_t *nif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  if (nif) esp_netif_get_ip_info(nif, &ip);

  uint8_t mac[6] = {0};
  esp_read_mac(mac, ESP_MAC_WIFI_STA);

  settings_counters_t c;
  settings_get_counters(&c);

  static char buf[800];
  snprintf(buf, sizeof(buf),
    "{\"door_state\":\"%s\",\"door_open_seconds\":%lld,\"lockout\":%s,"
    "\"rssi\":%d,\"rssi_quality\":\"%s\",\"temperature\":%.1f,\"humidity\":%.1f,"
    "\"uptime_seconds\":%lld,\"fw_version\":\"%s\",\"free_heap\":%u,"
    "\"mac\":\"%02x:%02x:%02x:%02x:%02x:%02x\",\"ip\":\"" IPSTR "\",\"reset_reason\":\"%s\","
    "\"cycles\":%u,\"auto_closes\":%u,\"left_open_trips\":%u,\"faults\":%u,"
    "\"auto_close_faulted\":%s,\"auto_close_sec\":%u,\"left_open_sec\":%u,"
    "\"auto_ota_paused\":%s}",
    door_state_str(door_get_state()), door_open_duration_us() / 1000000LL,
    settings_get_lockout() ? "true" : "false",
    rssi, q, t, h,
    esp_timer_get_time() / 1000000LL, FW_VERSION, (unsigned)esp_get_free_heap_size(),
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], IP2STR(&ip.ip), reset_reason_str(),
    (unsigned)c.cycles, (unsigned)c.auto_closes, (unsigned)c.left_open_trips, (unsigned)c.faults,
    door_auto_close_faulted() ? "true" : "false",
    (unsigned)settings_get_auto_close_sec(), (unsigned)settings_get_left_open_sec(),
    settings_get_auto_ota_paused() ? "true" : "false");

  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

static esp_err_t settings_post(httpd_req_t *req) {
  if (require_auth(req) != ESP_OK) return ESP_OK;
  char buf[128];
  if (read_body(req, buf, sizeof(buf)) < 0) { httpd_resp_send_500(req); return ESP_OK; }
  char v[16];
  uint32_t ac = settings_get_auto_close_sec(), lo = settings_get_left_open_sec();
  if (httpd_query_key_value(buf, "auto_close_sec", v, sizeof(v)) == ESP_OK) ac = atoi(v);
  if (httpd_query_key_value(buf, "left_open_sec",  v, sizeof(v)) == ESP_OK) lo = atoi(v);
  settings_set_timeouts(ac, lo);   /* clamps internally */
  httpd_resp_sendstr(req, "ok");
  return ESP_OK;
}

static esp_err_t lockout_post(httpd_req_t *req) {
  if (require_auth(req) != ESP_OK) return ESP_OK;
  char buf[32];
  if (read_body(req, buf, sizeof(buf)) < 0) { httpd_resp_send_500(req); return ESP_OK; }
  char v[8];
  bool on = settings_get_lockout();
  if (httpd_query_key_value(buf, "on", v, sizeof(v)) == ESP_OK) on = (atoi(v) != 0);
  settings_set_lockout(on);
  hap_garage_send_lockout(on);     /* sync the Home-app Switch tile */
  httpd_resp_sendstr(req, "ok");
  return ESP_OK;
}

static esp_err_t fault_reset_post(httpd_req_t *req) {
  if (require_auth(req) != ESP_OK) return ESP_OK;
  door_auto_close_fault_reset();
  httpd_resp_sendstr(req, "ok");
  return ESP_OK;
}

static esp_err_t password_post(httpd_req_t *req) {
  bool first = !settings_password_is_set();
  if (!first && require_auth(req) != ESP_OK) return ESP_OK;
  char buf[96];
  if (read_body(req, buf, sizeof(buf)) < 0) { httpd_resp_send_500(req); return ESP_OK; }
  char pw[64];
  if (httpd_query_key_value(buf, "password", pw, sizeof(pw)) != ESP_OK || strlen(pw) < 4) {
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_sendstr(req, "password too short (min 4)");
    return ESP_OK;
  }
  settings_password_set_value(pw);
  httpd_resp_sendstr(req, "ok");
  return ESP_OK;
}

//
// POST /api/ota — stream a firmware image into the inactive slot, then reboot.
//
static esp_err_t ota_post(httpd_req_t *req) {
  if (require_auth(req) != ESP_OK) return ESP_OK;
  if (ota_upload_in_progress()) {
    httpd_resp_set_status(req, "409 Conflict");
    httpd_resp_sendstr(req, "an OTA upload is already in progress");
    return ESP_OK;
  }
  if (ota_upload_begin() != ESP_OK) { httpd_resp_send_500(req); return ESP_OK; }

  char buf[1024];
  int remaining = req->content_len;
  esp_err_t err = ESP_OK;
  while (remaining > 0) {
    int want = remaining < (int)sizeof(buf) ? remaining : (int)sizeof(buf);
    int r = httpd_req_recv(req, buf, want);
    if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
    if (r <= 0) { err = ESP_FAIL; break; }
    err = ota_upload_write(buf, r);
    if (err != ESP_OK) break;
    remaining -= r;
  }
  if (err != ESP_OK) {
    ota_upload_abort();
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_sendstr(req, "OTA write/validation failed (wrong or corrupt image?)");
    return ESP_OK;
  }
  if (ota_upload_end() != ESP_OK) {
    ota_upload_abort();
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_sendstr(req, "OTA finalize failed (image invalid or too small)");
    return ESP_OK;
  }

  settings_set_auto_ota_paused(true);   /* so the poller won't revert the beta */
  httpd_resp_set_type(req, "application/json");
  httpd_resp_sendstr(req, "{\"status\":\"ok\",\"rebooting\":true}");
  ESP_LOGI(TAG, "OTA upload applied; rebooting into new image");
  vTaskDelay(pdMS_TO_TICKS(500));       /* let the response flush before reboot */
  esp_restart();
  return ESP_OK;                        /* not reached */
}

//
// POST /api/auto-ota — enable/disable the GitHub self-poller. Body: on=0|1
// (on=1 => auto-updates enabled/paused=false; on=0 => paused=true).
//
static esp_err_t auto_ota_post(httpd_req_t *req) {
  if (require_auth(req) != ESP_OK) return ESP_OK;
  char buf[32];
  if (read_body(req, buf, sizeof(buf)) < 0) { httpd_resp_send_500(req); return ESP_OK; }
  char v[8];
  bool enabled = !settings_get_auto_ota_paused();
  if (httpd_query_key_value(buf, "on", v, sizeof(v)) == ESP_OK) enabled = (atoi(v) != 0);
  settings_set_auto_ota_paused(!enabled);
  httpd_resp_sendstr(req, "ok");
  return ESP_OK;
}

// ---- lifecycle -----------------------------------------------------------

void web_server_start(void) {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.max_uri_handlers = 10;
  config.lru_purge_enable = true;
  if (httpd_start(&server, &config) != ESP_OK) {
    ESP_LOGE(TAG, "httpd start failed");
    return;
  }
  httpd_uri_t uris[] = {
    { .uri = "/",               .method = HTTP_GET,  .handler = root_get },
    { .uri = "/api/status",     .method = HTTP_GET,  .handler = status_get },
    { .uri = "/api/settings",   .method = HTTP_POST, .handler = settings_post },
    { .uri = "/api/lockout",    .method = HTTP_POST, .handler = lockout_post },
    { .uri = "/api/fault-reset",.method = HTTP_POST, .handler = fault_reset_post },
    { .uri = "/api/password",   .method = HTTP_POST, .handler = password_post },
    { .uri = "/api/ota",        .method = HTTP_POST, .handler = ota_post },
    { .uri = "/api/auto-ota",   .method = HTTP_POST, .handler = auto_ota_post },
  };
  for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++)
    httpd_register_uri_handler(server, &uris[i]);
  ESP_LOGI(TAG, "Web server started on port 80");
}
