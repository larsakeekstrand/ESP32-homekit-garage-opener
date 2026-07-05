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
#include "ota_update.h"

static const char *TAG = "WebServer";
static httpd_handle_t server = NULL;

/* The HomeKit (HAP) server already owns port 80 (CONFIG_HAP_HTTP_SERVER_PORT)
 * and the default esp_http_server control port 32768, so this diagnostics/config
 * server must use a different server AND control port — otherwise httpd_start
 * fails with "address already in use" and the web page never comes up. */
#define WEB_SERVER_PORT      8080
#define WEB_SERVER_CTRL_PORT 32769

// ---- static pages --------------------------------------------------------


// First-run "create password" page (served while no password is set).
static const char SETUP_HTML[] =
"<!doctype html><html><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\"><title>Garage setup</title><style>:root{--bg:#0d0e12;--ink:#eceef3;--mut:#868c9b;--amber:#f5a524;--line:rgba(255,255,255,.09);--sans:ui-sans-serif,-apple-system,\"Segoe UI\",Roboto,sans-serif;--mono:ui-monospace,\"SF Mono\",Menlo,Consolas,monospace}*{box-sizing:border-box}html,body{margin:0}body{font-family:var(--sans);color:var(--ink);background:var(--bg);min-height:100vh;display:grid;place-items:center;padding:24px;background-image:radial-gradient(120% 80% at 50% -10%,rgba(245,165,36,.12),transparent 60%)}.box{width:100%;max-width:400px;background:rgba(255,255,255,.035);border:1px solid var(--line);border-radius:20px;padding:28px}.tag{font-family:var(--mono);letter-spacing:.2em;text-transform:uppercase;font-size:11px;color:var(--amber);margin-bottom:14px}h1{font-size:22px;margin:0 0 10px}p{color:var(--mut);font-size:13px;line-height:1.55;margin:0 0 18px}input{width:100%;background:#15171d;border:1px solid var(--line);color:var(--ink);font-size:15px;padding:12px 14px;border-radius:12px;outline:none}input:focus{border-color:var(--amber)}button{width:100%;margin-top:12px;border:0;border-radius:12px;padding:12px;background:var(--amber);color:#241a06;font-weight:600;font-size:15px;cursor:pointer}</style></head><body><div class=\"box\"><div class=\"tag\">Garage Opener · Setup</div><h1>Set a password</h1><p>This device has no password yet. Choose one (min 4 characters). Do it promptly — anyone on this network can set it until you do.</p><input id=\"pw\" type=\"password\" placeholder=\"New password\" autofocus><button onclick=\"fetch('/api/password',{method:'POST',body:'password='+encodeURIComponent(pw.value)}).then(r=>r.ok?location.reload():r.text().then(t=>alert(t)))\">Save password</button></div></body></html>\n";

// Main status/config page. Values are filled by JS from /api/status.
static const char PAGE_HTML[] =
"<!doctype html><html><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\"><title>Garage</title>\n"
"<style>\n"
":root{\n"
"  --bg:#0d0e12; --panel:rgba(255,255,255,.035); --line:rgba(255,255,255,.09);\n"
"  --ink:#eceef3; --mut:#868c9b; --dim:#5b616f;\n"
"  --amber:#f5a524; --ok:#3ddb98; --open:#ffb020; --bad:#ff6a6a;\n"
"  --sans:ui-sans-serif,-apple-system,\"Segoe UI\",Roboto,sans-serif;\n"
"  --mono:ui-monospace,\"SF Mono\",\"JetBrains Mono\",Menlo,Consolas,monospace;\n"
"}\n"
"*{box-sizing:border-box}\n"
"html,body{margin:0}\n"
"body{\n"
"  font-family:var(--sans); color:var(--ink); background:var(--bg);\n"
"  min-height:100vh; padding:clamp(16px,4vw,40px) 16px 48px;\n"
"  background-image:\n"
"    radial-gradient(120% 80% at 50% -10%, rgba(245,165,36,.10), transparent 60%),\n"
"    radial-gradient(90% 60% at 90% 0%, rgba(61,219,152,.06), transparent 55%);\n"
"  background-attachment:fixed;\n"
"}\n"
".wrap{max-width:760px;margin:0 auto}\n"
"/* header */\n"
".brand{display:flex;align-items:baseline;gap:10px;margin:0 4px 20px;\n"
"  font-family:var(--mono);letter-spacing:.18em;text-transform:uppercase;font-size:13px;color:var(--mut)}\n"
".brand b{color:var(--ink);font-weight:600;letter-spacing:.14em}\n"
".brand .ver{margin-left:auto;font-size:11px;color:var(--dim);letter-spacing:.1em}\n"
".brand .ver::before{content:\"fw \";color:var(--dim)}\n"
"/* fault */\n"
".alert{display:none;margin:0 0 18px;padding:14px 16px;border-radius:14px;\n"
"  background:linear-gradient(180deg,rgba(255,106,106,.16),rgba(255,106,106,.07));\n"
"  border:1px solid rgba(255,106,106,.4);color:#ffd9d9;font-size:14px;\n"
"  gap:12px;align-items:center;justify-content:space-between}\n"
".alert.on{display:flex}\n"
".alert b{color:#fff}\n"
".alert button{background:#ff6a6a;color:#2a0d0d}\n"
"/* hero */\n"
".hero{position:relative;display:flex;align-items:center;gap:20px;\n"
"  padding:26px 26px;border-radius:20px;margin-bottom:14px;overflow:hidden;\n"
"  background:linear-gradient(180deg,rgba(255,255,255,.05),rgba(255,255,255,.02));\n"
"  border:1px solid var(--line);box-shadow:0 20px 60px -30px rgba(0,0,0,.8)}\n"
".dot{width:16px;height:16px;border-radius:50%;flex:none;position:relative;\n"
"  background:var(--ok);box-shadow:0 0 0 5px rgba(61,219,152,.14),0 0 22px 2px rgba(61,219,152,.55)}\n"
".dot.open{background:var(--open);box-shadow:0 0 0 5px rgba(255,176,32,.14),0 0 22px 2px rgba(255,176,32,.6)}\n"
".dot.move{background:var(--amber);animation:pulse 1.1s ease-in-out infinite}\n"
".dot.bad{background:var(--bad);box-shadow:0 0 0 5px rgba(255,106,106,.14),0 0 22px 2px rgba(255,106,106,.6)}\n"
"@keyframes pulse{0%,100%{box-shadow:0 0 0 4px rgba(245,165,36,.10),0 0 18px 1px rgba(245,165,36,.5);opacity:1}\n"
"  50%{box-shadow:0 0 0 12px rgba(245,165,36,0),0 0 26px 4px rgba(245,165,36,.75);opacity:.7}}\n"
".hero .state{font-size:clamp(30px,7vw,44px);font-weight:700;letter-spacing:-.02em;line-height:1;text-transform:capitalize}\n"
".hero .sub{margin-top:8px;font-family:var(--mono);font-size:13px;color:var(--mut);letter-spacing:.03em}\n"
".hero .glyph{margin-left:auto;color:var(--ok);opacity:.92;transition:color .3s}\n"
"/* grid + cards */\n"
".grid{display:grid;grid-template-columns:repeat(2,1fr);gap:14px}\n"
".card{padding:18px 18px 16px;border-radius:18px;background:var(--panel);\n"
"  border:1px solid var(--line);backdrop-filter:blur(6px);\n"
"  animation:rise .5s cubic-bezier(.2,.7,.2,1) both}\n"
".card.wide{grid-column:1/-1}\n"
"@keyframes rise{from{opacity:0;transform:translateY(10px)}to{opacity:1;transform:none}}\n"
".grid .card:nth-child(1){animation-delay:.04s}.grid .card:nth-child(2){animation-delay:.09s}\n"
".grid .card:nth-child(3){animation-delay:.14s}.grid .card:nth-child(4){animation-delay:.19s}\n"
".grid .card:nth-child(5){animation-delay:.24s}.grid .card:nth-child(6){animation-delay:.29s}\n"
".card h3{margin:0 0 14px;font-family:var(--mono);font-size:11px;font-weight:600;\n"
"  letter-spacing:.2em;text-transform:uppercase;color:var(--amber)}\n"
".row{display:flex;align-items:center;justify-content:space-between;gap:12px;padding:7px 0;\n"
"  border-top:1px solid rgba(255,255,255,.05)}\n"
".row:first-of-type{border-top:0}\n"
".row .k{color:var(--mut);font-size:13px}\n"
".row .v{font-family:var(--mono);font-size:14px;color:var(--ink);font-variant-numeric:tabular-nums;text-align:right}\n"
"/* big stats (environment) */\n"
".stat{display:flex;gap:22px;flex-wrap:wrap}\n"
".stat .b{flex:1;min-width:110px}\n"
".stat .num{font-family:var(--mono);font-size:30px;font-weight:600;font-variant-numeric:tabular-nums;line-height:1}\n"
".stat .num .u{font-size:14px;color:var(--mut);margin-left:3px}\n"
".stat .lbl{margin-top:6px;font-size:11px;letter-spacing:.14em;text-transform:uppercase;color:var(--mut)}\n"
"/* switches */\n"
".sw{display:flex;align-items:center;justify-content:space-between;gap:12px;padding:10px 0;border-top:1px solid rgba(255,255,255,.05)}\n"
".sw:first-of-type{border-top:0}\n"
".sw .t{font-size:14px;color:var(--ink)}\n"
".sw .t small{display:block;color:var(--dim);font-size:11px;margin-top:2px}\n"
".sw .st{margin-left:auto;margin-right:12px;font-family:var(--mono);font-size:11px;letter-spacing:.12em;text-transform:uppercase;color:var(--mut)}\n"
"input[type=checkbox].tog{appearance:none;-webkit-appearance:none;flex:none;cursor:pointer;\n"
"  width:46px;height:27px;border-radius:20px;background:#20232c;border:1px solid var(--line);\n"
"  position:relative;transition:background .2s,border-color .2s}\n"
"input[type=checkbox].tog::after{content:\"\";position:absolute;top:2px;left:2px;width:21px;height:21px;\n"
"  border-radius:50%;background:#c8ccd6;transition:transform .2s,background .2s;box-shadow:0 1px 3px rgba(0,0,0,.5)}\n"
"input[type=checkbox].tog:checked{background:var(--amber);border-color:transparent}\n"
"input[type=checkbox].tog:checked::after{transform:translateX(19px);background:#241a06}\n"
"/* settings form */\n"
".field{display:flex;align-items:center;justify-content:space-between;gap:12px;padding:9px 0;border-top:1px solid rgba(255,255,255,.05)}\n"
".field:first-of-type{border-top:0}\n"
".field label{font-size:13px;color:var(--mut)}\n"
".num-in{display:flex;align-items:center;gap:7px;font-family:var(--mono);font-size:13px;color:var(--mut)}\n"
"input[type=number]{width:76px;background:#15171d;border:1px solid var(--line);color:var(--ink);\n"
"  font-family:var(--mono);font-size:14px;text-align:right;padding:7px 9px;border-radius:9px;outline:none}\n"
"input[type=number]:focus{border-color:var(--amber)}\n"
".hint{min-height:1em;margin:8px 0 0;font-size:12px;color:var(--open)}\n"
"button{font-family:var(--sans);font-size:13px;font-weight:600;border:0;border-radius:10px;\n"
"  padding:9px 16px;background:var(--ink);color:#111;cursor:pointer;transition:transform .12s,filter .12s}\n"
"button:hover{filter:brightness(1.08)}button:active{transform:scale(.97)}\n"
".save{margin-top:12px;width:100%;background:var(--amber);color:#241a06}\n"
"@media(max-width:560px){.grid{grid-template-columns:1fr}.hero .glyph{display:none}}\n"
"</style></head>\n"
"<body><div class=\"wrap\">\n"
"  <div class=\"brand\"><b>Garage&nbsp;Opener</b><span class=\"ver\" id=\"fw\">1.3.1</span></div>\n"
"\n"
"  <div class=\"alert\" id=\"fault\"><span><b>Auto-close faulted.</b> The reed sensor may be stuck.</span>\n"
"    <button onclick=\"post('/api/fault-reset','')\">Reset</button></div>\n"
"\n"
"  <div class=\"hero\">\n"
"    <span class=\"dot\" id=\"dot\"></span>\n"
"    <div><div class=\"state\" id=\"door\">closed</div><div class=\"sub\" id=\"openfor\"></div></div>\n"
"    <svg class=\"glyph\" id=\"glyph\" width=\"56\" height=\"56\" viewBox=\"0 0 24 24\" fill=\"none\" stroke=\"currentColor\" stroke-width=\"1.5\" stroke-linecap=\"round\" stroke-linejoin=\"round\"><path d=\"M3 21V9.5L12 4l9 5.5V21\"/><rect x=\"6.5\" y=\"11.5\" width=\"11\" height=\"9.5\" rx=\"1\"/><path d=\"M6.5 14.5h11M6.5 17.5h11\"/></svg>\n"
"  </div>\n"
"\n"
"  <div class=\"grid\">\n"
"    <div class=\"card\">\n"
"      <h3>Environment</h3>\n"
"      <div class=\"stat\">\n"
"        <div class=\"b\"><div class=\"num\"><span id=\"temp\">22.2</span><span class=\"u\">°C</span></div><div class=\"lbl\">Temp</div></div>\n"
"        <div class=\"b\"><div class=\"num\"><span id=\"hum\">62</span><span class=\"u\">%</span></div><div class=\"lbl\">Humidity</div></div>\n"
"      </div>\n"
"    </div>\n"
"\n"
"    <div class=\"card\">\n"
"      <h3>Controls</h3>\n"
"      <div class=\"sw\"><div class=\"t\">Lockout</div><span class=\"st\" id=\"lockout_l\">off</span><input type=\"checkbox\" class=\"tog\" id=\"lockout\" onchange=\"post('/api/lockout','on='+(this.checked?1:0))\"></div>\n"
"      <div class=\"sw\"><div class=\"t\">Auto-update</div><span class=\"st\" id=\"autoota_l\">on</span><input type=\"checkbox\" class=\"tog\" id=\"autoota\" onchange=\"post('/api/auto-ota','on='+(this.checked?1:0))\"></div>\n"
"    </div>\n"
"\n"
"    <div class=\"card\">\n"
"      <h3>Network</h3>\n"
"      <div class=\"row\"><span class=\"k\">Wi-Fi</span><span class=\"v\" id=\"wifi\">-52 dBm</span></div>\n"
"      <div class=\"row\"><span class=\"k\">IP</span><span class=\"v\" id=\"ip\">192.168.1.112</span></div>\n"
"      <div class=\"row\"><span class=\"k\">MAC</span><span class=\"v\" id=\"mac\">68:09:47:9e:a9:10</span></div>\n"
"    </div>\n"
"\n"
"    <div class=\"card\">\n"
"      <h3>System</h3>\n"
"      <div class=\"row\"><span class=\"k\">Uptime</span><span class=\"v\" id=\"uptime\">0h 3m 41s</span></div>\n"
"      <div class=\"row\"><span class=\"k\">Free heap</span><span class=\"v\" id=\"heap\">168 KB</span></div>\n"
"      <div class=\"row\"><span class=\"k\">Last reset</span><span class=\"v\" id=\"rst\">software</span></div>\n"
"    </div>\n"
"\n"
"    <div class=\"card\">\n"
"      <h3>Activity</h3>\n"
"      <div class=\"row\"><span class=\"k\">Door cycles</span><span class=\"v\" id=\"cycles\">1</span></div>\n"
"      <div class=\"row\"><span class=\"k\">Auto-closes</span><span class=\"v\" id=\"ac\">0</span></div>\n"
"      <div class=\"row\"><span class=\"k\">Left-open trips</span><span class=\"v\" id=\"lo\">0</span></div>\n"
"      <div class=\"row\"><span class=\"k\">Faults</span><span class=\"v\" id=\"flt\">0</span></div>\n"
"    </div>\n"
"\n"
"    <div class=\"card\">\n"
"      <h3>Timers</h3>\n"
"      <div class=\"field\"><label>Auto-close after</label><span class=\"num-in\"><input type=\"number\" id=\"ac_sec\" min=\"0\" max=\"3600\" value=\"600\"> s</span></div>\n"
"      <div class=\"field\"><label>Left-open alert</label><span class=\"num-in\"><input type=\"number\" id=\"lo_sec\" min=\"0\" max=\"3600\" value=\"300\"> s</span></div>\n"
"      <p class=\"hint\" id=\"hint\"></p>\n"
"      <button class=\"save\" onclick=\"post('/api/settings','auto_close_sec='+ac_sec.value+'&left_open_sec='+lo_sec.value)\">Save timers</button>\n"
"    </div>\n"
"  </div>\n"
"</div>\n"
"<script>\n"
"function post(u,b){return fetch(u,{method:'POST',body:b}).then(r=>{if(!r.ok)r.text().then(t=>alert(t));setTimeout(refresh,300)})}\n"
"function fmt(s){s=Math.floor(s);var h=Math.floor(s/3600),m=Math.floor(s%3600/60);return h+'h '+m+'m '+(s%60)+'s'}\n"
"function paint(d){\n"
"  var moving=d.door_state=='opening'||d.door_state=='closing';\n"
"  document.getElementById('door').textContent=d.door_state;\n"
"  var dot=document.getElementById('dot'),gl=document.getElementById('glyph');\n"
"  dot.className='dot'+(d.auto_close_faulted?' bad':moving?' move':d.door_state=='open'?' open':'');\n"
"  gl.style.color=d.auto_close_faulted?'var(--bad)':moving?'var(--amber)':d.door_state=='open'?'var(--open)':'var(--ok)';\n"
"  document.getElementById('openfor').textContent=d.door_open_seconds?('open for '+fmt(d.door_open_seconds)):'—';\n"
"  var lk=document.getElementById('lockout');lk.checked=d.lockout;document.getElementById('lockout_l').textContent=d.lockout?'on':'off';\n"
"  var ao=document.getElementById('autoota');ao.checked=!d.auto_ota_paused;document.getElementById('autoota_l').textContent=d.auto_ota_paused?'paused':'on';\n"
"  document.getElementById('wifi').textContent=d.rssi+' dBm · '+d.rssi_quality;\n"
"  document.getElementById('temp').textContent=d.temperature.toFixed(1);\n"
"  document.getElementById('hum').textContent=d.humidity.toFixed(0);\n"
"  document.getElementById('uptime').textContent=fmt(d.uptime_seconds);\n"
"  document.getElementById('heap').textContent=Math.round(d.free_heap/1024)+' KB';\n"
"  document.getElementById('mac').textContent=d.mac;document.getElementById('ip').textContent=d.ip;\n"
"  document.getElementById('rst').textContent=d.reset_reason;document.getElementById('fw').textContent=d.fw_version;\n"
"  document.getElementById('cycles').textContent=d.cycles;document.getElementById('ac').textContent=d.auto_closes;\n"
"  document.getElementById('lo').textContent=d.left_open_trips;document.getElementById('flt').textContent=d.faults;\n"
"  document.getElementById('fault').className='alert'+(d.auto_close_faulted?' on':'');\n"
"  if(document.activeElement.id!='ac_sec')document.getElementById('ac_sec').value=d.auto_close_sec;\n"
"  if(document.activeElement.id!='lo_sec')document.getElementById('lo_sec').value=d.left_open_sec;\n"
"  document.getElementById('hint').textContent=(d.left_open_sec&&d.auto_close_sec&&d.left_open_sec>=d.auto_close_sec)?'Left-open ≥ auto-close: the alert may never fire.':'';\n"
"}\n"
"function refresh(){fetch('/api/status').then(r=>r.json()).then(paint).catch(function(){})}\n"
"refresh();setInterval(refresh,3000);\n"
"</script>\n"
"</body></html>\n"
"\n";

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
  httpd_resp_set_type(req, "text/html; charset=utf-8");
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

  static char buf[960];
  snprintf(buf, sizeof(buf),
    "{\"door_state\":\"%s\",\"door_open_seconds\":%lld,\"lockout\":%s,"
    "\"rssi\":%d,\"rssi_quality\":\"%s\",\"temperature\":%.1f,\"humidity\":%.1f,"
    "\"uptime_seconds\":%lld,\"fw_version\":\"%s\",\"free_heap\":%u,"
    "\"mac\":\"%02x:%02x:%02x:%02x:%02x:%02x\",\"ip\":\"" IPSTR "\",\"reset_reason\":\"%s\","
    "\"cycles\":%u,\"auto_closes\":%u,\"left_open_trips\":%u,\"faults\":%u,"
    "\"auto_close_faulted\":%s,\"auto_close_sec\":%u,\"left_open_sec\":%u,"
    "\"auto_ota_paused\":%s,"
    "\"ota_poll_sec\":%u,\"door_move_sec\":%u,\"relay_pulse_ms\":%u}",
    door_state_str(door_get_state()), door_open_duration_us() / 1000000LL,
    settings_get_lockout() ? "true" : "false",
    rssi, q, t, h,
    esp_timer_get_time() / 1000000LL, FW_VERSION, (unsigned)esp_get_free_heap_size(),
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], IP2STR(&ip.ip), reset_reason_str(),
    (unsigned)c.cycles, (unsigned)c.auto_closes, (unsigned)c.left_open_trips, (unsigned)c.faults,
    door_auto_close_faulted() ? "true" : "false",
    (unsigned)settings_get_auto_close_sec(), (unsigned)settings_get_left_open_sec(),
    settings_get_auto_ota_paused() ? "true" : "false",
    (unsigned)settings_get_ota_poll_sec(), (unsigned)settings_get_door_move_sec(),
    (unsigned)settings_get_relay_pulse_ms());

  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

static esp_err_t settings_post(httpd_req_t *req) {
  if (require_auth(req) != ESP_OK) return ESP_OK;
  char buf[192];
  if (read_body(req, buf, sizeof(buf)) < 0) { httpd_resp_send_500(req); return ESP_OK; }
  char v[16];
  uint32_t ac = settings_get_auto_close_sec(), lo = settings_get_left_open_sec();
  if (httpd_query_key_value(buf, "auto_close_sec", v, sizeof(v)) == ESP_OK) ac = atoi(v);
  if (httpd_query_key_value(buf, "left_open_sec",  v, sizeof(v)) == ESP_OK) lo = atoi(v);
  settings_set_timeouts(ac, lo);   /* clamps internally */
  if (httpd_query_key_value(buf, "ota_poll_min", v, sizeof(v)) == ESP_OK)
    settings_set_ota_poll_sec((uint32_t)atoi(v) * 60);
  if (httpd_query_key_value(buf, "door_move_sec", v, sizeof(v)) == ESP_OK)
    settings_set_door_move_sec((uint32_t)atoi(v));
  if (httpd_query_key_value(buf, "relay_pulse_ms", v, sizeof(v)) == ESP_OK)
    settings_set_relay_pulse_ms((uint32_t)atoi(v));
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

//
// POST /api/ota-check — trigger an immediate version check (no effect while paused).
//
static esp_err_t ota_check_post(httpd_req_t *req) {
  if (require_auth(req) != ESP_OK) return ESP_OK;
  if (settings_get_auto_ota_paused()) {
    httpd_resp_sendstr(req, "auto-update is paused — resume first");
    return ESP_OK;
  }
  ota_update_check_now();
  httpd_resp_sendstr(req, "checking for updates...");
  return ESP_OK;
}

//
// POST /api/reboot — restart the device (response is sent before the restart).
//
static esp_err_t reboot_post(httpd_req_t *req) {
  if (require_auth(req) != ESP_OK) return ESP_OK;
  httpd_resp_sendstr(req, "rebooting...");
  ESP_LOGW(TAG, "Reboot requested via web");
  vTaskDelay(pdMS_TO_TICKS(500));
  esp_restart();
  return ESP_OK;   /* not reached */
}

// ---- lifecycle -----------------------------------------------------------

void web_server_start(void) {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = WEB_SERVER_PORT;        /* 80 is taken by the HAP server */
  config.ctrl_port   = WEB_SERVER_CTRL_PORT;   /* HAP uses the default 32768 */
  config.max_uri_handlers = 12;
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
    { .uri = "/api/ota-check",  .method = HTTP_POST, .handler = ota_check_post },
    { .uri = "/api/reboot",     .method = HTTP_POST, .handler = reboot_post },
  };
  for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++)
    httpd_register_uri_handler(server, &uris[i]);
  ESP_LOGI(TAG, "Web server started on port %d", WEB_SERVER_PORT);
}
