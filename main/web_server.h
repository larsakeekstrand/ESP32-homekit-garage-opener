//
// web_server.h — authenticated local status/config web page (HTTP, LAN-only).
//
#pragma once

/* Start the HTTP server on port 80. Call after Wi-Fi is connected. */
void web_server_start(void);
