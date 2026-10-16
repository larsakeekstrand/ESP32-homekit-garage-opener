// devkit_dims.scad — the ESP32 DevKitC V4 clone on its sockets. MEASURE YOUR DEVKIT and correct these.
// Offsets are relative to socket pin 19 (5V, USB end) in scad coords (X right, Y up); heights above the carrier top.
socket_h = 8.5;          // female header height
devkit_t = 1.6;          // devkit PCB thickness
stack_h = socket_h + devkit_t + 3.5;   // top of the tallest devkit part (module shield / buttons)
usb_dy = -12.7;          // USB connector centre, Y offset from pin 19 (midway between the rows)
usb_w = 8.0;             // receptacle width (micro-USB 7.5, USB-C 8.9 — set to yours)
usb_hgt = 3.2;           // receptacle height
usb_z = socket_h + devkit_t + usb_hgt / 2;   // receptacle centre height
usb_plug_w = 12;         // cut-out for the cable overmold
usb_plug_h = 8;
btn_boot = [2.0, -20.5]; // BOOT switch centre, offset from pin 19 (X into the board, Y)
btn_en = [2.0, -4.9];    // EN switch centre
btn_top_z = socket_h + devkit_t + 2.0;   // top of the tactile switch plungers
