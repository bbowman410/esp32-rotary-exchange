#pragma once

// Edit these before uploading. Every board needs its own NODE_NUMBER.

const char* WIFI_SSID = "your-ssid";
const char* WIFI_PASSWORD = "your-password";

// Three digits, unique on the network. The softphone defaults to 111,
// so leave 111 free unless you change the softphone too.
constexpr uint16_t NODE_NUMBER = 456;
