#pragma once

#include <stdint.h>

#define WEBUI_AP_CHANNEL      1

#define WEBUI_SSID_PREFIX    "EduBot-"

#define WEBUI_DEFAULT_PASSWORD "edubot2026"

#define WEBUI_PASSWORD_LEN   8

#define WEBUI_MAX_CLIENTS    4

#define WEBUI_IDLE_TIMEOUT_MS 600000

#define WEBUI_AP_IP          192, 168, 4, 1
#define WEBUI_AP_GATEWAY     192, 168, 4, 1
#define WEBUI_AP_SUBNET      255, 255, 255, 0

#define WEBUI_WS_PATH        "/ws"
#define WEBUI_PARTITION      "spiffs"
#define WEBUI_MOUNT_POINT    "/littlefs"

#define WEBUI_STATUS_QUEUE_LEN   16
#define WEBUI_STATUS_LINE_MAX    240

#define WEBUI_NVS_NAMESPACE "edubot"
#define WEBUI_NVS_PW_KEY    "apPw"
#define WEBUI_NVS_SSID_KEY  "apSsid"
#define WEBUI_NVS_AUTO_KEY  "apAuto"