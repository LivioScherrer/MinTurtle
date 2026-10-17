#include "WebUi.h"
#include "WebUiConfig.h"
#include "BLEManager.h"
#include "EspNow.h"

#include <WiFi.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <esp_wifi.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <lwip/sockets.h>
#include <lwip/inet.h>
#include <lwip/ip_addr.h>

#include <ESPAsyncWebServer.h>
#include <AsyncWebSocket.h>

extern void onWebClientConnected();
extern void onWebClientDisconnected();

static AsyncWebServer  s_server(80);
static AsyncWebSocket  s_ws(WEBUI_WS_PATH);

static bool            s_apUp = false;
static bool            s_serverStarted = false;
static String          s_ssid;
static String          s_apIp;
static uint32_t        s_apStartedAt = 0;
static volatile bool   s_dnsRunning = false;

typedef char StatusLine[WEBUI_STATUS_LINE_MAX];
static StaticQueue_t   s_statusQStore;
static uint8_t          s_statusQMem[WEBUI_STATUS_QUEUE_LEN * sizeof(StatusLine)];
static QueueHandle_t    s_statusQ = nullptr;

#define RX_SLOTS 8
static char   s_rxBuf[RX_SLOTS][sizeof(BleEvent::cmd)];
static size_t s_rxLen[RX_SLOTS] = {0};
static uint32_t s_rxDropped = 0;


static String makePassword() {
  static const char kAlphabet[] = "23456789ABCDEFGHJKLMNPQRSTUVWXYZ";
  const uint8_t n = (uint8_t)(sizeof(kAlphabet) - 1);
  String pw;
  pw.reserve(WEBUI_PASSWORD_LEN);
  for (int i = 0; i < WEBUI_PASSWORD_LEN; i++) {
    pw += kAlphabet[esp_random() % n];
  }
  return pw;
}

String webUiEnsurePassword() {
  Preferences nvs;
  if (nvs.begin(WEBUI_NVS_NAMESPACE, false)) {
    String stored = nvs.getString(WEBUI_NVS_PW_KEY, "");
    if (stored.length() >= 8) {
      nvs.end();
      return stored;
    }
    String fresh = makePassword();
    nvs.putString(WEBUI_NVS_PW_KEY, fresh);
    nvs.end();
    return fresh;
  }
  String fallback = makePassword();
  Serial.println("[WEB] WARNUNG NVS nicht lesbar - Passwort wechselt bei jedem Start");
  return fallback;
}

String webUiPassword() {
  Preferences nvs;
  String out;
  if (nvs.begin(WEBUI_NVS_NAMESPACE, true)) {
    out = nvs.getString(WEBUI_NVS_PW_KEY, "");
    nvs.end();
  }
  return out;
}

String webUiResetPassword() {
  String fresh = makePassword();
  Preferences nvs;
  if (nvs.begin(WEBUI_NVS_NAMESPACE, false)) {
    nvs.putString(WEBUI_NVS_PW_KEY, fresh);
    nvs.end();
  }
  return fresh;
}

bool webUiAutoEnabled() {
  Preferences nvs;
  bool enabled = false;
  if (nvs.begin(WEBUI_NVS_NAMESPACE, true)) {
    enabled = nvs.getBool(WEBUI_NVS_AUTO_KEY, false);
    nvs.end();
  }
  return enabled;
}

bool webUiSetAutoEnabled(bool enabled) {
  Preferences nvs;
  if (!nvs.begin(WEBUI_NVS_NAMESPACE, false)) {
    Serial.println("[WEB] Autostart nicht gespeichert: NVS nicht schreibbar");
    return false;
  }
  const bool ok = (nvs.putBool(WEBUI_NVS_AUTO_KEY, enabled) != 0);
  nvs.end();
  if (!ok) Serial.println("[WEB] Autostart nicht gespeichert: NVS-Schreibfehler");
  return ok;
}

String webUiSsid() { return s_ssid; }

static void makeSsidFromMac(String& out) {
  uint8_t mac[6] = {0};
  esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
  char buf[32];
  snprintf(buf, sizeof(buf), "%s%02X%02X", WEBUI_SSID_PREFIX, mac[4], mac[5]);
  out = String(buf);
}


static void dnsTask(void*   ) {
  int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock < 0) {
    Serial.println("[DNS] socket() fehlgeschlagen - Captive-Portal-Erkennung faellt aus");
    vTaskDelete(NULL);
    return;
  }

  struct sockaddr_in local = {};
  local.sin_family = AF_INET;
  local.sin_port = htons(53);
  local.sin_addr.s_addr = IPADDR_ANY;
  if (bind(sock, (struct sockaddr*)&local, sizeof(local)) != 0) {
    Serial.println("[DNS] bind(53) fehlgeschlagen - iOS wuerde das WLAN verlassen");
    close(sock);
    vTaskDelete(NULL);
    return;
  }

  Serial.println("[DNS] DNS-Server auf Port 53 laeuft (alle Namen -> Roboter)");

  uint8_t req[512];
  uint8_t res[512];
  struct sockaddr_in from = {};

  for (;;) {
    if (!s_dnsRunning) break;

    socklen_t fromLen = sizeof(from);
    int n = recvfrom(sock, req, sizeof(req), 0, (struct sockaddr*)&from, &fromLen);
    if (n < 12) continue;

    int p = 12;
    while (p < n && req[p] != 0) {
      p += 1 + req[p];
    }
    p += 1;
    int qEnd = p + 4;
    if (qEnd > n) continue;

    bool isA = (req[p] == 0x00 && req[p + 1] == 0x01);
    bool isIn = (req[p + 2] == 0x00 && req[p + 3] == 0x01);

    memcpy(res, req, (size_t)n);

    res[2] = 0x81;
    res[3] = 0x80;
    res[4] = 0x00; res[5] = 0x01;
    res[6] = 0x00; res[7] = isA ? 0x01 : 0x00;
    res[8] = 0x00; res[9] = 0x00;
    res[10] = 0x00; res[11] = 0x00;

    size_t outLen = (size_t)qEnd;
    if (isA && isIn) {
      res[outLen++] = 0xC0; res[outLen++] = 0x0C;
      res[outLen++] = 0x00; res[outLen++] = 0x01;
      res[outLen++] = 0x00; res[outLen++] = 0x01;
      res[outLen++] = 0x00; res[outLen++] = 0x00;
      res[outLen++] = 0x00; res[outLen++] = 0x3C;
      res[outLen++] = 0x00; res[outLen++] = 0x04;
      res[outLen++] = 192; res[outLen++] = 168;
      res[outLen++] = 4;   res[outLen++] = 1;
    }

    sendto(sock, res, outLen, 0, (struct sockaddr*)&from, fromLen);
  }
}


static void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT: {
      BleEvent evt = { BLE_EVT_WEB_CONNECTED, "" };
      if (bleEventQueue) xQueueSend(bleEventQueue, &evt, 0);
      Serial.printf("[WEB] Client %u verbunden (%u online)\n",
                    (unsigned)client->id(), (unsigned)server->count());
      break;
    }
    case WS_EVT_DISCONNECT: {
      BleEvent evt = { BLE_EVT_WEB_DISCONNECTED, "" };
      if (bleEventQueue) xQueueSend(bleEventQueue, &evt, 0);
      Serial.printf("[WEB] Client %u getrennt\n", (unsigned)client->id());
      break;
    }
    case WS_EVT_DATA: {
      AwsFrameInfo* info = (AwsFrameInfo*)arg;
      if (!info || info->message_opcode != WS_TEXT) break;

      const uint16_t slot = (uint16_t)(client->id() % RX_SLOTS);
      if (info->index == 0) s_rxLen[slot] = 0;

      if (s_rxLen[slot] + len >= sizeof(BleEvent::cmd)) {
        s_rxLen[slot] = 0;
        s_rxDropped++;
        return;
      }
      memcpy(s_rxBuf[slot] + s_rxLen[slot], data, len);
      s_rxLen[slot] += len;
      if (!info->final) return;

      s_rxBuf[slot][s_rxLen[slot]] = '\0';
      s_rxLen[slot] = 0;

      if (s_rxBuf[slot][0] == '\0') return;

      BleEvent evt = { BLE_EVT_COMMAND, "" };
      memcpy(evt.cmd, s_rxBuf[slot], strlen(s_rxBuf[slot]) + 1);
      if (bleEventQueue) {
        if (xQueueSend(bleEventQueue, &evt, 0) != pdTRUE) {
          Serial.println("[WEB] Befehl-Queue voll - Befehl verworfen");
        }
      }
      break;
    }
    default:
      break;
  }
}


static void answerOk(AsyncWebServerRequest* req, const char* body) {
  req->send(200, "text/plain", body);
}

static void server_on(const char* path, ArRequestHandlerFunction fn) {
  s_server.on(path, HTTP_GET, fn);
}

static void registerCaptiveHandlers() {
  server_on("/hotspot-detect.html", [](AsyncWebServerRequest* r) { answerOk(r, "Success\n"); });
  server_on("/library/test/success.html", [](AsyncWebServerRequest* r) { answerOk(r, "Success\n"); });
  server_on("/generate_204", [](AsyncWebServerRequest* r) { r->send(204); });
  server_on("/gen_204", [](AsyncWebServerRequest* r) { r->send(204); });
  server_on("/mobile/status.php", [](AsyncWebServerRequest* r) { r->send(204); });
  server_on("/ncsi.txt", [](AsyncWebServerRequest* r) { answerOk(r, "Microsoft NCSI\n"); });
  server_on("/connecttest.txt", [](AsyncWebServerRequest* r) { answerOk(r, "Microsoft Connect Test\n"); });
  server_on("/success.txt", [](AsyncWebServerRequest* r) { answerOk(r, "success\n"); });
}

void webUiFlush() {
  if (!s_apUp) return;
  const size_t clients = s_ws.count();

  static uint32_t lastEmptyTick = 0;
  if (clients == 0) {
    if (lastEmptyTick == 0) lastEmptyTick = millis();
    if (millis() - lastEmptyTick > WEBUI_IDLE_TIMEOUT_MS) {
      Serial.println("[WEB] Leerlaufzeit ueberschritten - WLAN wird abgeschaltet");
      webUiStop();
      return;
    }
  } else {
    lastEmptyTick = 0;
  }

  if (clients == 0) {
    if (s_statusQ) xQueueReset(s_statusQ);
    return;
  }

  StatusLine line;
  for (int i = 0; i < 8 && s_statusQ && xQueueReceive(s_statusQ, line, 0) == pdTRUE; ) {
    s_ws.textAll(line);
  }
  static uint32_t lastCleanup = 0;
  if (millis() - lastCleanup > 1000) {
    lastCleanup = millis();
    s_ws.cleanupClients(WEBUI_MAX_CLIENTS);
  }
}

void webUiBroadcast(const char* line) {
  if (!s_statusQ || !line) return;
  if (s_ws.count() == 0) return;

  StatusLine item = {};
  size_t n = strnlen(line, sizeof(item) - 1);
  memcpy(item, line, n);

  if (xQueueSend(s_statusQ, item, 0) != pdTRUE) {
    StatusLine drop;
    if (xQueueReceive(s_statusQ, drop, 0) == pdTRUE) {
      xQueueSend(s_statusQ, item, 0);
    }
  }
}

bool webUiClientConnected() { return s_ws.count() > 0; }
uint8_t webUiClientCount() { return (uint8_t)s_ws.count(); }


void webUiSetup() {
  Serial.println("[WEB] === Web-UI: Vorbereitung, WLAN bleibt aus ===");

  if (!LittleFS.begin(true, WEBUI_MOUNT_POINT, 10, WEBUI_PARTITION)) {
    Serial.println("[WEB] LittleFS nicht mountbar - es wird eine leere Seite geliefert");
    Serial.println("[WEB] Loesung: tools\\build_webui.ps1  und  pio run -t uploadfs");
  } else {
    Serial.printf("[WEB] LittleFS gemountet: %u KB benutzt von %u KB\n",
                  (unsigned)(LittleFS.usedBytes() / 1024),
                  (unsigned)(LittleFS.totalBytes() / 1024));
  }

  s_statusQ = xQueueCreateStatic(WEBUI_STATUS_QUEUE_LEN, sizeof(StatusLine),
                                 s_statusQMem, &s_statusQStore);
  if (!s_statusQ) {
    Serial.println("[WEB] Statuspuffer konnte nicht angelegt werden - kein Web-Status");
  }

  String password = webUiEnsurePassword();
  makeSsidFromMac(s_ssid);

  Serial.println("");
  Serial.println("  WLAN ist AUS - Bluetooth laeuft wie gewohnt.");
  Serial.println("  Fuer ein iPad/iPhone: rot-blau-gruen vor den Farbsensor");
  Serial.println("  halten, dann erscheint das WLAN.");
  Serial.printf("  Netz wird es sein: %s\n", s_ssid.c_str());
  Serial.println("");
  Serial.println("=====================================================");
}

void webUiStart() {
  if (s_apUp) return;

  if (anyClientConnected()) {
    Serial.println("[WEB] Start abgelehnt: ein Client ist verbunden - WLAN bleibt aus");
    return;
  }

  String password = webUiPassword();
  if (password.length() < 8) password = webUiEnsurePassword();

  IPAddress ip(WEBUI_AP_IP), gw(WEBUI_AP_GATEWAY), mask(WEBUI_AP_SUBNET);
  if (!WiFi.softAPConfig(ip, gw, mask)) {
    Serial.println("[WEB] WARNUNG: DHCP-Konfiguration nicht angenommen");
  }

  bool ok = WiFi.softAP(s_ssid.c_str(), password.c_str(), WEBUI_AP_CHANNEL,
                        false, WEBUI_MAX_CLIENTS);
  if (!ok) {
    Serial.println("[WEB] FEHLER: Access Point liess sich nicht starten");
    return;
  }
  s_apUp = true;
  s_apStartedAt = millis();
  s_apIp = WiFi.softAPIP().toString();


  if (!s_serverStarted) {
    s_ws.onEvent(onWsEvent);
    s_server.addHandler(&s_ws);
    registerCaptiveHandlers();

    s_server.serveStatic("/fonts/", LittleFS, "/fonts/")
        .setCacheControl("max-age=604800, immutable");

    s_server.serveStatic("/", LittleFS, "/")
        .setDefaultFile("index.html")
        .setCacheControl("no-cache, no-store, must-revalidate");

    s_server.onNotFound([](AsyncWebServerRequest* r) {
      r->redirect("http://192.168.4.1/");
    });
    s_server.begin();
    s_serverStarted = true;
  }

  if (!s_dnsRunning) {
    s_dnsRunning = true;
    if (xTaskCreatePinnedToCore(dnsTask, "dns", 4096, nullptr, 3, nullptr, 0) != pdPASS) {
      s_dnsRunning = false;
      Serial.println("[WEB] DNS-Task liess sich nicht starten");
    }
  }

  espNowEnsureChannel();

  Serial.println("");
  Serial.println("=====================================================");
  Serial.println("  WLAN EINGESCHALTET - auf dem iPad so beitreten:");
  Serial.println("=====================================================");
  Serial.printf("   1. Netz      : %s\n", s_ssid.c_str());
  Serial.printf("   2. Passwort  : %s\n", password.c_str());
  Serial.printf("   3. Browser   : http://%s\n", s_apIp.c_str());
  Serial.println("=====================================================");
  Serial.printf("[WEB] Ohne Geraet geht es nach %u min wieder aus.\n",
                (unsigned)(WEBUI_IDLE_TIMEOUT_MS / 60000));
  Serial.printf("WIFI:T:WPA;S:%s;P:%s;;\n", s_ssid.c_str(), password.c_str());
}

void webUiStop() {
  if (!s_apUp) return;
  Serial.println("[WEB] WLAN wird abgeschaltet - Bluetooth laeuft weiter");

  s_dnsRunning = false;
  s_server.end();

  WiFi.mode(WIFI_STA);
  espNowEnsureChannel();

  s_apUp = false;
  s_apStartedAt = 0;
  Serial.println("[WEB] WLAN aus. Bluetooth laeuft wie gewohnt.");
}

bool webUiIsUp() { return s_apUp; }
uint32_t webUiUpFor() { return s_apUp ? (millis() - s_apStartedAt) : 0; }
void webUiPrintDiag() {
  Serial.println("[WEB] ---- Diagnose ----");
  if (s_apUp) {
    const uint32_t upMin = webUiUpFor() / 60000;
    Serial.printf("[WEB] WLAN: AN seit %u min  (SSID %s, IP %s, Kanal %u)\n",
                  (unsigned)upMin, s_ssid.c_str(), s_apIp.c_str(),
                  (unsigned)WiFi.channel());
  } else {
    Serial.printf("[WEB] WLAN: AUS  (bereit per Farbfolge rot-blau-gruen; SSID waere %s)\n",
                  s_ssid.c_str());
  }
  Serial.printf("[WEB] Clients: %u (max %u)  Bluetooth: %s\n",
                (unsigned)s_ws.count(), (unsigned)WEBUI_MAX_CLIENTS,
                deviceConnected ? "verbunden" : "frei");
  Serial.printf("[WEB] Heap frei: %u KB, groesster Block: %u KB\n",
                (unsigned)(ESP.getFreeHeap() / 1024),
                (unsigned)(heap_caps_get_largest_free_block(0) / 1024));
  if (s_statusQ) {
    Serial.printf("[WEB] Statuspuffer: %u von %u belegt\n",
                  (unsigned)uxQueueMessagesWaiting(s_statusQ),
                  (unsigned)WEBUI_STATUS_QUEUE_LEN);
  }
  Serial.printf("[WEB] Empfangene Fragmente verworfen: %u\n", (unsigned)s_rxDropped);
  Serial.printf("[WEB] WLAN-Modus: %s\n",
                (WiFi.getMode() == WIFI_AP_STA) ? "AP_STA" :
                (WiFi.getMode() == WIFI_AP) ? "AP" : "STA");
  Serial.println("[WEB] -------------------");
}