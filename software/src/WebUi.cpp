// EduBot V2 iOS - Web-UI Implementierung
//
// Drei Bausteine, bewusst getrennt gehalten:
//   1. Access Point  - spannt das WLAN auf
//   2. Dateiausgabe - liefert die Oberflaeche aus dem LittleFS
//   3. Transport     - WebSocket als zweiter Weg fuer Befehle und Status
//
// Zum Transport:
// Der WebSocket-Handler laeuft im Task des Webservers, NICHT im loop().
// Deshalb wird hier genau das getan, was CommandCallbacks::onWrite() fuer
// Bluetooth auch tut: die Zeile wird in bleEventQueue gelegt und im Hauptloop
// von bleProcessEvents() zu handleBleCommand() weitergereicht. handleCommand()
// wird niemals aus einem fremden Task aufgerufen - die Spiellogik laeuft
// sonst nebenbei und rechnet Motoren und Spielphasen an zwei Stellen.
//
// Umgekehrt sendet sendStatus() die Statuszeile an beide Transporte. Auch das
// darf nicht direkt geschehen: ws.textAll() ist nicht threadsicher gegen
// gleichzeitiges Senden und wuerde Speicher auf dem Heap reservieren, waehrend
// ein Motortask laeuft. Deshalb ein statischer Ringpuffer (s_statusQ) und das
// Leeren in webUiFlush() aus dem loop().
#include "WebUi.h"
#include "WebUiConfig.h"
#include "BLEManager.h"
#include "EspNow.h"       // espNowEnsureChannel()

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

// Handler in Edu_V2.cpp. Die Web-Clients bekommen exakt dieselbe Behandlung
// wie ein Bluetooth-Geraet - das ist der ganze Trick hinter "eine Oberflaeche
// fuer beide Wege" (Plan Abschnitt 6).
extern void onWebClientConnected();
extern void onWebClientDisconnected();

// ---- Zustand -------------------------------------------------------------
static AsyncWebServer  s_server(80);
static AsyncWebSocket  s_ws(WEBUI_WS_PATH);

static bool            s_apUp = false;
static bool            s_handlersRegistered = false;
static bool            s_serverStarted = false;
static String          s_ssid;
static String          s_apIp;
static uint32_t        s_apStartedAt = 0;
static volatile bool   s_dnsRunning = false;
static volatile bool   s_dnsAlive = false;

// Statuszeilen zwischen den Tasks und dem loop().
// Statisch alloziert: liegt im .bss, nimmt dem Heap nichts weg. 16 x 240 Byte
// sind knapp 4 KB - bei einem Board mit 320 KB RAM und schon 20 % Belegung
// durch die Spiellogik ist das der Unterschied zwischen "Passt noch" und
// "Motor ruckelt".
typedef char StatusLine[WEBUI_STATUS_LINE_MAX];
static StaticQueue_t   s_statusQStore;
static uint8_t          s_statusQMem[WEBUI_STATUS_QUEUE_LEN * sizeof(StatusLine)];
static QueueHandle_t    s_statusQ = nullptr;

// Gemaess AsyncWebSocket ist die Client-ID ein fortlaufender Zaehler ab 0,
// der nach DEFAULT_MAX_WS_CLIENTS (4-8) wieder auf 0 springt. Fuenf Slots
// reichen fuer WEBUI_MAX_CLIENTS und lassen Luft.
#define RX_SLOTS 8
static char   s_rxBuf[RX_SLOTS][sizeof(BleEvent::cmd)];
static size_t s_rxLen[RX_SLOTS] = {0};
static uint32_t s_rxDropped = 0;    // nur fuer die Diagnose

// =========================================================================
// 1. Access Point
// =========================================================================

// Passwort erzeugen. Ohne 0/O/1/l/I: die unterscheiden sich auf einem
// Aufkleber gedruckt zu schlecht, und ein Kind gibt "O" statt "0" ein.
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
  return String("");
}

String webUiPassword() {
  return String("");
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

// SSID aus der MAC. Zwei Bytes am Ende = vier Hex-Zellen = genau der Aufkleber.
static void makeSsidFromMac(String& out) {
  uint8_t mac[6] = {0};
  esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
  char buf[32];
  snprintf(buf, sizeof(buf), "%s%02X%02X", WEBUI_SSID_PREFIX, mac[4], mac[5]);
  out = String(buf);
}

// =========================================================================
// 2. DNS-Server  - der Teil, ohne den iOS aus dem WLAN geht
// =========================================================================
//
// Was passiert, wenn ein WLAN keinen Internetzugang hat:
// Das Betriebssystem prueft das. Es schickt eine HTTP-Anfrage an eine bekannte
// Adresse (bei iOS captive.apple.com, bei Android gstatic.com). Kommt die
// erwartete Antwort, gilt das WLAN als "hat Internet". Kommt keine oder eine
// falsche, gilt es als "kein Internet" - und iOS faellt dann auf die
// Mobilfunkdaten zurueck. Der Roboter ist dann nicht mehr erreichbar, obwohl
// das WLAN perfekt funktioniert.
//
// Und hier liegt der Haken: captive.apple.com muss erst aufgeloest werden.
// Der Access Point hat keinen DNS-Server, also sendet das Geraet die Anfrage an
// niemanden - die Pruefung scheitert schon an der Namensaufloesung, bevor sie
// den Roboter erreicht.
//
// Dieser winzige DNS-Server beantwortet JEDE Namensfrage mit 192.168.4.1.
// Damit laeuft die Betriebssystem-Pruefung beim Roboter auf, und die
// HTTP-Handler in registerHandlers() beantworten sie mit Erfolg. Ergebnis:
// iOS zeigt das WLAN als verbunden an und bleibt drin. Android genauso.
//
// Das ist die einzige Stelle im ganzen Projekt, an der "Captive Portal" nicht
// missverstaendlich ist: Es wird hier kein Portal gezeigt, sondern es wird
// ausdruecklich behauptet, alles sei in Ordnung - weil es das auch ist.

// DNS-Paket: 12 Byte Kopf, dann die Frage, dann die Antwort.
static void dnsTask(void* /*arg*/) {
  int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock < 0) {
    Serial.println("[DNS] socket() fehlgeschlagen - Captive-Portal-Erkennung faellt aus");
    s_dnsRunning = false;
    s_dnsAlive = false;
    vTaskDelete(NULL);
    return;
  }

  // Kurzes Empfangs-Timeout, damit der Task nach webUiStop() (s_dnsRunning=false)
  // auch ohne weiteren DNS-Verkehr zeitnah aufwacht, Port 53 schliesst und sich
  // sauber per vTaskDelete(NULL) beendet.
  struct timeval tv = {};
  tv.tv_sec = 0;
  tv.tv_usec = 250000;   // 250 ms
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  struct sockaddr_in local = {};
  local.sin_family = AF_INET;
  local.sin_port = htons(53);
  local.sin_addr.s_addr = IPADDR_ANY;
  if (bind(sock, (struct sockaddr*)&local, sizeof(local)) != 0) {
    Serial.println("[DNS] bind(53) fehlgeschlagen - iOS wuerde das WLAN verlassen");
    close(sock);
    s_dnsRunning = false;
    s_dnsAlive = false;
    vTaskDelete(NULL);
    return;
  }

  Serial.println("[DNS] DNS-Server auf Port 53 laeuft (alle Namen -> Roboter)");

  uint8_t req[512];
  uint8_t res[512];
  struct sockaddr_in from = {};

  for (;;) {
    // Abschalten? Der Task beendet sich selbst und gibt den Port 53 frei.
    // Ohne diese Pruefung hinge der DNS-Server weiter auf einem Port, den
    // es gar nicht mehr geben darf - und ein zweites webUiStart() kaeme
    // nie wieder durch bind().
    if (!s_dnsRunning) break;

    socklen_t fromLen = sizeof(from);
    int n = recvfrom(sock, req, sizeof(req), 0, (struct sockaddr*)&from, &fromLen);
    if (!s_dnsRunning) break;
    if (n < 12) continue;                    // Timeout oder zu kurz, ignorieren

    // Ende der Frage finden: QNAME (Label mit Laengenbyte, 0-terminiert,
    // oder 2-Byte-Kompressionszeiger 0xC0) danach noch QTYPE (2 Byte) und QCLASS (2 Byte).
    int p = 12;
    while (p < n && req[p] != 0) {
      if ((req[p] & 0xC0) == 0xC0) {
        p += 1;                             // +1 hier und +1 nach der Schleife = 2 Bytes Pointer
        break;
      }
      p += 1 + req[p];                      // Laengenbyte + Inhalt
    }
    p += 1;                                 // der 0-Terminator (bzw. 2. Pointer-Byte)
    int qEnd = p + 4;                       // + QTYPE + QCLASS
    // Platz fuer die 16-Byte-A-Record-Antwort im 512-Byte-Puffer sicherstellen.
    if (qEnd > n || qEnd + 16 > (int)sizeof(res)) continue;

    bool isA = (req[p] == 0x00 && req[p + 1] == 0x01);
    bool isIn = (req[p + 2] == 0x00 && req[p + 3] == 0x01);

    memcpy(res, req, (size_t)n);

    // Antwortkopf: QR=1, RD mit zurueckgeben, RA=1, Fehlercode 0.
    res[2] = 0x81;
    res[3] = 0x80;
    res[4] = 0x00; res[5] = 0x01;            // QDCOUNT = 1
    res[6] = 0x00; res[7] = isA ? 0x01 : 0x00;   // ANCOUNT
    res[8] = 0x00; res[9] = 0x00;
    res[10] = 0x00; res[11] = 0x00;

    size_t outLen = (size_t)qEnd;
    if (isA && isIn) {
      res[outLen++] = 0xC0; res[outLen++] = 0x0C;   // NAME = Zeiger auf Frage
      res[outLen++] = 0x00; res[outLen++] = 0x01;   // TYPE = A
      res[outLen++] = 0x00; res[outLen++] = 0x01;   // CLASS = IN
      res[outLen++] = 0x00; res[outLen++] = 0x00;   // TTL = 60 s
      res[outLen++] = 0x00; res[outLen++] = 0x3C;
      res[outLen++] = 0x00; res[outLen++] = 0x04;   // RDLENGTH = 4
      res[outLen++] = 192; res[outLen++] = 168;
      res[outLen++] = 4;   res[outLen++] = 1;
    }
    // Bei AAAA (und allem anderen) bewusst OHNE Antwort. Das Betriebssystem
    // fragt sehr oft nach IPv6; eine leere Antwort bedeutet "gibt es nicht",
    // und dann fragt es nach A - nicht nach IPv6. Ein erfundener AAAA-Eintrag
    // wuerde das Geraet dagegen auf ein nicht erreichbares IPv6 schicken.

    sendto(sock, res, outLen, 0, (struct sockaddr*)&from, fromLen);
  }

  close(sock);
  s_dnsAlive = false;
  vTaskDelete(NULL);
}

// =========================================================================
// 3. WebSocket
// =========================================================================

static void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT: {
      // Der Verbindungsaufbau ist eine Aktion fuer die Spiellogik (Boden
      // einlernen, Bereitschaft melden). Deshalb geht sie als Queue-Eintrag
      // an den Hauptloop - genau wie der Bluetooth-Verbindungsaufbau.
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

      // WebSocket-Nachrichten duerfen ueber mehrere Frames kommen. Ein Befehl
      // wird deshalb gesammelt, bis das Finale-Bit gesetzt ist. Ohne das
      // wuerde ein langer Befehl (PROG:SET:...) bei unlucky Fragmentierung
      // als zwei halbe Befehle ankommen und nichts bewirken.
      const uint16_t slot = (uint16_t)(client->id() % RX_SLOTS);
      if (info->index == 0) s_rxLen[slot] = 0;

      if (s_rxLen[slot] + len >= sizeof(BleEvent::cmd)) {
        s_rxLen[slot] = 0;
        s_rxDropped++;
        return;                             // Nachricht samt Rest verwerfen
      }
      memcpy(s_rxBuf[slot] + s_rxLen[slot], data, len);
      s_rxLen[slot] += len;
      if (!info->final) return;

      s_rxBuf[slot][s_rxLen[slot]] = '\0';
      s_rxLen[slot] = 0;

      if (s_rxBuf[slot][0] == '\0') return;  // leere Nachricht

      // Ab hier derselbe Weg wie bei Bluetooth. Kein Aufruf von handleCommand.
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

// =========================================================================
// 4. HTTP
// =========================================================================

// Erwartungs-Kopfdaten, die nacheinander geprueft werden, ob das WLAN
// Internet hat. Wir beantworten ALLE mit Erfolg - siehe Kommentar beim
// DNS-Server, warum das richtig ist und nicht betruegerisch.
static void answerOk(AsyncWebServerRequest* req, const char* body) {
  req->send(200, "text/plain", body);
}

// Kurzform, damit die Liste unten nicht in Zeilenumbruch-Geradeaus liegt.
static void server_on(const char* path, ArRequestHandlerFunction fn) {
  s_server.on(path, HTTP_GET, fn);
}

static void registerCaptiveHandlers() {
  // iOS / macOS: captive.apple.com
  server_on("/hotspot-detect.html", [](AsyncWebServerRequest* r) { answerOk(r, "Success\n"); });
  server_on("/library/test/success.html", [](AsyncWebServerRequest* r) { answerOk(r, "Success\n"); });
  // Android / Chrome
  server_on("/generate_204", [](AsyncWebServerRequest* r) { r->send(204); });
  server_on("/gen_204", [](AsyncWebServerRequest* r) { r->send(204); });
  server_on("/mobile/status.php", [](AsyncWebServerRequest* r) { r->send(204); });
  // Windows
  server_on("/ncsi.txt", [](AsyncWebServerRequest* r) { answerOk(r, "Microsoft NCSI\n"); });
  server_on("/connecttest.txt", [](AsyncWebServerRequest* r) { answerOk(r, "Microsoft Connect Test\n"); });
  // Firefox
  server_on("/success.txt", [](AsyncWebServerRequest* r) { answerOk(r, "success\n"); });
}

void webUiFlush() {
  if (!s_apUp) return;
  const size_t clients = s_ws.count();

  // ---- Selbst abschalten ------------------------------------------------
  // Nur wenn wirklich niemand verbunden ist. Sonst wuerde das WLAN mitten
  // in einer Fahrt wegbremsen.
  //
  // Die Zeit wird erst ab dem LETZTEN Weggang eines Geraets gezaehlt und
  // faengt wieder bei null an, sobald sich eines meldet. Ein Roboter, der
  // den ganzen Vormittag benutzt wird, schaltet also nie mittendrin ab.
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
    // Niemand hoert zu. Den Puffer leeren statt stehen lassen: sonst stehen
    // beim naechsten Verbinden Sekunden alte Meldungen im Browser.
    if (s_statusQ) xQueueReset(s_statusQ);
    return;
  }

  StatusLine line;
  // Hoechstens 8 Zeilen pro Durchlauf. Die loop() laeuft mit ~1 ms Takt, das
  // reicht fuer 8000 Zeilen/s und haelt die Sendezeit kurz.
  for (int i = 0; i < 8 && s_statusQ && xQueueReceive(s_statusQ, line, 0) == pdTRUE; i++) {
    s_ws.textAll(line);
  }
  // Ab und an die Liste der Clients aufraeumen (geschlossene Sockets, die
  // AsyncTCP nicht selbst erkannt hat). Jede Sekunde ist reichlich.
  static uint32_t lastCleanup = 0;
  if (millis() - lastCleanup > 1000) {
    lastCleanup = millis();
    s_ws.cleanupClients(WEBUI_MAX_CLIENTS);
  }
}

void webUiBroadcast(const char* line) {
  if (!s_statusQ || !line) return;
  if (s_ws.count() == 0) return;   // vor dem ersten Client nichts sammeln

  StatusLine item = {};
  size_t n = strnlen(line, sizeof(item) - 1);
  memcpy(item, line, n);

  if (xQueueSend(s_statusQ, item, 0) != pdTRUE) {
    // Puffer voll. Nicht die NEUE Zeile behalten, sondern die aelteste opfern:
    // ein KIND, das gerade den Pfeil drueckt, braucht den aktuellen Zustand -
    // eine 200 ms alte Diagnosezeile ist wertlos, der naechste Tastendruck
    // nicht.
    StatusLine drop;
    if (xQueueReceive(s_statusQ, drop, 0) == pdTRUE) {
      xQueueSend(s_statusQ, item, 0);
    }
  }
}

bool webUiClientConnected() { return s_ws.count() > 0; }
uint8_t webUiClientCount() { return (uint8_t)s_ws.count(); }

// =========================================================================
// Start und Stopp
// =========================================================================
//
// Zwei getrennte Funktionen, und der Unterschied ist der ganze Sinn der
// Firmware:
//
//   webUiSetup()  - einmal beim Booten. Dateisystem, Puffer, Passwort.
//                   Das WLAN bleibt AUS.
//   webUiStart()  - auf Abruf, wenn die Farbfolge vollstaendig war.
//   webUiStop()   - wieder aus, wenn lange nichts passiert ist.
//
// Warum das WLAN nicht einfach immer laeuft: In der Praxis wird der Roboter
// ueber Bluetooth gesteuert, weil das die Schulvariante ist. Ein WLAN, das
// dann trotzdem da ist, stoert - es steht bei jedem Schueler in der
// WLAN-Liste, ohne gebraucht zu werden. Und: das ESP32-Funkmodul laeuft
// wegen ESP-NOW ohnehin rund um die Uhr. Das Access Point ist nicht der
// Stromfresser, das Aergerlichkeits-Argument ist es.

void webUiSetup() {
  Serial.println("[WEB] === Web-UI: Vorbereitung, WLAN bleibt aus ===");

  // ---- Dateisystem -----------------------------------------------------
  // Schon hier und nicht in webUiStart(): bei einem frisch geflashten
  // Geraet formatiert das LittleFS beim Mounten, und das Formatieren
  // dauert. Es soll nicht mitten im Signal der Farbfolge passieren.
  if (!LittleFS.begin(true, WEBUI_MOUNT_POINT, 10, WEBUI_PARTITION)) {
    Serial.println("[WEB] LittleFS nicht mountbar - es wird eine leere Seite geliefert");
    Serial.println("[WEB] Loesung: tools\\build_webui.ps1  und  pio run -t uploadfs");
  } else {
    Serial.printf("[WEB] LittleFS gemountet: %u KB benutzt von %u KB\n",
                  (unsigned)(LittleFS.usedBytes() / 1024),
                  (unsigned)(LittleFS.totalBytes() / 1024));
  }

  // ---- Statuspuffer ----------------------------------------------------
  // Statisch, nicht auf dem Heap. Siehe Kommentar an der Deklaration.
  s_statusQ = xQueueCreateStatic(WEBUI_STATUS_QUEUE_LEN, sizeof(StatusLine),
                                 s_statusQMem, &s_statusQStore);
  if (!s_statusQ) {
    Serial.println("[WEB] Statuspuffer konnte nicht angelegt werden - kein Web-Status");
  }

  // ---- Passwort und SSID vorbereiten ------------------------------------
  // Auch hier, nicht in webUiStart(): das Passwort wird beim allerersten
  // Start in NVS erzeugt, und das soll beim Einschalten des WLANs nicht
  // ploetzlich ein neues sein.
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

  // Einstiegsschutz. Die Farbfolge darf den Start nur ausloesen, wenn kein
  // Client verbunden ist - aber zwischen dem letzten Sensorwert und diesem
  // Aufruf kann sich ein Bluetooth-Geraet verbinden. Deshalb steht die Pruefung
  // auch hier und nicht nur in ApSignal.cpp: das WLAN wird nur eingeschaltet,
  // wenn der Roboter gerade ueber keinen anderen Weg bedient wird.
  if (anyClientConnected()) {
    Serial.println("[WEB] Start abgelehnt: ein Client ist verbunden - WLAN bleibt aus");
    return;
  }

  String password = webUiPassword();
  if (password.length() < 8) password = webUiEnsurePassword();

  // DHCP-Bereich. Muss VOR softAP() gesetzt sein, sonst bekommen die
  // Geraete die Standard-DNS des Arduino-Beispiels (192.168.4.1 passt
  // zwar, aber das ist Zufall und nicht Absicht).
  IPAddress ip(WEBUI_AP_IP), gw(WEBUI_AP_GATEWAY), mask(WEBUI_AP_SUBNET);
  if (!WiFi.softAPConfig(ip, gw, mask)) {
    Serial.println("[WEB] WARNUNG: DHCP-Konfiguration nicht angenommen");
  }

  // Kanal = WEBUI_AP_CHANNEL (1). PFLICHT, weil ESP-NOW auf Kanal 1 laeuft.
  bool ok = WiFi.softAP(s_ssid.c_str(), nullptr, WEBUI_AP_CHANNEL,
                        false, WEBUI_MAX_CLIENTS);
  if (!ok) {
    Serial.println("[WEB] FEHLER: Access Point liess sich nicht starten");
    return;
  }
  s_apUp = true;
  s_apStartedAt = millis();
  s_apIp = WiFi.softAPIP().toString();

  // Kein WLAN-Stromsparmodus - aber das heisst hier ausdruecklich: NICHT
  // selbst umstellen.
  //
  // Der Plan (Abschnitt 4.3) schlaegt esp_wifi_set_ps(WIFI_PS_NONE) vor. Das
  // laesst den Roboter sofort abstuerzen, sobald der Access Point startet:
  //
  //   E wifi:Error! Should enable WiFi modem sleep when both WiFi and
  //           Bluetooth are enabled!!!!!!
  //   abort() was called at PC ... -> Endlosschleife, Roboter nie erreichbar
  //
  // WLAN und Bluetooth teilen sich auf dem ESP32-S3 dasselbe Funkmodul.
  // Ist Bluetooth aktiv, VERLANGT der WLAN-Teil den Modem-Sleep. Arduino
  // setzt ihn von selbst richtig (WiFiGeneric.cpp:769, auf dem S3
  // WIFI_PS_MIN_MODEM). Ein WIFI_PS_NONE - hier oder via
  // WiFi.setSleep(false) - hebt das wieder auf und killt den Roboter.

  // ---- HTTP ------------------------------------------------------------
  // Handler nur EINMAL registrieren (ein zweites addHandler()/serveStatic()
  // wuerde jeden Request doppelt beantworten). s_server.begin() dagegen nach
  // jedem webUiStop() erneut rufen, weil webUiStop() s_server.end() aufruft.
  if (!s_handlersRegistered) {
    s_ws.onEvent(onWsEvent);
    s_server.addHandler(&s_ws);
    registerCaptiveHandlers();

    // Schriften: dauerhaft im Cache. Sie aendern sich nie, solange die
    // Oberflaeche dieselbe bleibt.
    s_server.serveStatic("/fonts/", LittleFS, "/fonts/")
        .setCacheControl("max-age=604800, immutable");

    // Oberflaeche: NIE cachen. Nach einem Firmware-Update zeigt ein iPad
    // sonst wochenlang die alte Version, weil Safari aggressiv cacht.
    // Bei ~120 KB gzip ist der Preis einer zweiten Uebertragung eine
    // Sekunde - der Preis eines verwirrenden Kindes ist groesser.
    s_server.serveStatic("/", LittleFS, "/")
        .setDefaultFile("index.html")
        .setCacheControl("no-cache, no-store, must-revalidate");

    // Alles, was keine Datei ist: zurueck auf die Startseite.
    s_server.onNotFound([](AsyncWebServerRequest* r) {
      r->redirect("http://192.168.4.1/");
    });
    s_handlersRegistered = true;
  }
  if (!s_serverStarted) {
    s_server.begin();
    s_serverStarted = true;
  }

  // ---- DNS -------------------------------------------------------------
  if (!s_dnsRunning) {
    s_dnsRunning = true;
    s_dnsAlive = true;
    if (xTaskCreatePinnedToCore(dnsTask, "dns", 4096, nullptr, 3, nullptr, 0) != pdPASS) {
      s_dnsRunning = false;
      s_dnsAlive = false;
      Serial.println("[WEB] DNS-Task liess sich nicht starten");
    }
  }

  // Der Moduswechsel kann den WLAN-Kanal zurueckgesetzt haben. Ohne diesen
  // Aufruf waeren die Dellen ab jetzt tot - ohne Fehlermeldung.
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

  s_dnsRunning = false;     // DNS-Task beendet sich spaetestens nach 250 ms Timeout
  for (int i = 0; i < 35 && s_dnsAlive; i++) {
    delay(10);
  }
  s_ws.closeAll();
  s_server.end();
  s_serverStarted = false;

  // Nur den AP abschalten, NICHT das Funkmodul: ESP-NOW braucht es fuer
  // die Duelle. WiFi.mode(WIFI_STA) hebt den AP auf und laesst den Treiber
  // laufen - anders als softAPdisconnect(true), das esp_wifi_stop() ruft
  // und damit ESP-NOW toeten wuerde.
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
    // Der Zustand, in dem der Roboter die meiste Zeit ist. Ausdruecklich
    // sagen, statt die IP leer zu lassen - sonst sieht man im Log nur einen
    // unvollstaendigen Satz und weiss nicht, ob das WLAN aus ist oder die
    // Diagnose kaputt ist.
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
    // uxQueueMessagesWaiting() liefert die Zahl der wartenden Eintraege -
    // also die BELegung, nicht die Restkapazitaet. Die umgekehrte Rechnung
    // hat hier zuerst "16 von 16 belegt" gemeldet, obwohl die Warteschlange
    // leer war und genau das war der Zustand, den man pruefen wollte.
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