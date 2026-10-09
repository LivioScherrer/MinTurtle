// EduBot V2 - Farbsensor-Implementierung
// Stand: exakte Funktionsweise aus Edu V1.3 (hat funktioniert).
#include "ColorSensor.h"
#include "Config.h"
#include "I2CBus.h"
#include "ProgGame.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <semphr.h>

Adafruit_TCS34725 tcs = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_50MS,
  TCS34725_GAIN_4X
);

volatile int sensorR = 0;
volatile int sensorG = 0;
volatile int sensorB = 0;
volatile uint16_t rawSensorR = 0, rawSensorG = 0, rawSensorB = 0, rawSensorC = 0;
volatile char sensorName[16] = "UNBEKANNT";

SemaphoreHandle_t colorSem = nullptr;
// volatile: siehe Deklaration in ColorSensor.h (Cross-Core-Sichtbarkeit).
volatile bool colorSensorOk = false;

int calibrateColor(uint16_t raw, uint16_t blackVal, uint16_t whiteVal) {
  if (whiteVal <= blackVal) return 0;
  long value = map(static_cast<long>(raw), static_cast<long>(blackVal), static_cast<long>(whiteVal), 0L, 255L);
  value = constrain(value, 0L, 255L);
  return static_cast<int>(value);
}

// ---- Eingelernte Farbreferenzen als erste Instanz -------------------
//
// CAL:RED/YELLOW/GREEN/BLUE legt den kalibrierten Messwert dieses Moments
// ab. Verglichen wird NICHT absolut: ein naechster Nachbar im RGB-Wuerfel
// waere genauso an die Helligkeit des Einlernzeitpunkts gekoppelt wie die
// festen Schwellen, die vorher allein entschieden haben.
//
// Verglichen wird die Chromazitaet (r,g,b)/Summe. Die bleibt bei
// gleichmaessig veraenderter Beleuchtung konstant - nachgerechnet fuer ein
// GELB: bei 40 % Licht 0,430/0,394/0,176 gegenueber 0,426/0,388/0,186, ein
// Drift von 0,026. Zum Vergleich: die alte Heuristik fiel bei 21 % weniger
// Licht schon aus und meldete bei 70 % weniger Licht ein Farbplaettchen als
// SCHWARZ - was der Tischkanten-Waechter als Kante liest.
//
// Warum das noetig war: die Heuristik allein erkennt nicht jede echte Farbe.
// Ein real gemessenes Gelb (28,212,101) hat einen so starken Gruenanteil, dass
// der alte Test "r > 100" scheitert und "g > 1.25*r && g > 1.25*b" greift - es
// wurde als GRUEN gemeldet. Der Abstand im Farbraum zwischen diesem Gelb und
// echtem Gruen (52,94,61) betraegt aber 0,34 und ist damit sauber trennbar.
//
// Nur wirklich eingelernte Referenzen wirken. Ohne CAL:ROT.. ist das
// Verhalten bitweise unveraendert. Reine Rechnung, kein I2C, keine Sperre,
// kein delay - der 150-ms-Task wartet nicht.
static const char* refColorName(int r, int g, int b, float tol) {
  const float sm = (float)(r + g + b);
  if (sm < 1.0f) return nullptr;
  const char* const names[4] = { "ROT", "GELB", "GRUEN", "BLAU" };
  const ColorRef* const refs[4] = { &config.calRot, &config.calGelb,
                                    &config.calGruen, &config.calBlau };
  for (int i = 0; i < 4; i++) {
    if (!refs[i]->calibrated) continue;
    const float sr = (float)(refs[i]->r + refs[i]->g + refs[i]->b);
    if (sr < 1.0f) continue;
    const float d = fabsf((float)r / sm - (float)refs[i]->r / sr)
                  + fabsf((float)g / sm - (float)refs[i]->g / sr)
                  + fabsf((float)b / sm - (float)refs[i]->b / sr);
    if (d <= tol) return names[i];
  }
  return nullptr;
}

const char* detectColorName(int r, int g, int b) {
  // Erst pruefen ob ueberhaupt eine Farbe erkannt wird (sehr dunkel)
  int total = r + g + b;
  if (total < 60) return "SCHWARZ";
  // WEISS: alle drei Kanaele hell und annaehernd gleich
  if (r > COLOR_HIGH && g > COLOR_HIGH && b > COLOR_HIGH &&
      abs(r - g) < 60 && abs(g - b) < 60 && abs(r - b) < 60) {
    return "WEISS";
  }
  // Eingelernte Referenzen schlagen die festen Heuristiken. Steht bewusst
  // HINTER SCHWARZ und WEISS: sonst wuerde eine eingelernte Zielfarbe die
  // Referenzflaeche wegfangen und Linienfolger, Tischkanten-Waechter und die
  // weisse Flaeche beim Einlernen verlieren ihr Weiss. Findet sich keine
  // Referenz, laeuft es unveraendert in die Heuristik.
  //
  // Toleranz 0,18: die kleinste Trennung zweier eingelernter Farben liegt bei
  // 0,34 (Gelb gegen Gruen), also bei 53 % davon - und rund siebenmal so gross
  // wie der groesste beobachtete Helligkeits-Drift von 0,075.
  const char* ref = refColorName(r, g, b, 0.18f);
  if (ref) return ref;
  // dominant channel
  int maxC = r > g ? (r > b ? r : b) : (g > b ? g : b);
  if (maxC < 50) return "UNBEKANNT";  // zu dunkel fuer echte Farbe

  // ---- Mindest-Saettigung fuer warme Toene (config.minColorSaturation) ----
  // Hintergrund: die GELB-/ROT-Pruefungen unten sind bewusst recht weit
  // gefasst, damit auch unter schlechter Kalibrierung noch Farben erkannt
  // werden. Die Kehrseite: eine blasse, warme Flaeche - abgenutztes Parkett,
  // ein beige Kartonstueck, warmes Licht auf einer weissen Flaeche - erfuellt
  // sie mit und wurde als GELB bzw. ROT gemeldet. Die Sattigung trennt das:
  // eine echte Farbmarke ist kraeftig gesaettigt, eine gewoehnliche
  // Umgebungsflaeche nicht.
  //
  // Belegte Reichweite der Schranke (analytisch und ueber den Farbraum
  // geprueft): die kleinste Saettigung, die GELB bzw. ROT ueberhaupt noch
  // erreichen, ist 0.218 bzw. 0.259 - beide liegen unter dem Default 0.40.
  // Die Schranke greift also genau dort, wo sie soll, namely bei blassen
  // und dunklen Warmtoenen. Gemessene reine Markerfarben liegen bei
  // Saettigung 0.9..1.0 und sind nie betroffen.
  //
  // Bewusst NUR auf GELB und ROT angewendet: Blau und Gruen koennen mit einer
  // warmen Umgebungsflaeche nicht verwechselt werden, und eine zu strenge
  // Schranke wuerde sie bei schlechter Kalibrierung faelschlich zu
  // UNBEKANNT machen. Die Division ist sicher, denn maxC >= 50 ist oben
  // bereits geprueft. Per SAT:<0..1> einstellbar; SAT:0 schaltet die
  // Schranke praktisch ab.
  const int minC = r < g ? (r < b ? r : b) : (g < b ? g : b);
  const float sat = (float)(maxC - minC) / (float)maxC;
  const bool warmEnough = (sat >= config.minColorSaturation);

  // GELB: rot und gruen hoch, blau deutlich niedriger.
  //
  // Der Blau-Test ist RELATIV (b < 0.60 * r) statt absolut (b < 80).
  //
  // Grund, gemessen (COM9, 2026-10): mit frisch eingelernten Grundwerten
  // (Weiss 4143 -> 4848, also 17 % zu hoch vorher) liegt der Blau-Kanal
  // eines gelben Plaettchens bei 111, nicht bei 71 wie unter der alten
  // Einlernung. Die absolute Grenze b < 80 stammte aus genau dieser alten
  // Einlernung und hat jedes Gelb durchfallen lassen - es wurde ROT
  // gemeldet und im Web-UI als "unbekannt" angezeigt.
  //
  // Der relative Test haengt nicht mehr an den Einlernwerten:
  //   Gelb  b/r = 111/254 = 0.44  -> erkannt
  //   Rot   b/r =  31/107 = 0.29  -> nicht als Gelb (faellt zu ROT durch)
  //   Weiss b/r =   0/255 = 0.00  -> nicht als Gelb
  // Die Faktoren 1.4f fuer r und g gegen b bleiben, sie sind bereits
  // relativ und damit einlernungsunabhaengig.
  if (warmEnough && r > 100 && g > 100 && b < r * 0.60f &&
      r > b * 1.4f && g > b * 1.4f) return "GELB";

  if (warmEnough && r > g * RATIO_RED && r > b * RATIO_RED) return "ROT";
  if (g > r * RATIO_GB && g > b * RATIO_GB) return "GRUEN";
  if (b > r * RATIO_GB && b > g * RATIO_GB) return "BLAU";
  return "UNBEKANNT";
}

void colorSetup() {
  // Bus-Mutex zuerst: colorSetup() und distSetup() teilen sich denselben Bus.
  i2cBusInit();
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!i2cBusLock(100)) {
    Serial.println("[SENSOR] I2C-Bus belegt -> colorSetup übersprungen");
    colorSem = xSemaphoreCreateBinary();
    if (colorSem) xSemaphoreGive(colorSem);
    return;
  }
  colorSensorOk = tcs.begin();
  i2cBusUnlock();
  if (colorSensorOk) {
    Serial.println("[SENSOR] TCS34725 gefunden");
  } else {
    Serial.println("[SENSOR] TCS34725 NICHT gefunden! (Trotzdem gestartet)");
  }
  colorSem = xSemaphoreCreateBinary();
  if (colorSem) xSemaphoreGive(colorSem);
}

bool resetI2CBus() {
  // Sperre MUSS halten: neun Clock-Pulse und das Wire.begin() wuerden
  // sonst mitten in einer VL53L0X-Transaktion den Bus umbauen. Ohne
  // Sperre zu arbeiten (Rueckgabewert ignorieren) ist genau der Fehler,
  // den I2CBus eigentlich verhindern soll.
  //
  // Rueckgabe false = Bus war belegt, es wurde NICHHT angefasst. Der
  // Aufrufer darf das nicht als Fehlversuch werten: der Distanzsensor
  // haelt den Bus waehrend seiner Kalibrierung sekundenlang, und drei
  // vergebliche Wartezeiten wuerden das Recovery-Budget erschoepfen und
  // den Farbsensor dauerhaft stilllegen.
  if (!i2cBusLock(200)) {
    Serial.println("[SENSOR] resetI2CBus: I2C-Bus belegt -> Bus nicht angefasst");
    return false;
  }
  pinMode(SDA_PIN, OUTPUT);
  pinMode(SCL_PIN, OUTPUT);
  for (int i = 0; i < 9; i++) {
    digitalWrite(SDA_PIN, LOW);
    pinMode(SCL_PIN, OUTPUT); digitalWrite(SCL_PIN, HIGH); delayMicroseconds(5);
    digitalWrite(SCL_PIN, LOW); delayMicroseconds(5);
  }
  pinMode(SDA_PIN, INPUT);
  pinMode(SCL_PIN, INPUT);
  Wire.begin(SDA_PIN, SCL_PIN);
  i2cBusUnlock();
  return true;
}

void getCurrentColorName(char* out, size_t outLen) {
  if (colorSem && xSemaphoreTake(colorSem, pdMS_TO_TICKS(20)) == pdTRUE) {
    strlcpy(out, (const char*)sensorName, outLen);
    xSemaphoreGive(colorSem);
  } else {
    strlcpy(out, "UNBEKANNT", outLen);
  }
}

// Anzahl der Recovery-Versuche, danach ruht der Sensor (kein Log-Spam und
// kein wiederholtes resetI2CBus(), das den VL53L0X mit zerstoeren wuerde).
static constexpr uint8_t COLOR_RECOVERY_TRIES = 3;

// Lebt der Sensor noch? Register 0x12 ist die Device-ID des TCS34725
// (0x44 = TCS34721/TCS34725, 0x4D = TCS34723/TCS34727).
//
// WICHTIG, es gaenger die Logik des 0-Wert-Erkennens unten: Adafruit_TCS34725
// meldet einen I2C-Fehler NICHT - read16() liefert bei Fehlschlag lautlos 0
// zurueck, und getRawData() gibt es gar nicht zurueck. Ein "alles 0" war
// deshalb zweideutig: entweder ist der Bus tot ODER der Sensor schaut auf
// etwas schwarzes. Drei schwarze Samples (ca. 0,5 s) genuegten fuer
// colorSensorOk = false, und nach drei Recovery-Versuchen ruhte der Task
// endgueltig - eine dunkle Linie, ein Schatten oder eine schwarze Platte
// unter dem Sensor hat also die Farberkennung bis zum naechsten Strom-
// zyklus abgeschaltet, und das untergrub dabei noch den gesunden I2C-Bus
// des Distanzsensors (resetI2CBus() = 9 Takt + Wire.begin()).
//
// Der Device-ID-Test trennt die beiden Faelle sauber: antwortet der Sensor
// auf 0x12, lebt er, und 0-Werte sind eine echte dunkle Messung.
static bool colorSensorAlive() {
  if (!i2cBusLock(30)) return true;  // Bus belegt: nicht als tot werten
  uint8_t id = 0;
  Wire.beginTransmission(TCS34725_ADDRESS);
  Wire.write(TCS34725_COMMAND_BIT | 0x12);
  bool ok = (Wire.endTransmission(false) == 0);
  if (ok) ok = (Wire.requestFrom((int)TCS34725_ADDRESS, (int)1) == 1);
  if (ok) id = Wire.read();
  i2cBusUnlock();
  return ok && (id == 0x44 || id == 0x4D);
}

void colorSensorTask(void* parameter) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(150);
  int failCount = 0;
  uint8_t recoveries = 0;

  while (true) {
    if (colorSensorOk) {
      uint16_t rawR = 0, rawG = 0, rawB = 0, rawC = 0;
      // I2C-Sequenz gegen den VL53L0X (Core 1) absichern.
      if (!i2cBusLock(30)) { vTaskDelayUntil(&lastWakeTime, period); continue; }
      tcs.getRawData(&rawR, &rawG, &rawB, &rawC);
      i2cBusUnlock();

      // Rohwerte unter colorSem veroeffentlichen. Die Leser (CAL:BLACK und
      // CAL:WHITE in Edu_V2.cpp) halten genau diese Sperre. Vorher wurde hier
      // ohne Sperre geschrieben - wodurch colorSem fuer diese vier Variablen
      // ueberhaupt keine gegenseitige Ausschliessung mehr war und eine
      // Kalibrierung R aus Sample N und G/B aus Sample N+1 lesen konnte.
      if (xSemaphoreTake(colorSem, pdMS_TO_TICKS(20)) == pdTRUE) {
        rawSensorR = rawR; rawSensorG = rawG; rawSensorB = rawB; rawSensorC = rawC;
        xSemaphoreGive(colorSem);
      }

      if (rawR == 0 && rawG == 0 && rawB == 0 && rawC == 0) {
        // 0/0/0/0 ist mehrdeutig -> erst nachfragen, ob der Sensor lebt.
        if (colorSensorAlive()) {
          // Sensor antwortet -> das ist eine echte dunkle Messung, kein
          // Busfehler. Als SCHWARZ melden, NICHT den Sensor abschalten.
          failCount = 0;
          if (xSemaphoreTake(colorSem, pdMS_TO_TICKS(100)) == pdTRUE) {
            sensorR = 0; sensorG = 0; sensorB = 0;
            strlcpy((char*)sensorName, "SCHWARZ", sizeof(sensorName));
            xSemaphoreGive(colorSem);
          }
        } else {
          failCount++;
          if (failCount >= 3) {
            colorSensorOk = false;
            Serial.println("[SENSOR] Farbsensor antwortet nicht (Device-ID weg) -> Recovery");
          }
        }
      } else {
        failCount = 0;
        edgeGuardTick(rawC);   // muss VOR der Kalibrierung stehen: nutzt die Rohwerte

        int r = calibrateColor(rawR, config.blackR, config.whiteR);
        int g = calibrateColor(rawG, config.blackG, config.whiteG);
        int b = calibrateColor(rawB, config.blackB, config.whiteB);

        const char* detected = detectColorName(r, g, b);

        // Timeout statt portMAX_DELAY - verhindert Task-Hang
        if (xSemaphoreTake(colorSem, pdMS_TO_TICKS(100)) == pdTRUE) {
          sensorR = r;
          sensorG = g;
          sensorB = b;
          strlcpy((char*)sensorName, detected, sizeof(sensorName));
          xSemaphoreGive(colorSem);
        }
      }
    } else if (recoveries < COLOR_RECOVERY_TRIES) {
      // Erst den Bus wirklich freigeraeumen, dann den Versuch zaehlen.
      // Sonst verbraucht ein belegter Bus (Distanzsensor kalibriert) das
      // Recovery-Budget, ohne dass ueberhaupt etwas versucht wurde.
      if (!resetI2CBus()) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        lastWakeTime = xTaskGetTickCount();
        vTaskDelayUntil(&lastWakeTime, period);
        continue;
      }
      recoveries++;
      Serial.printf("[SENSOR] Recovery-Versuch %d/%d\n", recoveries, COLOR_RECOVERY_TRIES);
      if (i2cBusLock(50)) {
        colorSensorOk = tcs.begin();
        i2cBusUnlock();
      }
      if (colorSensorOk) {
        Serial.println("[SENSOR] Recovered!");
        failCount = 0;
        recoveries = 0;
      }
      vTaskDelay(pdMS_TO_TICKS(1000));
      lastWakeTime = xTaskGetTickCount();
    } else {
      // Recovery erschoepft: nicht weiter den Bus anfassen. Der
      // Distanzsensor bleibt so ueber lebensfaehig.
      vTaskDelay(pdMS_TO_TICKS(2000));
      lastWakeTime = xTaskGetTickCount();
    }
    vTaskDelayUntil(&lastWakeTime, period);
  }
}
