# Build & Upload Hinweise (Stand 2026-10-10, verifiziert)

## Was am 10.10. klemmte — und was es war

1. **Zwei PlatformIO-Cores.** `python -m platformio` nutzte Core 6.1.19
   (Python38), `pio.exe` nutzt 6.2.0. Beide teilen sich
   `~/.platformio/packages` — 6.1.19 hat `tool-scons` down-/`upgegradet
   und damit jeden Build zerschossen (`No platform named 'win32'`).
   **Behoben:** 6.1.19 per `pip uninstall platformio` entfernt.
   **Regel:** nur noch `pio.exe` aus `.platformio/penv/Scripts` (6.2.0)
   oder die VSCode-Extension nutzen, nie `python -m platformio`.
2. **Zwei Builds gleichzeitig ins selbe `.pio/build`.** Erkennbar an
   wandernden Fehlermeldungen (`Slalom.cpp.o`, `EspNow.cpp.o`,
   `LedStrip.cpp.o` fehlt mal hier, mal da) und blockierten Dateien
   beim Löschen. **Regel:** immer nur EIN Build gleichzeitig. Vor dem
   Löschen von `.pio/build` prüfen, dass kein `scons`/`cc1plus`/
   `xtensa-g++` mehr läuft.
3. **Langsamer Upload.** Board-Default ist 460800 Baud. Jetzt in
   `platformio.ini`: `upload_speed = 921600`, dazu `monitor_speed = 115200`
   und `monitor_port = COM11`. Ergebnis: **Upload in 44 s** (davon 10 s
   reines Flashen bei 1168 kbit/s), danach Bootlog verifiziert
   (TMC 8/8 Register, IFCNT zählt, Motor-Task läuft).

## Baseline (10.10.)

- RAM 20,9 % (68348 / 327680), Flash 43,5 % (1453729 / 3342336).
- `Edu_V2.html` 9409 Zeilen, `Edu_V2_dev.html` 280 Zeilen (OK, Konsole).
- Emoji-Gate: 0 Treffer.

## Bei kaputtem Build-Cache

1. Prozesse prüfen (`scons`, `cc1plus`, `xtensa`, `pio`) — erst wenn
   Ruhe ist, weiter.
2. Nur `.pio/build/seeed_xiao_esp32s3` löschen (nicht das ganze `.pio`,
   die Pakete sind fein).
3. Neu bauen. Falls es wieder wandernde `.o`-Fehler gibt: es läuft
   irgendwo ein zweiter Build — finden, nicht drumherum bauen.
