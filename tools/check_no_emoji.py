#!/usr/bin/env python3
"""Gate: keine Emoji und keine System-Glyphen in den Web-Seiten.

Der Look der Seite ist Handschrift + Tinte. Jedes Zeichen, das vom
Betriebssystem geliefert wird (Emoji-Farben, dickleufige Glyphen),
bricht diesen Look. Deshalb: alles in der Oberfläche selbst gezeichnet.

Ausnahmen:
  * Pfeile U+2190..U+21FF   bewusst erlaubt (monochrom, in jeder Schrift
                            vorhanden), werden nur als INFO gemeldet.
  * Zeilen mit dem Marker   'emoji-export-ok'  werden uebersprungen.
                            Nur fuer reine Text-Exporte in die
                            Zwischenablage, niemals fuer sichtbare
                            Oberflaeche.

Aufruf:
    python tools/check_no_emoji.py                 # alle Web-Seiten
    python tools/check_no_emoji.py datei.html      # einzelne Datei
Exit 0 = sauber, Exit 1 = Treffer.
"""
import sys
from pathlib import Path

# (von, bis, kat) -- kat: 'fail' blockiert, 'info' nur melden
RANGES = [
    (0x1F000, 0x1FAFF, 'fail'),   # Emoji (Transport, Spiel, Gesicht, ...)
    (0x1F1E6, 0x1F1FF, 'fail'),   # Regionalindikatoren (Flaggen)
    (0x2B00, 0x2BFF, 'fail'),     # ⬜ ⬛ ⭐ ⬅  (Zusatz-Symbole)
    (0x2600, 0x27BF, 'fail'),     # ☀ ★ ✓ ✗ ⚠ ➜ ✅  (Misc + Dingbats)
    (0x2300, 0x23FF, 'fail'),     # ⏳ ⏱ ⚲ ⌛  (Technische Zeichen)
    (0x25A0, 0x25FF, 'fail'),     # ▲ ▼ ▾ ■ □  (geometrische Formen)
    (0xFE00, 0xFE0F, 'fail'),     # Variation Selector 15/16
    (0x200D, 0x200D, 'fail'),     # Zero Width Joiner (nur in Emoji-Sequenzen)
    (0x20E3, 0x20E3, 'fail'),     # Keycap-Overlay (1️⃣)
    (0xFF00, 0xFFEF, 'fail'),     # Vollbreite Formen (％ ＋ －)
    (0x2190, 0x21FF, 'info'),     # Pfeile ← → ↑ ↓
]


def classify(ch):
    cp = ord(ch)
    for lo, hi, kind in RANGES:
        if lo <= cp <= hi:
            return kind
    return None


def scan(path):
    fails = []
    infos = []
    text = Path(path).read_text(encoding='utf-8')
    # Entities aufloesen: sonst tarnt &#9654; ein verbotenes Dreieck als
    # harmlosen ASCII-Text (gesehen am Programm-Start-Button). Import lokal,
    # damit das Tool ohne Abhaengigkeiten lauffaehig bleibt.
    import html as _html
    text = _html.unescape(text)
    for no, line in enumerate(text.split('\n'), 1):
        if 'emoji-export-ok' in line:
            continue
        for kind, bucket in (('fail', fails), ('info', infos)):
            hits = sorted({ch for ch in line if classify(ch) == kind})
            if hits:
                bucket.append((no, ''.join(hits), line.strip()[:120]))
    return fails, infos


def main(argv):
    if argv:
        files = argv
    else:
        root = Path(__file__).resolve().parent.parent
        files = [str(root / 'Edu_V2.html'), str(root / 'Edu_V2_dev.html')]
    bad = 0
    for f in files:
        p = Path(f)
        if not p.exists():
            print('FEHLT: %s' % p)
            bad += 1
            continue
        fails, infos = scan(p)
        count = sum(len(ch) for _, ch, _ in fails)
        print('%s: %d Treffer auf %d Zeilen' % (p.name, count, len(fails)))
        for no, ch, ctx in fails:
            print('  %5d  %-8s %s' % (no, ch, ctx))
        for no, ch, ctx in infos:
            print('  INFO %5d  %-8s %s' % (no, ch, ctx))
        if fails:
            bad += 1
    if bad:
        print('\nFEHLER: Emoji/System-Glyphen gefunden -> ersetzen.')
        return 1
    print('\nOK: keine Emoji, keine System-Glyphen.')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
