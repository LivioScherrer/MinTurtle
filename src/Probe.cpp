// EduBot V2 - Praezisions-Pruefroutine (Probe), Implementierung.
// Siehe Probe.h.
#include "Probe.h"
#include "Motor.h"
#include "Config.h"

static long s_refL = 0;
static long s_refR = 0;

void probeBegin() {
  long l = 0, r = 0;
  motorStepPositions(l, r);
  s_refL = l;
  s_refR = r;
}

int32_t probeMM() {
  long l = 0, r = 0;
  motorStepPositions(l, r);
  // Mittelwert beider Raeder: gleicht den Drift-Unterschied aus und ist die
  // Naeherung fuer den Weg der Chassis-Mitte.
  const float steps = ((float)(l - s_refL) + (float)(r - s_refR)) * 0.5f;
  const float spmm  = configStepsPerMM();
  if (spmm <= 0.0f) return 0;
  return (int32_t)(steps / spmm);
}

uint8_t probeScoreMM(int32_t mm) {
  const int32_t absMm = (mm < 0) ? -mm : mm;
  if (absMm < PROBE_PERFECT_MM) return 2;
  if (absMm < PROBE_GOOD_MM)    return 1;
  return 0;
}
