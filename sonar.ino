#include <HCSR04.h>

void setupSonar() {
  HCSR04.begin(TRIG_PIN, ECHO_PIN);
}