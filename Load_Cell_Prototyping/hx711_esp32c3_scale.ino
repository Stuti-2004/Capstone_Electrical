/*
 * HX711 + 5 kg bar load cell on an ESP32-C3 Super Mini
 * -----------------------------------------------------
 * Library:  "HX711 Arduino Library" by Bogdan Necula (bogde)
 * Board:    ESP32C3 Dev Module
 * IMPORTANT board setting: Tools -> USB CDC On Boot -> Enabled
 *
 * Wiring
 *   Load cell red   -> HX711 E+
 *   Load cell black -> HX711 E-
 *   Load cell white -> HX711 A-
 *   Load cell green -> HX711 A+
 *   HX711 VCC -> ESP32 3V3      (NOT 5V - the DT line would be 5V logic)
 *   HX711 GND -> ESP32 GND
 *   HX711 DT  -> GPIO4
 *   HX711 SCK -> GPIO5
 *
 * Serial commands (115200 baud, Newline or No line ending both fine):
 *   t  tare (zero) with the platform empty
 *   c  calibrate: you'll be asked for the weight of a known object in grams
 *   +  increase calibration factor by 1
 *   -  decrease calibration factor by 1
 *   r  dump 10 raw ADC values (diagnostics)
 *   h  help
 */

#include "HX711.h"

// ---------------------------------------------------------------- config
const int DOUT_PIN = 4;
const int SCK_PIN  = 5;

// Raw ADC counts per gram. This is a starting guess only - run 'c' to find
// the real value for your cell, then paste it here so it survives a reboot.
float calFactor = 420.0;

const uint8_t SAMPLES = 5;      // readings averaged per displayed value
const uint32_t PRINT_MS = 500;  // how often to print

// ---------------------------------------------------------------- globals
HX711 scale;
uint32_t lastPrint = 0;

// ---------------------------------------------------------------- helpers
void printHelp() {
  Serial.println();
  Serial.println(F("Commands:  t = tare   c = calibrate   + / - = nudge factor"));
  Serial.println(F("           r = raw dump   h = help"));
  Serial.printf ("Current calibration factor: %.3f counts/gram\n\n", calFactor);
}

void doTare() {
  Serial.println(F("\nTaring - keep the platform empty and still..."));
  scale.tare(20);                     // average 20 readings for the zero point
  Serial.printf("Done. Offset = %ld\n\n", scale.get_offset());
}

void doRawDump() {
  Serial.println(F("\nRaw ADC values:"));
  for (int i = 0; i < 10; i++) {
    Serial.printf("  %ld\n", scale.read());
    delay(120);
  }
  Serial.println(F("Values near +8388607 or -8388608 mean the input is railed:"));
  Serial.println(F("check A+/A- and E+/E- wiring.\n"));
}

void doCalibrate() {
  Serial.println(F("\n--- Calibration ---"));
  Serial.println(F("1. Empty the platform, then send any character."));
  while (!Serial.available()) delay(10);
  while (Serial.available()) Serial.read();

  scale.tare(20);
  Serial.printf("   Zero set (offset = %ld)\n", scale.get_offset());

  Serial.println(F("2. Place a known weight on the platform."));
  Serial.println(F("3. Type its mass in GRAMS and press Enter:"));

  Serial.setTimeout(30000);
  float known = Serial.parseFloat();
  while (Serial.available()) Serial.read();

  if (known <= 0) {
    Serial.println(F("   No valid weight entered - calibration aborted.\n"));
    return;
  }

  long netCounts = scale.get_value(20);      // averaged, offset already removed
  calFactor = (float)netCounts / known;
  scale.set_scale(calFactor);

  Serial.printf("   Net counts: %ld for %.1f g\n", netCounts, known);
  Serial.printf("   New calibration factor: %.3f\n", calFactor);
  Serial.println(F("   Copy this into calFactor at the top of the sketch.\n"));
}

void handleCommand(char c) {
  switch (c) {
    case 't': doTare();      break;
    case 'c': doCalibrate(); break;
    case 'r': doRawDump();   break;
    case 'h': printHelp();   break;
    case '+': calFactor += 1; scale.set_scale(calFactor);
              Serial.printf("calFactor = %.3f\n", calFactor); break;
    case '-': calFactor -= 1; scale.set_scale(calFactor);
              Serial.printf("calFactor = %.3f\n", calFactor); break;
    default:  break;                          // ignore \r, \n, junk
  }
}

// ---------------------------------------------------------------- setup
void setup() {
  Serial.begin(115200);
  delay(2000);                    // let native USB CDC enumerate before printing
  Serial.println();
  Serial.println(F("=== HX711 load cell / ESP32-C3 Super Mini ==="));

  scale.begin(DOUT_PIN, SCK_PIN);

  if (scale.wait_ready_timeout(2000)) {
    Serial.println(F("HX711 detected."));
  } else {
    Serial.println(F("HX711 is NOT responding."));
    Serial.println(F("Check VCC (3V3), GND, DT->GPIO4, SCK->GPIO5."));
  }

  scale.set_scale(calFactor);
  doTare();
  printHelp();
}

// ---------------------------------------------------------------- loop
void loop() {
  while (Serial.available()) handleCommand((char)Serial.read());

  if (millis() - lastPrint >= PRINT_MS) {
    lastPrint = millis();

    if (!scale.is_ready()) return;

    long  raw   = scale.read_average(SAMPLES);
    float grams = (raw - scale.get_offset()) / calFactor;

    Serial.printf("raw: %9ld   net: %9ld   weight: %8.1f g\n",
                  raw, raw - scale.get_offset(), grams);
  }
}
