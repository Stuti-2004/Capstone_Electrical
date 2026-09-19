/*
 * ============================================================
 * ESP32-C3 Super Mini + HX711 + 5 kg Load Cell
 * ============================================================
 *
 * FEATURES:
 *   - Manual tare
 *   - Calibration
 *   - Averaged weight readings
 *   - Slow automatic zero-drift correction
 *   - Zero deadband
 *   - Proper HX711 ready checking
 *
 * WIRING:
 *
 * Load Cell:
 *   Red   -> HX711 E+
 *   Black -> HX711 E-
 *   White -> HX711 A-
 *   Green -> HX711 A+
 *
 * HX711:
 *   VCC -> ESP32 3V3
 *   GND -> ESP32 GND
 *   DT  -> ESP32 GPIO4
 *   SCK -> ESP32 GPIO5
 *
 * SERIAL MONITOR:
 *   115200 baud
 *
 * COMMANDS:
 *   t = tare
 *   c = calibrate
 *   r = raw readings
 *   + = increase calibration factor
 *   - = decrease calibration factor
 *   h = help
 *
 * ============================================================
 */

#include "HX711.h"


// ============================================================
// PIN CONFIGURATION
// ============================================================

const int DOUT_PIN = 4;
const int SCK_PIN  = 5;


// ============================================================
// HX711
// ============================================================

HX711 scale;


// ============================================================
// CALIBRATION
// ============================================================

float calFactor = -188.860;


// ============================================================
// READING SETTINGS
// ============================================================

const int DISPLAY_SAMPLES = 10;

const unsigned long PRINT_INTERVAL = 500;

unsigned long lastPrint = 0;


// ============================================================
// AUTOMATIC ZERO SETTINGS
// ============================================================

// Maximum weight considered "close enough to zero"
const float ZERO_TRACK_RANGE = 3.0;

// Anything smaller than this is displayed as 0.0 g
const float ZERO_DEADBAND = 1.5;

// How quickly the zero follows slow drift
const float ZERO_TRACK_RATE = 0.01;

// Reading must remain near zero for this long
const unsigned long ZERO_TRACK_DELAY = 5000;

unsigned long nearZeroSince = 0;


// ============================================================
// HELP
// ============================================================

void printHelp()
{
  Serial.println();
  Serial.println("======================================");
  Serial.println("COMMANDS");
  Serial.println("======================================");

  Serial.println("t = manual tare / zero");
  Serial.println("c = calibrate");
  Serial.println("r = raw ADC readings");
  Serial.println("+ = increase calibration factor");
  Serial.println("- = decrease calibration factor");
  Serial.println("h = help");

  Serial.println();

  Serial.print("Current calibration factor: ");
  Serial.println(calFactor, 3);

  Serial.println();
}


// ============================================================
// TARE
// ============================================================

void doTare()
{
  Serial.println();
  Serial.println("--------------------------------------");
  Serial.println("TARING");
  Serial.println("--------------------------------------");

  Serial.println("Remove ALL weight.");
  Serial.println("Keep the platform completely still.");
  Serial.println();

  delay(1000);

  Serial.println("Taking zero readings...");

  // Wait for HX711
  if (!scale.wait_ready_timeout(2000))
  {
    Serial.println("ERROR: HX711 did not become ready.");
    Serial.println("Check wiring and power.");
    Serial.println();

    return;
  }

  // Take 30 readings for tare
  scale.tare(30);

  Serial.print("Done. Offset = ");
  Serial.println(scale.get_offset());

  nearZeroSince = millis();

  Serial.println();
  Serial.println("ZERO COMPLETE.");
  Serial.println();
}


// ============================================================
// RAW READINGS
// ============================================================

void doRawDump()
{
  Serial.println();
  Serial.println("--------------------------------------");
  Serial.println("RAW ADC READINGS");
  Serial.println("--------------------------------------");

  for (int i = 0; i < 10; i++)
  {
    if (!scale.wait_ready_timeout(1000))
    {
      Serial.println("HX711 timeout.");
      delay(200);
      continue;
    }

    long raw = scale.read();

    Serial.print("RAW: ");
    Serial.println(raw);

    delay(150);
  }

  Serial.println();
}


// ============================================================
// CALIBRATION
// ============================================================

void doCalibrate()
{
  Serial.println();
  Serial.println("--------------------------------------");
  Serial.println("CALIBRATION");
  Serial.println("--------------------------------------");

  Serial.println("1. Remove ALL weight.");
  Serial.println("2. Keep the platform still.");
  Serial.println("3. Send any character.");

  // Wait for user input
  while (!Serial.available())
  {
    delay(10);
  }

  // Clear serial input
  while (Serial.available())
  {
    Serial.read();
  }

  delay(1000);

  // Tare first
  if (!scale.wait_ready_timeout(2000))
  {
    Serial.println("HX711 timeout during tare.");
    return;
  }

  scale.tare(30);

  Serial.print("Zero set. Offset = ");
  Serial.println(scale.get_offset());

  Serial.println();

  Serial.println("Place a known weight on the platform.");
  Serial.println("Let it sit still for a few seconds.");
  Serial.println();

  Serial.println("Enter the weight in GRAMS:");

  Serial.setTimeout(30000);

  float knownWeight = Serial.parseFloat();

  // Clear serial input
  while (Serial.available())
  {
    Serial.read();
  }

  if (knownWeight <= 0)
  {
    Serial.println();
    Serial.println("Invalid weight.");
    Serial.println("Calibration cancelled.");
    Serial.println();

    return;
  }

  // Give load cell time to settle
  Serial.println();
  Serial.println("Waiting for the weight to settle...");

  delay(3000);


  // Take several groups of measurements
  const int CALIBRATION_GROUPS = 5;

  long totalCounts = 0;

  Serial.println();
  Serial.println("Taking calibration readings...");

  for (int i = 0; i < CALIBRATION_GROUPS; i++)
  {
    if (!scale.wait_ready_timeout(2000))
    {
      Serial.println("HX711 timeout during calibration.");
      return;
    }

    long reading = scale.get_value(20);

    totalCounts += reading;

    Serial.print("Sample ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(reading);
    Serial.println(" counts");

    delay(500);
  }


  // Calculate average
  long averageCounts =
    totalCounts / CALIBRATION_GROUPS;


  // Calculate calibration factor
  calFactor =
    (float)averageCounts / knownWeight;


  // Apply factor
  scale.set_scale(calFactor);


  // Print result
  Serial.println();

  Serial.println("--------------------------------------");
  Serial.println("CALIBRATION COMPLETE");
  Serial.println("--------------------------------------");

  Serial.print("Known weight: ");
  Serial.print(knownWeight, 2);
  Serial.println(" g");

  Serial.print("Average counts: ");
  Serial.println(averageCounts);

  Serial.print("New calibration factor: ");
  Serial.println(calFactor, 3);

  Serial.println();

  Serial.println("Copy this value into calFactor at");
  Serial.println("the top of the sketch.");

  Serial.println();

  nearZeroSince = 0;
}


// ============================================================
// AUTOMATIC ZERO DRIFT CORRECTION
// ============================================================

void updateZeroTracking(float grams)
{
  // If there is more than a few grams on the scale,
  // assume this is real weight.
  //
  // DO NOT change the zero offset.

  if (fabs(grams) > ZERO_TRACK_RANGE)
  {
    nearZeroSince = 0;

    return;
  }


  // Reading is close to zero.
  // Start the timer.

  if (nearZeroSince == 0)
  {
    nearZeroSince = millis();

    return;
  }


  // Don't correct anything until the reading has
  // stayed near zero for several seconds.

  if (millis() - nearZeroSince < ZERO_TRACK_DELAY)
  {
    return;
  }


  // Get a fresh raw reading

  if (!scale.wait_ready_timeout(1000))
  {
    return;
  }

  long currentRaw =
    scale.read_average(5);


  // Current zero offset

  long currentOffset =
    scale.get_offset();


  // Difference between current raw value
  // and the current zero

  long error =
    currentRaw - currentOffset;


  // Move only a small amount toward the
  // current value.

  long correction =
    (long)(error * ZERO_TRACK_RATE);


  // Make sure tiny drift can eventually move

  if (correction == 0 && error != 0)
  {
    correction =
      (error > 0) ? 1 : -1;
  }


  // New zero offset

  long newOffset =
    currentOffset + correction;


  scale.set_offset(newOffset);
}


// ============================================================
// SERIAL COMMAND HANDLER
// ============================================================

void handleCommand(char command)
{
  switch (command)
  {
    case 't':
      doTare();
      break;


    case 'c':
      doCalibrate();
      break;


    case 'r':
      doRawDump();
      break;


    case '+':
      calFactor += 1.0;

      scale.set_scale(calFactor);

      Serial.print("Calibration factor = ");
      Serial.println(calFactor, 3);

      break;


    case '-':
      calFactor -= 1.0;

      scale.set_scale(calFactor);

      Serial.print("Calibration factor = ");
      Serial.println(calFactor, 3);

      break;


    case 'h':
      printHelp();
      break;


    default:
      break;
  }
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  delay(2000);SS

  Serial.println();
  Serial.println("======================================");
  Serial.println("HX711 + ESP32-C3 SCALE");
  Serial.println("DRIFT-COMPENSATED VERSION");
  Serial.println("======================================");
  Serial.println();


  // Start HX711

  scale.begin(
    DOUT_PIN,
    SCK_PIN
  );


  // Check HX711

  if (scale.wait_ready_timeout(2000))
  {
    Serial.println("HX711 detected.");
  }
  else
  {
    Serial.println("WARNING: HX711 not responding.");
    Serial.println("Check wiring and power.");
  }


  // Apply calibration factor

  scale.set_scale(calFactor);


  // Initial tare

  doTare();


  // Help menu

  printHelp();
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  // ----------------------------------------------------------
  // CHECK SERIAL COMMANDS
  // ----------------------------------------------------------

  while (Serial.available())
  {
    char command =
      (char)Serial.read();

    handleCommand(command);
  }


  // ----------------------------------------------------------
  // PRINT WEIGHT
  // ----------------------------------------------------------

  if (millis() - lastPrint >= PRINT_INTERVAL)
  {
    lastPrint = millis();


    // Wait for HX711 conversion

    if (!scale.wait_ready_timeout(1000))
    {
      Serial.println("HX711 timeout - check wiring/power.");

      return;
    }


    // Get averaged raw reading

    long raw =
      scale.read_average(DISPLAY_SAMPLES);


    // Get current zero offset

    long offset =
      scale.get_offset();


    // Calculate net counts

    long net =
      raw - offset;


    // Convert to grams

    float grams =
      (float)net / calFactor;


    // Automatic drift correction

    updateZeroTracking(grams);


    // Small zero deadband

    if (fabs(grams) < ZERO_DEADBAND)
    {
      grams = 0.0;
    }


    // Print measurement

    Serial.print("raw: ");
    Serial.print(raw);

    Serial.print("   net: ");
    Serial.print(net);

    Serial.print("   weight: ");
    Serial.print(grams, 1);

    Serial.println(" g");
  }
}
