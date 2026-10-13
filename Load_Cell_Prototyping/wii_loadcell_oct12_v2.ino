/*
 * ============================================================
 * ESP32-C3 Super Mini + HX711 + Wii Balance Board Load Cell
 * ============================================================
 *
 * V2: FAST STEP-AWARE FILTER + DRIFT HANDLING   (kg, 40 kg max)
 *
 * WHY V1 WAS SLOW
 *
 *   V1 took 25 samples per reading. The HX711 only makes
 *   ~10 samples/s, so every reading took ~2.5 s. It then took a
 *   median of the last 5 READINGS, so the display needed about
 *   3 new readings (~7-8 s) before it would follow a change.
 *   Weight changes mid-batch also got blended into one average.
 *
 * V2 PIPELINE (runs on every single HX711 sample, no batching)
 *
 *   raw sample
 *       |
 *   [1] median of 3          kills single-sample spikes (0.2 s lag)
 *       |
 *   [2] step-aware average   averages up to 20 samples when the
 *       |                    weight is still, but DUMPS its history
 *       |                    the moment the weight really changes,
 *       |                    so it follows steps in ~0.3 s
 *       |
 *   [3] zero tracking        slowly cancels drift of the zero
 *       |                    point, only when the scale looks
 *       |                    empty and still
 *       |
 *   displayed kg
 *
 * DRIFT HANDLING
 *
 *   - 30 s warm-up before the first tare (gauges drift most just
 *     after power-up).
 *   - Tare and calibration wait until the signal is quiet AND
 *     flat (no slope) before they capture anything.
 *   - Zero tracking follows slow zero drift. The running total is
 *     printed as zeroAdj (counts) so you can SEE the drift.
 *
 * WIRING
 *
 *   Load Cell: Red -> E+, Black -> E-, White -> A-, Green -> A+
 *   (swap A+ / A- if the reading goes negative when loaded)
 *   HX711: VCC -> 3V3, GND -> GND, DT -> GPIO4, SCK -> GPIO5
 *
 * SERIAL: 115200 baud
 *
 * COMMANDS
 *
 *   t = tare (waits for a stable zero)
 *   c = calibrate (known weight in KG)
 *   r = raw ADC readings
 *   z = zero tracking on / off
 *   + = calibration factor +1%
 *   - = calibration factor -1%
 *   h = help
 *
 * All sample counts below assume the HX711 default of 10 SPS.
 * ============================================================
 */

#include "HX711.h"
#include <math.h>


// ============================================================
// PINS / HX711
// ============================================================

const int DOUT_PIN = 4;
const int SCK_PIN  = 5;

HX711 scale;


// ============================================================
// LIMITS (KG)
// ============================================================

const float MAX_WEIGHT_KG   = 40.0;
const float WARN_FRACTION   = 0.90;
const float OVERLOAD_MARGIN = 1.02;


// ============================================================
// CALIBRATION  (counts per kg)
// ============================================================

float calFactor = 756.667;


// ============================================================
// STAGE 1: SPIKE REJECTION
// ============================================================

const int MED_WIN = 3;

long medBuf[MED_WIN];
int  medIdx   = 0;
int  medCount = 0;


// ============================================================
// STAGE 2: STEP-AWARE AVERAGE
// ============================================================
//
// AVG_MAX         longest averaging window (20 samples = 2 s).
//                 Longer = quieter when still. Shorter = less
//                 settling time after a small change.
//
// STEP_SIGMA      a sample counts as "different" if it is more
//                 than this many noise-sigmas from the average.
//
// STEP_FLOOR_KG   ...but never less than this many kg.
//
// STEP_CONFIRM    this many CONSECUTIVE different samples (same
//                 direction) are needed to call it a real step.
//                 One odd sample = spike = ignored.
//
// ============================================================

const int   AVG_MAX           = 20;
const int   STEP_MIN_HISTORY  = 3;
const int   SIGMA_MIN_HISTORY = 6;
const float STEP_SIGMA        = 4.5;
const float STEP_FLOOR_KG     = 0.15;
const int   STEP_CONFIRM      = 2;

long avgBuf[AVG_MAX];
int  avgIdx   = 0;
int  avgCount = 0;

long stepVals[STEP_CONFIRM];
int  stepCount = 0;
int  stepSign  = 0;
bool stepFlag  = false;


// ============================================================
// STAGE 3: ZERO TRACKING (DRIFT CANCELLATION)
// ============================================================
//
// Only acts when ALL of these are true:
//   - the average window is full (no step in the last 2 s)
//   - the reading is within ZERO_CAPTURE_KG of zero
//   - that has been true for ZERO_DELAY_MS
//
// Pulls the zero point toward the current reading by
// ZERO_TRACK_RATE per sample (0.01 = roughly a 10 s time
// constant). A real load bigger than ZERO_CAPTURE_KG is never
// touched. A load SMALLER than that WILL be zeroed away slowly.
//
// ============================================================

const float ZERO_DEADBAND_KG = 0.015;
const float ZERO_CAPTURE_KG  = 0.20;
const unsigned long ZERO_DELAY_MS = 3000;
const float ZERO_TRACK_RATE  = 0.01;

bool zeroTrackingOn = true;

double zeroOffset = 0.0;
double tareOffset = 0.0;

unsigned long nearZeroSince = 0;


// ============================================================
// STABILITY DETECTION (used by tare and calibration)
// ============================================================
//
// Collect STABLE_N samples. The window is "stable" when
//   - its standard deviation <= STABLE_STD_COUNTS, AND
//   - the second half's mean differs from the first half's mean
//     by <= STABLE_DRIFT_COUNTS (i.e. it is not sloping)
//
// If it never stabilises before the timeout, the last window is
// used anyway and you get a warning.
//
// ============================================================

const int   STABLE_N            = 20;
const float STABLE_STD_COUNTS   = 70.0;
const float STABLE_DRIFT_COUNTS = 50.0;

const unsigned long TARE_TIMEOUT_MS = 30000;
const unsigned long CAL_TIMEOUT_MS  = 40000;

// Time to let the cell and HX711 settle after power-up.
const unsigned long WARMUP_SECONDS = 30;


// ============================================================
// TIMING / STATE
// ============================================================

const unsigned long PRINT_INTERVAL = 250;

unsigned long lastPrint    = 0;
unsigned long lastSampleMs = 0;

long lastRaw = 0;


// ============================================================
// HELPER: SORT LONG ARRAY
// ============================================================

void sortLongArray(long values[], int count)
{
  for (int i = 0; i < count - 1; i++)
  {
    for (int j = i + 1; j < count; j++)
    {
      if (values[j] < values[i])
      {
        long temp = values[i];
        values[i] = values[j];
        values[j] = temp;
      }
    }
  }
}


// ============================================================
// FILTER: RESET
// ============================================================

void clearFilter()
{
  medIdx = 0;
  medCount = 0;

  avgIdx = 0;
  avgCount = 0;

  stepCount = 0;
  stepSign = 0;

  nearZeroSince = 0;
}


// ============================================================
// FILTER: AVERAGE BUFFER HELPERS
// ============================================================

void pushAvg(long value)
{
  avgBuf[avgIdx] = value;

  avgIdx = (avgIdx + 1) % AVG_MAX;

  if (avgCount < AVG_MAX)
  {
    avgCount++;
  }
}


double bufferMean()
{
  if (avgCount == 0)
  {
    return 0.0;
  }

  double sum = 0.0;

  for (int i = 0; i < avgCount; i++)
  {
    sum += avgBuf[i];
  }

  return sum / avgCount;
}


double bufferSigma()
{
  if (avgCount < 2)
  {
    return 0.0;
  }

  double mean = bufferMean();

  double sum = 0.0;

  for (int i = 0; i < avgCount; i++)
  {
    double d = avgBuf[i] - mean;

    sum += d * d;
  }

  return sqrt(sum / (avgCount - 1));
}


// Counts a sample must differ from the average to count as "different".
double stepThreshold()
{
  double threshold = STEP_FLOOR_KG * fabs(calFactor);

  if (avgCount >= SIGMA_MIN_HISTORY)
  {
    double adaptive = STEP_SIGMA * bufferSigma();

    if (adaptive > threshold)
    {
      threshold = adaptive;
    }
  }

  return threshold;
}


// ============================================================
// STAGE 1: MEDIAN OF LAST FEW RAW SAMPLES
// ============================================================

long medianOfWindow()
{
  long tmp[MED_WIN];

  for (int i = 0; i < medCount; i++)
  {
    tmp[i] = medBuf[i];
  }

  sortLongArray(tmp, medCount);

  return tmp[medCount / 2];
}


// ============================================================
// STAGE 2: STEP-AWARE AVERAGE
// ============================================================

void stepAwareAverage(long m)
{
  // Not enough history yet to judge: just collect.

  if (avgCount < STEP_MIN_HISTORY)
  {
    pushAvg(m);

    return;
  }


  double dev = (double)m - bufferMean();

  if (fabs(dev) <= stepThreshold())
  {
    // Normal sample. Any half-built step candidate was a spike.

    stepCount = 0;
    stepSign = 0;

    pushAvg(m);

    return;
  }


  // Sample is far from the average: possible step or spike.

  int sign = (dev > 0) ? 1 : -1;

  if (sign != stepSign)
  {
    stepSign = sign;
    stepCount = 0;
  }

  stepVals[stepCount] = m;

  stepCount++;


  if (stepCount < STEP_CONFIRM)
  {
    // Hold it back. If the next sample is normal, it was a spike.

    return;
  }


  // CONFIRMED STEP: throw away the old average and restart on
  // the new level, so the display follows immediately.

  avgIdx = 0;
  avgCount = 0;

  for (int i = 0; i < stepCount; i++)
  {
    pushAvg(stepVals[i]);
  }

  stepCount = 0;
  stepSign = 0;

  stepFlag = true;

  nearZeroSince = 0;
}


// ============================================================
// STAGE 3: ZERO TRACKING
// ============================================================

void updateZeroTracking()
{
  if (!zeroTrackingOn)
  {
    return;
  }


  // A full window means no step in the last AVG_MAX samples.

  if (avgCount < AVG_MAX)
  {
    nearZeroSince = 0;

    return;
  }


  double error = bufferMean() - zeroOffset;

  if (fabs(error) > ZERO_CAPTURE_KG * fabs(calFactor))
  {
    nearZeroSince = 0;

    return;
  }


  if (nearZeroSince == 0)
  {
    nearZeroSince = millis();

    return;
  }


  if (millis() - nearZeroSince < ZERO_DELAY_MS)
  {
    return;
  }


  zeroOffset += error * ZERO_TRACK_RATE;
}


// ============================================================
// PROCESS ONE HX711 SAMPLE
// ============================================================

void processSample(long raw)
{
  lastRaw = raw;

  lastSampleMs = millis();


  // Stage 1

  medBuf[medIdx] = raw;

  medIdx = (medIdx + 1) % MED_WIN;

  if (medCount < MED_WIN)
  {
    medCount++;
  }

  long m = medianOfWindow();


  // Stage 2

  stepAwareAverage(m);


  // Stage 3

  updateZeroTracking();
}


// ============================================================
// BLOCKING STABLE CAPTURE (tare / calibration)
// ============================================================
//
// Returns:
//    1  stable, mean is good
//    0  timed out while still unstable (mean = last window)
//   -1  HX711 did not respond
//
// ============================================================

int waitForStableMean(unsigned long timeoutMs, double &meanOut, double &sdOut)
{
  long win[STABLE_N];

  long sorted[STABLE_N];

  unsigned long start = millis();

  meanOut = 0.0;
  sdOut = 0.0;


  while (true)
  {
    for (int i = 0; i < STABLE_N; i++)
    {
      if (!scale.wait_ready_timeout(1000))
      {
        Serial.println("HX711 timeout.");

        return -1;
      }

      win[i] = scale.read();
    }


    // Half means (slope check)

    int half = STABLE_N / 2;

    double h1 = 0.0;
    double h2 = 0.0;

    for (int i = 0; i < half; i++)
    {
      h1 += win[i];
    }

    for (int i = half; i < STABLE_N; i++)
    {
      h2 += win[i];
    }

    h1 /= half;

    h2 /= (STABLE_N - half);


    // Standard deviation of the whole window

    double all = 0.0;

    for (int i = 0; i < STABLE_N; i++)
    {
      all += win[i];
    }

    all /= STABLE_N;

    double var = 0.0;

    for (int i = 0; i < STABLE_N; i++)
    {
      double d = win[i] - all;

      var += d * d;
    }

    double sd = sqrt(var / (STABLE_N - 1));


    // Trimmed mean (drop 3 lowest + 3 highest)

    for (int i = 0; i < STABLE_N; i++)
    {
      sorted[i] = win[i];
    }

    sortLongArray(sorted, STABLE_N);

    const int TRIM = 3;

    double total = 0.0;

    for (int i = TRIM; i < STABLE_N - TRIM; i++)
    {
      total += sorted[i];
    }

    meanOut = total / (STABLE_N - 2 * TRIM);

    sdOut = sd;


    double slope = fabs(h2 - h1);

    if (sd <= STABLE_STD_COUNTS && slope <= STABLE_DRIFT_COUNTS)
    {
      return 1;
    }


    Serial.print("  waiting for stable signal: sd=");

    Serial.print(sd, 1);

    Serial.print("  slope=");

    Serial.println(slope, 1);


    if (millis() - start >= timeoutMs)
    {
      return 0;
    }
  }
}


// ============================================================
// HELP
// ============================================================

void printHelp()
{
  Serial.println();

  Serial.println("======================================");
  Serial.println("COMMANDS");
  Serial.println("======================================");

  Serial.println("t = tare / zero (waits for stable)");
  Serial.println("c = calibrate (known weight in KG)");
  Serial.println("r = raw ADC readings");
  Serial.println("z = zero tracking on/off");
  Serial.println("+ = calibration factor +1%");
  Serial.println("- = calibration factor -1%");
  Serial.println("h = help");

  Serial.println();

  Serial.print("Calibration factor (counts/kg): ");

  Serial.println(calFactor, 3);

  Serial.print("Zero tracking: ");

  Serial.println(zeroTrackingOn ? "ON" : "OFF");

  Serial.print("Working maximum: ");

  Serial.print(MAX_WEIGHT_KG, 1);

  Serial.println(" kg");

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


  double mean = 0.0;
  double sd = 0.0;

  int result = waitForStableMean(TARE_TIMEOUT_MS, mean, sd);


  if (result < 0)
  {
    Serial.println("ERROR: HX711 did not respond.");

    return;
  }


  if (result == 0)
  {
    Serial.println("WARNING: signal never fully settled.");

    Serial.println("Zero tracking will clean up what is left.");
  }


  zeroOffset = mean;

  tareOffset = mean;

  clearFilter();


  Serial.print("ZERO COMPLETE. Offset = ");

  Serial.println(mean, 1);

  Serial.println();
}


// ============================================================
// RAW DUMP
// ============================================================

void doRawDump()
{
  Serial.println();

  Serial.println("--------------------------------------");
  Serial.println("RAW ADC READINGS");
  Serial.println("--------------------------------------");


  for (int i = 0; i < 20; i++)
  {
    if (!scale.wait_ready_timeout(1000))
    {
      Serial.println("HX711 timeout.");

      continue;
    }

    Serial.print("RAW: ");

    Serial.println(scale.read());
  }


  Serial.println();
}


// ============================================================
// CALIBRATION (KG)
// ============================================================

void doCalibrate()
{
  Serial.println();

  Serial.println("--------------------------------------");
  Serial.println("CALIBRATION (KG)");
  Serial.println("--------------------------------------");

  Serial.println("1. Remove ALL weight.");

  Serial.println("2. Keep platform completely still.");

  Serial.println("3. Send any character.");


  while (!Serial.available())
  {
    delay(10);
  }

  while (Serial.available())
  {
    Serial.read();
  }


  delay(1000);


  // ----------------------------------------------------------
  // ZERO
  // ----------------------------------------------------------

  double zeroMean = 0.0;
  double sd = 0.0;

  int result = waitForStableMean(TARE_TIMEOUT_MS, zeroMean, sd);

  if (result < 0)
  {
    Serial.println("HX711 timeout during zero.");

    return;
  }


  Serial.print("Zero captured. Counts = ");

  Serial.println(zeroMean, 1);

  Serial.println();

  Serial.println("Place the known weight on the platform.");

  Serial.println("Put it in the SAME spot you will use later.");

  Serial.println("Keep it completely still.");

  Serial.println();

  Serial.print("Use between 5 and ");

  Serial.print(MAX_WEIGHT_KG, 0);

  Serial.println(" kg (10-30 kg is best).");

  Serial.println();

  Serial.println("Enter the weight in KILOGRAMS (e.g. 20 or 12.5):");


  // ----------------------------------------------------------
  // KNOWN WEIGHT
  // ----------------------------------------------------------

  Serial.setTimeout(60000);

  float knownWeight = Serial.parseFloat();

  while (Serial.available())
  {
    Serial.read();
  }


  if (knownWeight <= 0)
  {
    Serial.println();

    Serial.println("Invalid weight. Calibration cancelled.");

    return;
  }


  if (knownWeight > MAX_WEIGHT_KG)
  {
    Serial.println();

    Serial.print("Weight is above the ");

    Serial.print(MAX_WEIGHT_KG, 0);

    Serial.println(" kg limit. Remove it and use less.");

    Serial.println("Calibration cancelled.");

    return;
  }


  // ----------------------------------------------------------
  // SETTLE, THEN CAPTURE
  // ----------------------------------------------------------

  Serial.println();

  Serial.println("Letting the cell settle...");

  delay(5000);


  double loadedMean = 0.0;

  result = waitForStableMean(CAL_TIMEOUT_MS, loadedMean, sd);

  if (result < 0)
  {
    Serial.println("HX711 timeout during measurement.");

    return;
  }


  if (result == 0)
  {
    Serial.println("WARNING: reading never settled (creep or weight moving).");

    Serial.println("The result below may be inaccurate.");
  }


  double counts = loadedMean - zeroMean;

  if (fabs(counts) < 100.0)
  {
    Serial.println();

    Serial.println("Almost no signal change. Check wiring and that");

    Serial.println("the weight is really loading the cell.");

    Serial.println("Calibration cancelled.");

    return;
  }


  calFactor = (float)(counts / knownWeight);

  zeroOffset = zeroMean;

  tareOffset = zeroMean;

  clearFilter();


  // ----------------------------------------------------------
  // RESULTS
  // ----------------------------------------------------------

  Serial.println();

  Serial.println("--------------------------------------");
  Serial.println("CALIBRATION COMPLETE");
  Serial.println("--------------------------------------");

  Serial.print("Known weight: ");

  Serial.print(knownWeight, 3);

  Serial.println(" kg");

  Serial.print("Signal change: ");

  Serial.print(counts, 0);

  Serial.println(" counts");

  Serial.print("New calibration factor (counts/kg): ");

  Serial.println(calFactor, 3);

  if (calFactor < 0)
  {
    Serial.println("(negative: A+ and A- are swapped; this still works)");
  }

  Serial.println();

  Serial.println("Copy this value into calFactor at the top.");

  Serial.println("Then check with a SECOND, different weight.");

  Serial.println();
}


// ============================================================
// COMMAND HANDLER
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


    case 'z':

      zeroTrackingOn = !zeroTrackingOn;

      Serial.print("Zero tracking: ");

      Serial.println(zeroTrackingOn ? "ON" : "OFF");

      break;


    case '+':

      calFactor *= 1.01;

      clearFilter();

      Serial.print("Calibration factor = ");

      Serial.println(calFactor, 3);

      break;


    case '-':

      calFactor *= 0.99;

      clearFilter();

      Serial.print("Calibration factor = ");

      Serial.println(calFactor, 3);

      break;


    case 'h':

      printHelp();

      break;


    default:

      break;
  }


  lastSampleMs = millis();
}


// ============================================================
// PRINT ONE DISPLAY LINE
// ============================================================

void printReading()
{
  if (avgCount == 0)
  {
    return;
  }


  double kg = (bufferMean() - zeroOffset) / calFactor;

  if (fabs(kg) < ZERO_DEADBAND_KG)
  {
    kg = 0.0;
  }


  Serial.print("kg: ");

  Serial.print(kg, 3);

  Serial.print("   raw: ");

  Serial.print(lastRaw);

  Serial.print("   win: ");

  Serial.print(avgCount);

  Serial.print("   zeroAdj: ");

  Serial.print(zeroOffset - tareOffset, 1);


  if (stepFlag)
  {
    Serial.print("   STEP");

    stepFlag = false;
  }


  if (kg > MAX_WEIGHT_KG * OVERLOAD_MARGIN)
  {
    Serial.print("   *** OVERLOAD - REMOVE WEIGHT ***");
  }
  else if (kg > MAX_WEIGHT_KG * WARN_FRACTION)
  {
    Serial.print("   ! near ");

    Serial.print(MAX_WEIGHT_KG, 0);

    Serial.print(" kg limit");
  }


  Serial.println();
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  delay(2000);


  Serial.println();

  Serial.println("======================================");

  Serial.println("HX711 + ESP32-C3 WII LOAD CELL (KG)");

  Serial.println("V2: step-aware filter + drift handling");

  Serial.println("======================================");

  Serial.println();


  scale.begin(DOUT_PIN, SCK_PIN);


  if (scale.wait_ready_timeout(2000))
  {
    Serial.println("HX711 detected.");
  }
  else
  {
    Serial.println("WARNING: HX711 not responding.");

    Serial.println("Check VCC, GND, DT->GPIO4, SCK->GPIO5.");
  }


  // Warm-up: let the strain gauge and HX711 settle.

  if (WARMUP_SECONDS > 0)
  {
    Serial.print("Warming up for ");

    Serial.print((long)WARMUP_SECONDS);

    Serial.println(" s. Keep the platform empty and still.");

    unsigned long start = millis();

    unsigned long lastMsg = start;

    while (millis() - start < WARMUP_SECONDS * 1000UL)
    {
      if (scale.is_ready())
      {
        scale.read();
      }

      if (millis() - lastMsg >= 10000)
      {
        lastMsg = millis();

        Serial.print("  ");

        Serial.print((long)(WARMUP_SECONDS - (millis() - start) / 1000));

        Serial.println(" s left...");
      }

      delay(5);
    }
  }


  doTare();

  printHelp();

  lastSampleMs = millis();
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  while (Serial.available())
  {
    handleCommand((char)Serial.read());
  }


  // One sample at a time, never blocking.

  if (scale.is_ready())
  {
    processSample(scale.read());
  }


  if (millis() - lastPrint >= PRINT_INTERVAL)
  {
    lastPrint = millis();

    printReading();
  }


  if (millis() - lastSampleMs > 2000)
  {
    lastSampleMs = millis();

    Serial.println("HX711 timeout - check wiring/power.");
  }
}
