/*
 * ============================================================
 * ESP32-C3 Super Mini + HX711 + 5 kg Load Cell
 * ============================================================
 *
 * RESPONSIVE + STABLE SAMPLING VERSION
 *
 * Measurement pipeline:
 *
 *   HX711 raw readings
 *          ↓
 *   outlier rejection
 *          ↓
 *   trimmed average
 *          ↓
 *   short rolling median
 *          ↓
 *   displayed weight
 *
 *
 * The previous version used slow exponential smoothing.
 * That caused the displayed value to lag badly when weights
 * were added OR removed.
 *
 * This version is designed to:
 *
 *   - respond quickly to weight changes
 *   - reject random noise spikes
 *   - remain stable when weight is sitting still
 *   - behave similarly going UP and DOWN
 *   - avoid large artificial lag
 *
 *
 * WIRING
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
 * SERIAL:
 *   115200 baud
 *
 *
 * COMMANDS:
 *
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
// PINS
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
//
// This will be replaced after calibration.
//
// Your latest calibration produced approximately:
//
//     -49.895
//
// BUT your newest calibration produced:
//
//     +402.684
//
// Therefore we use the newest value.
//
// ============================================================

float calFactor = 402.684;


// ============================================================
// RAW SAMPLING
// ============================================================
//
// Number of ADC readings collected for one measurement.
//
// 25 gives us much better noise rejection than 5 without
// creating a huge amount of lag.
//
// ============================================================

const int RAW_SAMPLES = 25;


// Number of readings removed from each end after sorting.
//
// 25 samples:
//
// remove lowest 4
// remove highest 4
// average remaining 17
//
// This removes isolated spikes.
const int TRIM_COUNT = 4;


// ============================================================
// MEDIAN HISTORY
// ============================================================
//
// Instead of slowly averaging old measurements forever,
// we keep only a SHORT history.
//
// This means the scale can respond quickly when the weight
// changes.
//
// ============================================================

const int HISTORY_SIZE = 5;

float weightHistory[HISTORY_SIZE];

int historyIndex = 0;

int historyCount = 0;


// ============================================================
// TIMING
// ============================================================

const unsigned long PRINT_INTERVAL = 300;

unsigned long lastPrint = 0;


// ============================================================
// ZERO SETTINGS
// ============================================================

const float ZERO_RANGE = 3.0;

const float ZERO_DEADBAND = 1.5;

const unsigned long ZERO_DELAY = 5000;

const float ZERO_TRACK_RATE = 0.003;

unsigned long nearZeroSince = 0;


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
// HELPER: SORT FLOAT ARRAY
// ============================================================

void sortFloatArray(float values[], int count)
{
  for (int i = 0; i < count - 1; i++)
  {
    for (int j = i + 1; j < count; j++)
    {
      if (values[j] < values[i])
      {
        float temp = values[i];

        values[i] = values[j];

        values[j] = temp;
      }
    }
  }
}


// ============================================================
// GET ROBUST RAW READING
// ============================================================
//
// 25 readings are collected.
//
// Then:
//
//     sort
//     ↓
//     remove 4 lowest
//     ↓
//     remove 4 highest
//     ↓
//     average middle 17
//
// ============================================================

long getRobustRaw()
{
  long readings[RAW_SAMPLES];


  for (int i = 0; i < RAW_SAMPLES; i++)
  {
    if (!scale.wait_ready_timeout(1000))
    {
      return scale.read();
    }

    readings[i] = scale.read();
  }


  sortLongArray(
    readings,
    RAW_SAMPLES
  );


  long total = 0;

  int start = TRIM_COUNT;

  int end =
    RAW_SAMPLES - TRIM_COUNT;


  int count =
    end - start;


  for (int i = start; i < end; i++)
  {
    total += readings[i];
  }


  return total / count;
}


// ============================================================
// MEDIAN OF RECENT WEIGHTS
// ============================================================
//
// The median is excellent for removing sudden spikes.
//
// Example:
//
//     100
//     101
//     100
//     250   <-- bad reading
//     101
//
// Median:
//
//     101
//
// rather than allowing 250 to pull the displayed value upward.
//
// ============================================================

float getMedianWeight()
{
  float values[HISTORY_SIZE];


  for (int i = 0; i < historyCount; i++)
  {
    values[i] =
      weightHistory[i];
  }


  sortFloatArray(
    values,
    historyCount
  );


  if (historyCount % 2 == 1)
  {
    return values[
      historyCount / 2
    ];
  }
  else
  {
    int a =
      (historyCount / 2) - 1;

    int b =
      historyCount / 2;

    return
      (values[a] + values[b]) / 2.0;
  }
}


// ============================================================
// ADD WEIGHT TO HISTORY
// ============================================================

float addWeightToHistory(float weight)
{
  weightHistory[historyIndex] =
    weight;


  historyIndex++;

  if (historyIndex >= HISTORY_SIZE)
  {
    historyIndex = 0;
  }


  if (historyCount < HISTORY_SIZE)
  {
    historyCount++;
  }


  return getMedianWeight();
}


// ============================================================
// CLEAR FILTER HISTORY
// ============================================================

void clearHistory()
{
  historyIndex = 0;

  historyCount = 0;


  for (int i = 0; i < HISTORY_SIZE; i++)
  {
    weightHistory[i] = 0.0;
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

  Serial.println("t = manual tare / zero");
  Serial.println("c = calibrate");
  Serial.println("r = raw ADC readings");
  Serial.println("+ = increase calibration factor");
  Serial.println("- = decrease calibration factor");
  Serial.println("h = help");

  Serial.println();

  Serial.print("Calibration factor: ");

  Serial.println(
    calFactor,
    3
  );

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


  delay(1500);


  if (!scale.wait_ready_timeout(2000))
  {
    Serial.println(
      "ERROR: HX711 did not become ready."
    );

    return;
  }


  // More readings for a cleaner zero point

  scale.tare(50);


  Serial.print("Done. Offset = ");

  Serial.println(
    scale.get_offset()
  );


  // Clear all previous filtered values

  clearHistory();


  nearZeroSince =
    millis();


  Serial.println();

  Serial.println("ZERO COMPLETE.");

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


    long raw =
      scale.read();


    Serial.print("RAW: ");

    Serial.println(raw);


    delay(100);
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


  Serial.println(
    "1. Remove ALL weight."
  );

  Serial.println(
    "2. Keep platform completely still."
  );

  Serial.println(
    "3. Send any character."
  );


  while (!Serial.available())
  {
    delay(10);
  }


  while (Serial.available())
  {
    Serial.read();
  }


  delay(1500);


  // ----------------------------------------------------------
  // TARE
  // ----------------------------------------------------------

  if (!scale.wait_ready_timeout(2000))
  {
    Serial.println(
      "HX711 timeout during tare."
    );

    return;
  }


  scale.tare(50);


  Serial.print("Zero set. Offset = ");

  Serial.println(
    scale.get_offset()
  );


  Serial.println();

  Serial.println(
    "Place the known weight on the platform."
  );

  Serial.println(
    "Keep it completely still."
  );

  Serial.println();

  Serial.println(
    "Enter the weight in GRAMS:"
  );


  // ----------------------------------------------------------
  // READ KNOWN WEIGHT
  // ----------------------------------------------------------

  Serial.setTimeout(30000);

  float knownWeight =
    Serial.parseFloat();


  while (Serial.available())
  {
    Serial.read();
  }


  if (knownWeight <= 0)
  {
    Serial.println();

    Serial.println(
      "Invalid weight."
    );

    Serial.println(
      "Calibration cancelled."
    );

    return;
  }


  // ----------------------------------------------------------
  // SETTLE
  // ----------------------------------------------------------

  Serial.println();

  Serial.println(
    "Waiting for the load cell to settle..."
  );


  delay(5000);


  // ----------------------------------------------------------
  // STABILITY MEASUREMENT
  // ----------------------------------------------------------
  //
  // Take 15 robust measurements.
  //
  // We then average them.
  //
  // This gives us a much more reliable calibration factor.
  //
  // ----------------------------------------------------------

  const int CAL_SAMPLES = 15;

  long calibrationValues[CAL_SAMPLES];


  Serial.println();

  Serial.println(
    "Taking calibration measurements..."
  );


  for (int i = 0; i < CAL_SAMPLES; i++)
  {
    long raw =
      getRobustRaw();


    long net =
      raw - scale.get_offset();


    calibrationValues[i] =
      net;


    Serial.print("Sample ");

    Serial.print(i + 1);

    Serial.print(": ");

    Serial.print(net);

    Serial.println(" counts");


    delay(250);
  }


  // ----------------------------------------------------------
  // SORT CALIBRATION VALUES
  // ----------------------------------------------------------

  sortLongArray(
    calibrationValues,
    CAL_SAMPLES
  );


  // ----------------------------------------------------------
  // TRIM CALIBRATION OUTLIERS
  // ----------------------------------------------------------

  const int CAL_TRIM = 3;


  long total = 0;

  int start =
    CAL_TRIM;

  int end =
    CAL_SAMPLES - CAL_TRIM;


  for (int i = start; i < end; i++)
  {
    total +=
      calibrationValues[i];
  }


  int usable =
    end - start;


  long averageCounts =
    total / usable;


  // ----------------------------------------------------------
  // CALCULATE FACTOR
  // ----------------------------------------------------------

  calFactor =
    (float)averageCounts /
    knownWeight;


  scale.set_scale(
    calFactor
  );


  // ----------------------------------------------------------
  // RESULTS
  // ----------------------------------------------------------

  Serial.println();

  Serial.println("--------------------------------------");
  Serial.println("CALIBRATION COMPLETE");
  Serial.println("--------------------------------------");


  Serial.print("Known weight: ");

  Serial.print(
    knownWeight,
    2
  );

  Serial.println(" g");


  Serial.print("Stable average counts: ");

  Serial.println(
    averageCounts
  );


  Serial.print("New calibration factor: ");

  Serial.println(
    calFactor,
    3
  );


  Serial.println();

  Serial.println(
    "Copy this value into calFactor at"
  );

  Serial.println(
    "the top of the sketch."
  );


  Serial.println();


  // Clear old filter data

  clearHistory();


  // Do not allow zero tracking while weight is present

  nearZeroSince = 0;
}


// ============================================================
// AUTOMATIC ZERO TRACKING
// ============================================================
//
// IMPORTANT:
//
// This only operates when the measured weight is already
// extremely close to zero.
//
// It cannot tare away a real 57 g, 81.5 g, etc.
//
// ============================================================

void updateZeroTracking(float grams)
{
  if (fabs(grams) > ZERO_RANGE)
  {
    nearZeroSince = 0;

    return;
  }


  if (nearZeroSince == 0)
  {
    nearZeroSince =
      millis();

    return;
  }


  if (
    millis() - nearZeroSince
    <
    ZERO_DELAY
  )
  {
    return;
  }


  if (!scale.wait_ready_timeout(1000))
  {
    return;
  }


  long currentRaw =
    scale.read_average(10);


  long currentOffset =
    scale.get_offset();


  long error =
    currentRaw -
    currentOffset;


  long correction =
    (long)(
      error *
      ZERO_TRACK_RATE
    );


  if (
    correction == 0 &&
    error != 0
  )
  {
    correction =
      (error > 0) ? 1 : -1;
  }


  scale.set_offset(
    currentOffset +
    correction
  );
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


    case '+':

      calFactor += 1.0;

      scale.set_scale(
        calFactor
      );

      Serial.print(
        "Calibration factor = "
      );

      Serial.println(
        calFactor,
        3
      );

      break;


    case '-':

      calFactor -= 1.0;

      scale.set_scale(
        calFactor
      );

      Serial.print(
        "Calibration factor = "
      );

      Serial.println(
        calFactor,
        3
      );

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


  delay(2000);


  Serial.println();

  Serial.println("======================================");

  Serial.println(
    "HX711 + ESP32-C3 SCALE"
  );

  Serial.println(
    "RESPONSIVE STABLE SAMPLING"
  );

  Serial.println(
    "======================================"
  );

  Serial.println();


  // Start HX711

  scale.begin(
    DOUT_PIN,
    SCK_PIN
  );


  if (
    scale.wait_ready_timeout(2000)
  )
  {
    Serial.println(
      "HX711 detected."
    );
  }
  else
  {
    Serial.println(
      "WARNING: HX711 not responding."
    );

    Serial.println(
      "Check VCC, GND, DT->GPIO4, SCK->GPIO5."
    );
  }


  // Set calibration

  scale.set_scale(
    calFactor
  );


  // Initial tare

  doTare();


  printHelp();
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  // ----------------------------------------------------------
  // SERIAL COMMANDS
  // ----------------------------------------------------------

  while (Serial.available())
  {
    char command =
      (char)Serial.read();

    handleCommand(command);
  }


  // ----------------------------------------------------------
  // MEASUREMENT TIMING
  // ----------------------------------------------------------

  if (
    millis() - lastPrint
    >=
    PRINT_INTERVAL
  )
  {
    lastPrint =
      millis();


    // --------------------------------------------------------
    // HX711 READY
    // --------------------------------------------------------

    if (
      !scale.wait_ready_timeout(1000)
    )
    {
      Serial.println(
        "HX711 timeout - check wiring/power."
      );

      return;
    }


    // --------------------------------------------------------
    // ROBUST RAW MEASUREMENT
    // --------------------------------------------------------

    long raw =
      getRobustRaw();


    // --------------------------------------------------------
    // OFFSET
    // --------------------------------------------------------

    long offset =
      scale.get_offset();


    // --------------------------------------------------------
    // NET COUNTS
    // --------------------------------------------------------

    long net =
      raw - offset;


    // --------------------------------------------------------
    // CONVERT TO GRAMS
    // --------------------------------------------------------

    float measuredWeight =
      (float)net /
      calFactor;


    // --------------------------------------------------------
    // ZERO TRACKING
    // --------------------------------------------------------

    updateZeroTracking(
      measuredWeight
    );


    // --------------------------------------------------------
    // ADD TO SHORT HISTORY
    // --------------------------------------------------------

    float displayedWeight =
      addWeightToHistory(
        measuredWeight
      );


    // --------------------------------------------------------
    // ZERO DEAD BAND
    // --------------------------------------------------------

    if (
      fabs(displayedWeight)
      <
      ZERO_DEADBAND
    )
    {
      displayedWeight = 0.0;
    }


    // --------------------------------------------------------
    // PRINT
    // --------------------------------------------------------

    Serial.print("raw: ");

    Serial.print(raw);


    Serial.print("   net: ");

    Serial.print(net);


    Serial.print("   measured: ");

    Serial.print(
      measuredWeight,
      1
    );


    Serial.print(" g   median: ");

    Serial.print(
      displayedWeight,
      1
    );


    Serial.println(" g");
  }
}
