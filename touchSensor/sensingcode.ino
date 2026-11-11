
const int touchPin = 4;
const int threshold = 800;

// Moving average filter settings
const int numSamples = 5;
int readings[numSamples];
int sampleIndex = 0;
long total = 0;
int sampleCount = 0;

void setup() {
  Serial.begin(115200);

  Serial.println("ESP32 Touch Sensor Started");

  // Initialize filter
  for (int i = 0; i < numSamples; i++) {
    readings[i] = 0;
  }
}

void loop() {

  // 1. Read touch sensor
  int touchValue = touchRead(touchPin);

  // 2. Update moving average filter
  total -= readings[sampleIndex];

  readings[sampleIndex] = touchValue;
  total += touchValue;

  // 3. Count actual samples
  if (sampleCount < numSamples) {
    sampleCount++;
  }

  // 4. Move to next sample position
  sampleIndex = (sampleIndex + 1) % numSamples;

  // 5. Calculate moving average
  int averageValue = total / sampleCount;

  // 6. Detect touch
  if (averageValue > threshold) {
    Serial.print("NOT TOUCHED: ");
    Serial.println(averageValue);
  }
  else {
    Serial.print("TOUCHED: ");
    Serial.println(averageValue);
  }

  delay(200);
}
