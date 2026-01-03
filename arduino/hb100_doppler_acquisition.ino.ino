/*
 * HB100 RAW SAMPLER for MATLAB FFT Processing
 * Sends blocks of 512 samples
 */

#define RADAR_PIN A0
#define FFT_SIZE 512
#define SAMPLE_RATE 10000 // Hz

unsigned long sampleInterval = 1000000UL / SAMPLE_RATE;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("# RAW HB100 Sampler Ready");
}

void loop() {
  Serial.println("START");  // block header

  unsigned long lastMicros = micros();
  for (int i = 0; i < FFT_SIZE; i++) {
    // precise timed sampling
    while (micros() - lastMicros < sampleInterval);
    lastMicros = micros();

    int raw = analogRead(RADAR_PIN);
    Serial.println(raw);
  }

  Serial.println("END");  // block footer
  delay(20);              // small pause before next frame
}
