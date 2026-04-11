const int HALL_PIN = A0;
const int RESTING = 207;
const int THRESHOLD = 30;

unsigned long lastPulseTime = 0;
unsigned long pulsePeriod = 0;
bool lastState = false;

// What's your expected max RPM? 
// MIN_PULSE_GAP = 60,000,000 / (maxRPM * 4 magnets)
// e.g. for 500 RPM max: 60000000 / (500*4) = 30000 microseconds
const unsigned long MIN_PULSE_GAP = 30000;

// Averaging - keeps last 5 pulse periods and averages them
const int AVG_SIZE = 5;
unsigned long periods[AVG_SIZE];
int periodIndex = 0;
int periodCount = 0;

void setup() {
  Serial.begin(9600);
  memset(periods, 0, sizeof(periods));
}

void loop() {
  int raw = analogRead(HALL_PIN);
  bool magnetDetected = (raw - RESTING) > THRESHOLD;

  if (magnetDetected && !lastState) {
    unsigned long now = micros();
    unsigned long gap = now - lastPulseTime;

    if (lastPulseTime == 0) {
      lastPulseTime = now;
    } else if (gap > MIN_PULSE_GAP) {
      // Store in rolling average buffer
      periods[periodIndex] = gap;
      periodIndex = (periodIndex + 1) % AVG_SIZE;
      if (periodCount < AVG_SIZE) periodCount++;
      lastPulseTime = now;
    }
  }
  lastState = magnetDetected;

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();

    if (micros() - lastPulseTime > 1000000 || periodCount == 0) {
      Serial.println("RPM: 0");
    } else {
      // Average the stored periods
      unsigned long avgPeriod = 0;
      for (int i = 0; i < periodCount; i++) avgPeriod += periods[i];
      avgPeriod /= periodCount;

      float rpm = (60.0 * 1000000.0) / (avgPeriod * 4.0);
      Serial.print("RPM: ");
      Serial.println(rpm, 1);
    }
  }
}