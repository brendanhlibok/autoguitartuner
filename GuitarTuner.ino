// === CONFIGURATION ===
const int sampleRate = 5000;       // Hz
const int frameSize = 500;         // Number of audio samples per frame
const float thresh = 100.0;        // Threshold for validating signal
const int analogPin = A0;          // Audio input pin
const int buttonPin = 14;
const int ledPins[] = {2, 3, 4, 5, 6, 7};  // Assign digital pins for LEDs
const int numLEDs = sizeof(ledPins) / sizeof(ledPins[0]);
const int dirPin1 = 26;   // H-bridge input 1
const int dirPin2 = 24;   // H-bridge input 2
const int enablePin = 22; // H-bridge enable pin
const int numStates = 6;

// === GLOBAL VARIABLES ===
float frame[frameSize];            // Buffer to store audio samples

float periodRanges[6][2] = {
  {0.0123, 0.0113}, // Low E
  {0.0095, 0.0085}, // A
  {0.0071, 0.0062}, // D
  {0.0055, 0.0045}, // G
  {0.0042, 0.0034}, // B
  {0.0030, 0.0027} // High E
};

int currentLED = 0;
bool buttonPressed = false;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// === FUNCTION: Sample Audio from A0 ===
void sampleAudioFrame() {
  for (int i = 0; i < frameSize; i++) {
    frame[i] = analogRead(analogPin); // Reads raw ADC value (0-1023)
    delayMicroseconds(200);          // 5kHz sampling rate (200us interval)
  }
}

// === FUNCTION: Check if Frame is Valid ===
bool isValidFrame(float* frame, int size, float thresh) {
  int L = size * 0.3;
  float maxStart = -1e6;
  float maxEnd = -1e6;

  for (int i = 0; i < L; i++) {
    if (frame[i] > maxStart) maxStart = frame[i];
  }
  for (int i = size - L; i < size; i++) {
    if (frame[i] > maxEnd) maxEnd = frame[i];
  }

  return (maxStart > thresh) && (maxEnd > thresh);
}

// === FUNCTION: Compute ADMF ===
void computeADMF(float* frame, int size, float* admf) {
  int admfLength = size / 2;
  float maxVal = -1e6;

  for (int i = 0; i < admfLength; i++) {
    float sum = 0.0;
    for (int j = 0; j < admfLength; j++) {
      sum += fabs(frame[j] - frame[j + i]);
    }
    admf[i] = sum / admfLength;
    if (admf[i] > maxVal) maxVal = admf[i];
  }

  // Invert ADMF: admf = max(admf) - admf
  for (int i = 0; i < admfLength; i++) {
    admf[i] = maxVal - admf[i];
  }
}

// === FUNCTION: Find Peaks in ADMF ===
int findPeaks(float* data, int size, int* locs, float threshold) {
  int peakCount = 0;
  for (int i = 1; i < size - 1; i++) {
    if (data[i] > threshold && data[i] > data[i - 1] && data[i] > data[i + 1]) {
      locs[peakCount++] = i;
    }
  }
  return peakCount;
}

// === FUNCTION: Estimate Period from Peaks ===
float estimatePeriod(int* locs, int peakCount, int sampleRate) {
  if (peakCount < 2) return 0.0;

  float sumDiffs = 0.0;
  for (int i = 1; i < peakCount; i++) {
    sumDiffs += (locs[i] - locs[i - 1]);
  }
  float avgDiff = sumDiffs / (peakCount - 1);
  return avgDiff / sampleRate; // Period in seconds
}

// === FUNCTION: Analyze Audio Frame ===
float analyzeFrame(float* frame, int size) {
  if (!isValidFrame(frame, size, thresh)) {
    Serial.println("Frame invalid - too quiet or incomplete.");
    return 0.0;
  }

  int admfLength = size / 2;
  float admf[admfLength];
  computeADMF(frame, size, admf);

  // Find peaks in ADMF
  float peakThresh = 0.7 * admf[0];
  for (int i = 1; i < admfLength; i++) {
    if (admf[i] > peakThresh) peakThresh = 0.7 * admf[i];
  }

  int locs[admfLength];
  int peakCount = findPeaks(admf, admfLength, locs, peakThresh);

  // Print peak locations
  //Serial.print("Peak Count: ");
  //Serial.println(peakCount);
  //Serial.print("Peak Locations: ");
  for (int i = 0; i < peakCount; i++) {
    //Serial.print(locs[i]);
    //Serial.print(" ");
  }
  //Serial.println();

  // Estimate period
  float periodInSeconds = estimatePeriod(locs, peakCount, sampleRate);
  Serial.print("Estimated Period (s): ");
  Serial.println(periodInSeconds, 6);

  return periodInSeconds;
}

void commandMotor(float period, float max, float min){
  if (period > max){
    cwRot();
  } else if (period < min){
    ccRot();
  } else {
    digitalWrite(enablePin,LOW);
  }
  delay(200);
}

void cwRot(){
  digitalWrite(dirPin1, HIGH);
  digitalWrite(dirPin2, LOW);
  digitalWrite(enablePin, HIGH);
}

void ccRot(){
  digitalWrite(dirPin1, LOW);
  digitalWrite(dirPin2, HIGH);
  digitalWrite(enablePin, HIGH);
}

int setString(){
  int reading = digitalRead(buttonPin);
  //unsigned long currentTime = millis();
  // If button is pressed and not already handled
  if (reading == HIGH) {
    buttonPressed = true;
    //lastDebounceTime = currentTime;
    if (currentLED < 5){
      currentLED = (currentLED + 1);
    } else {
      currentLED = 0;
    }
  }

  // Reset buttonPressed when button is released
  if (reading == LOW) {
    buttonPressed = false;
    //lastDebounceTime = currentTime;
  }
}

void updateLEDs(int activeLED) {
  for (int i = 0; i < numLEDs; i++) {
    digitalWrite(ledPins[i], i == activeLED ? HIGH : LOW);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(dirPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(analogPin, INPUT);
  Serial.println("=== Audio Analysis Initialized ===");
  for (int i = 0; i < numLEDs; i++) {
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW);
  }
  updateLEDs(currentLED); // Turn on first LED
  pinMode(buttonPin, INPUT); // Button supplies HIGH when pressed (use pull-down resistor)
}

void loop() {
  setString();
  Serial.print("currentLED: ");
  Serial.println(currentLED);
  updateLEDs(currentLED);
  sampleAudioFrame();
  float period = analyzeFrame(frame, frameSize);
  commandMotor(period, periodRanges[currentLED][0],periodRanges[currentLED][1]);

  delay(100); // Optional delay between frames
}