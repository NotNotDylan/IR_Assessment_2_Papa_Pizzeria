// Arduino Uno R3 e-stop sender (pin 7 active-low with INPUT_PULLUP).
// Protocol (newline-terminated ASCII):
//   "ESTOP"  when pin7 == LOW  (grounded)
//   "CLEAR"  when pin7 == HIGH (released)
//   "HB"     heartbeat once per second

const uint8_t ESTOP_PIN = 7;
const unsigned long DEBOUNCE_MS   = 30;
const unsigned long HEARTBEAT_MS  = 1000;

bool lastStable = HIGH;    // because INPUT_PULLUP => idle HIGH
bool lastRead   = HIGH;
unsigned long lastEdgeMs  = 0;
unsigned long lastHbMs    = 0;

void setup() {
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  // Give the host a hint we’re alive
  Serial.println("HELLO");
}

void loop() {
  // --- debounce (edge + settle) ---
  int raw = digitalRead(ESTOP_PIN);
  unsigned long now = millis();

  if (raw != lastRead) {
    lastRead = raw;
    lastEdgeMs = now; // start debounce timer
  }

  if ((now - lastEdgeMs) >= DEBOUNCE_MS && lastStable != lastRead) {
    lastStable = lastRead;

    if (lastStable == LOW) {
      Serial.println("ESTOP");
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      Serial.println("CLEAR");
      digitalWrite(LED_BUILTIN, LOW);
    }
  }

  // --- heartbeat ---
  if (now - lastHbMs >= HEARTBEAT_MS) {
    lastHbMs = now;
    Serial.println("HB");
  }
}