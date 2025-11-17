// Simple diagnostic sketch to read flip switch and limit switches
// Wiring (per ME350 setup):
// - Flip switch: pin 5, uses INPUT_PULLUP (HIGH = ON, LOW = OFF)
// - Left limit:  pin 8, INPUT_PULLUP (HIGH when pressed)
// - Right limit: pin 9, INPUT_PULLUP (HIGH when pressed)

#define FLIP_SWITCH_PIN 5
#define LIMIT_LEFT_PIN 8
#define LIMIT_RIGHT_PIN 9

void setup() {
  Serial.begin(115200);
  pinMode(FLIP_SWITCH_PIN, INPUT_PULLUP);
  pinMode(LIMIT_LEFT_PIN, INPUT_PULLUP);
  pinMode(LIMIT_RIGHT_PIN, INPUT_PULLUP);

  Serial.println(F("\nSwitch/Limit Diagnostics"));
  Serial.println(F("Flip: pin 5 (HIGH=ON), Left limit: pin 8 (HIGH=pressed), Right limit: pin 9 (HIGH=pressed)"));
  Serial.println(F("Reading states every 500 ms...\n"));
}

void loop() {
  bool flipOn = digitalRead(FLIP_SWITCH_PIN) == HIGH;
  bool leftPressed = digitalRead(LIMIT_LEFT_PIN) == HIGH;
  bool rightPressed = digitalRead(LIMIT_RIGHT_PIN) == HIGH;

  Serial.print(F("Flip: "));
  Serial.print(flipOn ? F("ON (HIGH)") : F("OFF (LOW)"));
  Serial.print(F(" | Left limit: "));
  Serial.print(leftPressed ? F("PRESSED (LOW)") : F("open (HIGH)"));
  Serial.print(F(" | Right limit: "));
  Serial.println(rightPressed ? F("PRESSED (LOW)") : F("open (HIGH)"));

  delay(500);
}
