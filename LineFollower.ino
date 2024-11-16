/* ------ Arduino Line Follower Code ------ */

/* ------- Defining Inputs ------ */
#define LS 2      // Left sensor
#define RS 3      // Right sensor

/* ------- Defining Outputs ------ */
#define LM1 4     // Left motor forward
#define LM2 5     // Left motor backward
#define RM1 6     // Right motor forward
#define RM2 7     // Right motor backward

void setup() {
  // Configure sensor pins
  pinMode(LS, INPUT);
  pinMode(RS, INPUT);

  // Configure motor pins
  pinMode(LM1, OUTPUT);
  pinMode(LM2, OUTPUT);
  pinMode(RM1, OUTPUT);
  pinMode(RM2, OUTPUT);
}

void loop() {
  // Move Forward
  if (digitalRead(LS) && digitalRead(RS)) {
    digitalWrite(LM1, HIGH);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, HIGH);
    digitalWrite(RM2, LOW);
  }

  // Turn Right
  if (!digitalRead(LS) && digitalRead(RS)) {
    digitalWrite(LM1, LOW);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, HIGH);
    digitalWrite(RM2, LOW);
  }

  // Turn Left
  if (digitalRead(LS) && !digitalRead(RS)) {
    digitalWrite(LM1, HIGH);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, LOW);
    digitalWrite(RM2, LOW);
  }

  // Stop
  if (!digitalRead(LS) && !digitalRead(RS)) {
    digitalWrite(LM1, LOW);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, LOW);
    digitalWrite(RM2, LOW);
  }
}
