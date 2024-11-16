/* ------ Arduino Advanced Motor Control Code ------ */

// Speed and Turn Parameters
int vSpeed = 110;        // Speed for straight motion
int turn_speed = 230;    // Speed during turns
int turn_delay = 10;     // Delay during turns

// Motor Pins
const int motorA1 = 8;
const int motorA2 = 10;
const int motorAspeed = 9;
const int motorB1 = 12;
const int motorB2 = 13;
const int motorBspeed = 11;

// Sensor Pins
const int left_sensor_pin = A0;
const int right_sensor_pin = A1;

// Sensor States
int left_sensor_state;
int right_sensor_state;

void setup() {
  // Configure motor pins
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

  // Start serial communication for debugging
  Serial.begin(9600);
  delay(3000);
}

void loop() {
  // Read sensor values
  left_sensor_state = analogRead(left_sensor_pin);
  right_sensor_state = analogRead(right_sensor_pin);

  if (right_sensor_state > 500 && left_sensor_state < 500) {
    // Turn Right
    Serial.println("Turning right");
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, HIGH);
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, HIGH);
    analogWrite(motorAspeed, vSpeed);
    analogWrite(motorBspeed, turn_speed);
  } else if (right_sensor_state < 500 && left_sensor_state > 500) {
    // Turn Left
    Serial.println("Turning left");
    digitalWrite(motorA1, HIGH);
    digitalWrite(motorA2, LOW);
    digitalWrite(motorB1, HIGH);
    digitalWrite(motorB2, LOW);
    analogWrite(motorAspeed, turn_speed);
    analogWrite(motorBspeed, vSpeed);
    delay(turn_delay);
  } else if (right_sensor_state > 500 && left_sensor_state > 500) {
    // Move Forward
    Serial.println("Going forward");
    digitalWrite(motorA1, HIGH);
    digitalWrite(motorA2, LOW);
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, HIGH);
    analogWrite(motorAspeed, vSpeed);
    analogWrite(motorBspeed, vSpeed);
    delay(turn_delay);
  } else {
    // Stop
    Serial.println("Stopping");
    analogWrite(motorAspeed, 0);
    analogWrite(motorBspeed, 0);
  }
}
