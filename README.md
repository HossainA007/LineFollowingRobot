
# Line Follower Robot using Arduino

This repository contains the implementation details, circuit design, and source code for a **Line Follower Robot** built using Arduino. This beginner-friendly robotics project demonstrates the concept of light-based line detection and autonomous movement.

---

## Overview

A Line Follower Robot is designed to follow a predefined path (either a black or white line). The project utilizes IR sensors to detect the line and an Arduino microcontroller to control the robot’s movements. It is divided into three main sections:
1. **Sensor Section**: Detects the line using IR sensors.
2. **Control Section**: Processes sensor data with an Arduino microcontroller.
3. **Driver Section**: Drives the motors using a motor driver IC.

---

## Features
- Detects and follows a black or white line using IR sensors.
- Automatically turns left or right based on line detection.
- Stops when both sensors detect a black surface.
- Operates using an Arduino Pro Mini and an L293D motor driver.

---

## Components
1. **Arduino Pro Mini**: Acts as the control unit to process sensor inputs and drive motors.
2. **L293D Motor Driver IC**: Provides sufficient voltage and current to the motors.
3. **IR Sensors**: Detects the line (black or white) and sends feedback to the Arduino.
4. **DC Motors**: Drives the robot’s movement.
5. **Power Supply**: Supplies power to the Arduino, motor driver, and sensors.

---

## Working Principle
- **Light Behavior**: 
  - White surfaces reflect IR light, while black surfaces absorb it.
  - The IR sensor detects these reflections and sends signals to the Arduino.
- **Logic**:
  - When both sensors detect a white surface, the robot moves forward.
  - When the left sensor detects a black line, the robot turns left.
  - When the right sensor detects a black line, the robot turns right.
  - When both sensors detect a black surface, the robot stops.

---

## Circuit Explanation
- **Sensor Section**:
  - IR sensors with comparators (LM358) generate digital signals based on light reflection.
- **Control Section**:
  - The Arduino processes sensor inputs and sends commands to the motor driver.
- **Driver Section**:
  - The L293D motor driver controls two DC motors based on Arduino signals.

---

## Arduino Code

The project includes complete Arduino code to operate the robot. Below is a snippet of the code:

```c
/* ------ Arduino Line Follower Code ----- */
#define LS 2 // Left sensor
#define RS 3 // Right sensor

#define LM1 4 // Left motor forward
#define LM2 5 // Left motor backward
#define RM1 6 // Right motor forward
#define RM2 7 // Right motor backward

void setup() {
  pinMode(LS, INPUT);
  pinMode(RS, INPUT);
  pinMode(LM1, OUTPUT);
  pinMode(LM2, OUTPUT);
  pinMode(RM1, OUTPUT);
  pinMode(RM2, OUTPUT);
}

void loop() {
  if (digitalRead(LS) && digitalRead(RS)) { // Move Forward
    digitalWrite(LM1, HIGH);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, HIGH);
    digitalWrite(RM2, LOW);
  }
  if (!digitalRead(LS) && digitalRead(RS)) { // Turn Right
    digitalWrite(LM1, LOW);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, HIGH);
    digitalWrite(RM2, LOW);
  }
  if (digitalRead(LS) && !digitalRead(RS)) { // Turn Left
    digitalWrite(LM1, HIGH);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, LOW);
    digitalWrite(RM2, LOW);
  }
  if (!digitalRead(LS) && !digitalRead(RS)) { // Stop
    digitalWrite(LM1, LOW);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, LOW);
    digitalWrite(RM2, LOW);
  }
}
```

The complete code is available in the repository for download.

---

## Applications
- Autonomous robots for warehouses or factories.
- Educational projects for learning robotics.
- Prototypes for advanced line-following systems.

---

## Repository Contents
- **`/Arduino_Code`**: Contains the complete Arduino sketch for the robot.
- **`/Circuit_Diagram`**: Circuit schematic for wiring the components.
- **`/Documentation`**: Detailed explanation of the project, its working principle, and applications.

---

## How to Build
1. **Hardware Setup**:
   - Assemble the circuit as per the provided schematic.
   - Connect IR sensors, Arduino, motor driver, and DC motors.
2. **Upload Code**:
   - Upload the Arduino code using the Arduino IDE.
3. **Run the Robot**:
   - Power the robot and place it on a path with black or white lines.
   - Observe the robot follow the path autonomously.

---
