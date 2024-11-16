Line Follower Robot using Arduino


Arduino Line Follower Robot Project with Code and Circuit Diagram
Line follower Robot is a very simple robot that follows a line, either a black line or a white line. These type of robots are very simple to build and is often the first choice for beginners who are getting started with robotics. Basically, there are two types of line follower robots: one is a black line follower which follows the black line and the second is a white line follower which follows the white line. Line follower actually senses the line and follows it. Though the idea sounds simple, with a little more development, robots similar to this are practically used in many applications like factory floor management robots or warehouse robots. 
Concepts of Line Follower
The concept of working of line follower is related to light. We use here the behavior of light at the black and white surfaces. When light falls on a white surface it is almost fully reflected and in the case of a black surface light is completely absorbed. This behavior of light is used in building a line follower robot.
 
 
In this Arduino based line follower robot, we have used IR Transmitters and IR receivers also called photodiodes. They are used for sending and receiving light. IR transmits infrared lights. When infrared rays falls on the white surface, it’s reflected back and caught by photodiodes which generate some voltage changes. When IR light falls on a black surface, light is absorbed by the black surface and no rays are reflected back, thus photo diode does not receive any light or rays. Here in this Arduino line follower robot when the sensor senses white surface then Arduino gets 1 as input and when senses black line Arduino gets 0 as input.
Since the Line follower robot is an interesting beginners project, we have also built it using different development boards other than Arduino, you can also check them out using the below link if interested 
	Line Follower Robot using 8051 Microcontroller 
	Line Follower Robot using Raspberry Pi
	Texas MSP430 Launchpad based Line Follower
	Simple Line Follower using PIC Microcontroller
	Line Follower using ATmega16 AVR Microcontroller 
Circuit Explanation
The whole Arduino line follower robot can be divided into 3 sections: sensor section, a control section, and driver section.
Sensor section:
This section contains IR diodes, potentiometer, Comparator (Op-Amp) and LED’s. The potentiometer is used for setting reference voltage at comparator’s one terminal and IR sensors are used to sense the line and provide a change in voltage at the comparator’s second terminal. Then the comparator compares both voltages and generates a digital signal at the output. Here in this line follower circuit, we have used two comparators for two sensors. LM 358 is used as a comparator. LM358 has inbuilt two low noise Op-amps.
Control Section:
Arduino Pro Mini is used for controlling the whole the process of the line follower robot. The outputs of comparators are connected to digital pin numbers 2 and 3 of Arduino. Arduino read these signals and send commands to driver circuit to driveline follower. 
Driver section:
The driver section consists of motor driver and two DC motors. The motor driver is used for driving motors because Arduino does not supply enough voltage and current to the motor. So we add a motor driver circuit to get enough voltage and current for the motor. Arduino sends commands to this motor driver and then it drives motors.
Working of Line Follower Robot using Arduino
Building a Line follower robot using Arduino is interesting. The line follower robot senses a black line by using a sensor and then sends the signal to Arduino. Then Arduino drives the motor according to sensors' output.
 
Here in this project, we are using two IR sensor modules namely the left sensor and the right sensor. When both left and right sensor senses white then the robot moves forward.
 
If the left sensor comes on a black line then the robot turn the left side.
 
If the right sensor sense black line then robot turn right side until both sensors comes at the white surface. When the white surface comes robot starts moving on forward again.
 
If both sensors come on the black line, the robot stops.
 
Circuit Diagram
 
The complete circuit diagram for arduino line follower robot is shown in the above image. As you can see the output of comparators is directly connected to Arduino digital pin number 2 and 3. And motor driver’s input pin 2, 7, 10 and 15 is connected to Arduino's digital pin number 4, 5, 6 and 7 respectively. And one motor is connected at the output pin of motor drivers 3 and 6 and another motor is connected at pin 11 and 14. 
Program Explanation
In the program, first of all, we defined input and output pin, and then in loop, we check inputs and sends output according to inputs to the output pin for the driving motor. For checking the input pin we used “if” statements. The complete line follower robot code can be found at the bottom of this page. 
 
 
There are four conditions in this line following robot that we read by using Arduino. We have used two sensors namely the left sensor and the right sensor.
Input	Output	Movement Of Robot
Left Sensor	Right Sensor	Left Motor	Right Motor	
LS	RS	LM1	LM2	RM1	RM2	
0	0	0	0	0	0	Stop
0	1	1	0	0	0	Turn Right
1	0	0	0	1	0	Turn Left
1	1	1	0	1	0	Forward
We write the arduino line follower code according to the conditions shown in table above.
Required Components
Arduino
In our Project, we have used a microcontroller to control whole the process of system that is ARDUINO. Arduino is an open-source hardware and very useful for project developments. There are many types of arduino like Arduino UNO, arduino mega, arduino pro mini, Lilypad etc. available in the market. Here we have used arduino pro mini in this project as arduino pro mini is small and so breadboard compatible. To burn the line follower robot arduino code we have used an FTDI burner.
 
L293D Motor Driver
L293D is a motor driver IC which has two channels for driving two motors. L293D has two inbuilt Transistor Darlington pair for current amplification and a separate power supply pin for giving external supply to motors.
 
IR Module:
IR Module is sensor circuit that consists IR LED/photodiode pair, potentiometer, LM358, resistors and LED. IR sensor transmits Infrared light and photodiode receives the infrared light.
 
Power Supply  
I have added a voltage regulator to get 5 volts for Arduino, comparator and motor driver. And a 9-volt battery is used to power the circuit.
Code
/*------ Arduino Line Follower Code----- */
/*-------defining Inputs------*/
#define LS 2      // left sensor
#define RS 3      // right sensor
/*-------defining Outputs------*/
#define LM1 4       // left motor
#define LM2 5       // left motor
#define RM1 6       // right motor
#define RM2 7       // right motor
void setup()
{
  pinMode(LS, INPUT);
  pinMode(RS, INPUT);
  pinMode(LM1, OUTPUT);
  pinMode(LM2, OUTPUT);
  pinMode(RM1, OUTPUT);
  pinMode(RM2, OUTPUT);
}
void loop()
{
  if(digitalRead(LS) && digitalRead(RS))     // Move Forward
  {
    digitalWrite(LM1, HIGH);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, HIGH);
    digitalWrite(RM2, LOW);
  }
  
  if(!(digitalRead(LS)) && digitalRead(RS))     // Turn right
  {
    digitalWrite(LM1, LOW);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, HIGH);
    digitalWrite(RM2, LOW);
  }
  
  if(digitalRead(LS) && !(digitalRead(RS)))     // turn left
  {
    digitalWrite(LM1, HIGH);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, LOW);
    digitalWrite(RM2, LOW);
  }
  
  if(!(digitalRead(LS)) && !(digitalRead(RS)))     // stop
  {
    digitalWrite(LM1, LOW);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, LOW);
    digitalWrite(RM2, LOW);
  }
}


  int vSpeed = 110;        // MAX 255
  int turn_speed = 230;    // MAX 255 
  int turn_delay = 10;
  
//L293 Connection   
  const int motorA1      = 8;  
  const int motorA2      = 10; 
  const int motorAspeed  = 9;
  const int motorB1      = 12; 
  const int motorB2      = 13; 
  const int motorBspeed  = 11;

//Sensor Connection
  const int left_sensor_pin =A0;
  const int right_sensor_pin =A1;

  
  
  int left_sensor_state;
  int right_sensor_state;

void setup() {
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

  Serial.begin(9600);

  delay(3000);
  
}

void loop() {


left_sensor_state = analogRead(left_sensor_pin);
right_sensor_state = analogRead(right_sensor_pin);

if(right_sensor_state > 500 && left_sensor_state < 500)
{
  Serial.println("turning right");

  digitalWrite (motorA1,LOW);
  digitalWrite(motorA2,HIGH);                       
  digitalWrite (motorB1,LOW);
  digitalWrite(motorB2,HIGH);

  analogWrite (motorAspeed, vSpeed);
  analogWrite (motorBspeed, turn_speed);
  
  }
if(right_sensor_state < 500 && left_sensor_state > 500)
{
  Serial.println("turning left");
  
  digitalWrite (motorA1,HIGH);
  digitalWrite(motorA2,LOW);                       
  digitalWrite (motorB1,HIGH);
  digitalWrite(motorB2,LOW);

  analogWrite (motorAspeed, turn_speed);
  analogWrite (motorBspeed, vSpeed);

  delay(turn_delay);
  }

if(right_sensor_state > 500 && left_sensor_state > 500)
{
  Serial.println("going forward");

  digitalWrite (motorA2,LOW);
  digitalWrite(motorA1,HIGH);                       
  digitalWrite (motorB2,HIGH);
  digitalWrite(motorB1,LOW);

  analogWrite (motorAspeed, vSpeed);
  analogWrite (motorBspeed, vSpeed);

  delay(turn_delay);
  
  }

if(right_sensor_state < 500 && left_sensor_state < 500)
{ 
  Serial.println("stop");
  
  analogWrite (motorAspeed, 0);
  analogWrite (motorBspeed, 0);
  
  }

 
}


 
Concepts of Line Follower
Concept of working of line follower is related to light. We use here the behavior of light at black and white surface. When light fall on a white surface it is almost full reflected and in case of black surface light is completely absorbed. This behavior of light is used in building a line follower robot.
 
In this arduino based line follower robot we have used IR Transmitters and IR receivers also called photo diodes. They are used for sending and receiving light. IR transmits infrared lights. When infrared rays falls on white surface, it’s reflected back and catched by photodiodes which generates some voltage changes. When IR light falls on a black surface, light is absorb by the black surface and no rays are reflected back, thus photo diode does not receive any light or rays.
Here in this arduino line follower robot when sensor senses white surface then arduino gets 1 as input and when senses black line arduino gets 0 as input.
Circuit Explanation
The whole arduino line follower robot can be divided into 3 sections: sensor section, control section and driver section.
Sensor section:
This section contains IR diodes, potentiometer, Comparator (Op-Amp) and LED’s. Potentiometer is used for setting reference voltage at comparator’s one terminal and IR sensors are used to sense the line and provide a change in voltage at comparator’s second terminal. Then comparator compares both voltages and generates a digital signal at output. Here in this line follower circuit we have used two comparator for two sensors. LM 358 is used as comparator. LM358 has inbuilt two low noise Op-amps.
Working of Line Follower Robot using Arduino
Working of line follower is very interesting. Line follower robot senses black line by using sensor and then sends the signal to arduino. Then arduino drives the motor according to sensors' output.
 
Here in this project we are using two IR sensor modules namely left sensor and right sensor. When both left and right sensor senses white then robot move forward.
 
If left sensor comes on black line then robot turn left side.
 
If right sensor sense black line then robot turn right side until both sensor comes at white surface. When white surface comes robot starts moving on forward again.
 
If both sensors comes on black line, robot stops.
