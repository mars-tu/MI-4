/*
   Arduino RC Receiver - MI-4
   by MARS Robotics Society
   Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#define enA 9  // Arduino pin D9 - CH6 on PCB board - PWM output
#define in1 8  // D8 - CH5
#define in2 7  // D7 - CH4
#define in3 6  // D6 - CH3 
#define in4 4  // D4 - CH1 
#define enB 5  // D5 - CH2 - PWM output

Servo sCLAW;
Servo sJOINT;
int sCLAW, sJOINT;
RF24 radio(3, 2);   // nRF24L01 (CE, CSN)
const byte address[6] = "00001";
unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;
// Max size of this struct is 32 bytes
struct Data_Package {
  byte j1PotX;
  byte j1PotY;
  byte j1Button;
  byte j2PotX;
  byte j2PotY;
  byte j2Button;
  byte pot1;
  byte pot2;
  byte tSwitch1;
  byte tSwitch2;
  byte button1;
  byte button2;
  byte button3;
  byte button4;
};
Data_Package data; //Creating a variable with the above structure
int  steering, throttle;
int motorSpeedA = 0;
int motorSpeedB = 0;
void setup() {
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  //Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening(); //  Set the module as receiver
  resetData();
  servo1.attach(4); // D4 - CH1
  servo2.attach(5); // D5 - CH2
}
void loop() {
  // Check whether we keep receving data, or we have a connection between the two modules
  currentTime = millis();
  if ( currentTime - lastReceiveTime > 1000 ) { // If current time is more then 1 second since we have recived the last data, that means we have lost connection
    resetData(); // If connection is lost, reset the data. It prevents unwanted behavior, for example if a drone jas a throttle up, if we lose connection it can keep flying away if we dont reset the function
  }
  // Check whether there is data to be received
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package)); // Read the whole data and store it into the 'data' structure
    lastReceiveTime = millis(); // At this moment we have received the data
  }
  
  // Parse the data from the Joystic 1 to the throttle and steering variables
  throttle = data.j1PotY;
  steering = data.j1PotX;
  
  // Throttle used for forward and backward control
  // Joystick values: 0 to 255; down = 0; middle = 127; up = 255
  if (throttle < 110) {
    // Set Motor A backward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    // Set Motor B backward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    // Convert the declining throttle readings for going backward from 110 to 0 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedA = map(throttle, 110, 0, 0, 255);
    motorSpeedB = map(throttle, 110, 0, 0, 255);
  }
  else if (throttle > 140) {
    // Set Motor A forward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    // Set Motor B forward
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    // Convert the increasing throttle readings for going forward from 140 to 255 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedA = map(throttle, 140, 255, 0, 255);
    motorSpeedB = map(throttle, 140, 255, 0, 255);
  }
  // If joystick stays in middle the motors are not moving
  else {
    motorSpeedA = 0;
    motorSpeedB = 0;
  }
  // Steering used for left and right control
  if (steering < 110) {
    // Convert the declining steering readings from 140 to 255 into increasing 0 to 255 value
    int xMapped = map(steering, 110, 0, 0, 255);
    // Move to left - decrease left motor speed, increase right motor speed
    motorSpeedA = motorSpeedA - xMapped;
    motorSpeedB = motorSpeedB + xMapped;
    // Confine the range from 0 to 255
    if (motorSpeedA < 0) {
      motorSpeedA = 0;
    }
    if (motorSpeedB > 255) {
      motorSpeedB = 255;
    }
  }
  if (steering > 140) {
    // Convert the increasing steering readings from 110 to 0 into 0 to 255 value
    int xMapped = map(steering, 140, 255, 0, 255);
    // Moving right
    motorSpeedA = motorSpeedA + xMapped;
    motorSpeedB = motorSpeedB - xMapped;
    // Confine the range from 0 to 255
    if (motorSpeedA > 255) {
      motorSpeedA = 255;
    }
    if (motorSpeedB < 0) {
      motorSpeedB = 0;
    }
  }
  // Prevent buzzing at low speeds (We can adjust according to the Result)
  if (motorSpeedA < 70) {
    motorSpeedA = 0;
  }
  if (motorSpeedB < 70) {
    motorSpeedB = 0;
  }

  
  // Controlling servos for the Arm of the robot
  lift = data.j2PotX;
  claw = data.j2PotY;
  // Joystick values: 0 to 255; down = 0; middle = 127; up = 255
  if (lift < 110) {
    servo1Value = map(data.j2PotX, 0, 255, 0, 180);
  }
  
  servo1Value = map(data.j2PotX, 0, 255, 0, 180); // Map the receiving value form 0 to 255 to 0 to 180(degrees), values used for controlling servos
  servo2Value = map(data.j2PotY, 0, 255, 0, 180);
  
  servo1.write(servo1Value); // Send PWM signal to joint servo motor
  servo2.write(servo2Value); // Send PWM signal to claw servo motor
  analogWrite(enA, motorSpeedA); // Send PWM signal to motor A
  analogWrite(enB, motorSpeedB); // Send PWM signal to motor B
  
}
void resetData() {
  // Reset the values when there is no radio connection - Set initial default values
  data.j1PotX = 127;
  data.j1PotY = 127;
  data.j2PotX = 127;
  data.j2PotY = 127;
  data.j1Button = 1;
  data.j2Button = 1;
  data.pot1 = 1;
  data.pot2 = 1;
  data.tSwitch1 = 1;
  data.tSwitch2 = 1;
  data.button1 = 1;
  data.button2 = 1;
  data.button3 = 1;
  data.button4 = 1;
}
