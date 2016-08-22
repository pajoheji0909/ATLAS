//#include <Arduino.h>
#include <SoftwareSerial.h>
/*
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Float32.h>
*/
#define bluetoothTX 2
#define bluetoothRX 3
#define Vx A0
#define Vy A1
#define sw 7

int sw_press = 0;
int x = 0;
int y = 0;
/*
ros::NodeHandle  nh;
std_msgs::Float32 joystickV;
ros::Publisher bt_pub("controller_xy", &joystickV);
*/
SoftwareSerial btSerial(bluetoothTX, bluetoothRX);

float btData;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  btSerial.begin(9600);
  pinMode(sw, INPUT);
  pinMode(Vx, INPUT);
  pinMode(Vy, INPUT);
  digitalWrite(sw, HIGH);
  /*
  nh.initNode();
  nh.advertise(bt_pub);
  */
}

void readJoystick() {
  sw_press = digitalRead(sw);//reads whether joystick has been pressed down (selected) or not
  x= analogRead(Vx);//reads the X-coordinate value
  y= analogRead(Vy);//reads the Y-coordinate value
  btData = x+0.0001*y; // 1234.5678 -> 1234 x, 5678 y value
  joystickV.data = btData;
}

void btSend() {
  if (btSerial.available()) {
    btSerial.print(btData);
    btSerial.println(" is input");
  }
  else
    btSerial.println("Bluetooth Communitcation is Unstable");
}

void loop() {
  // put your main code here, to run repeatedly:
  readJoystick();
  Serial.print("X: ");
  Serial.println(x);//prints the X-coordinate
  Serial.print("Y: ");
  Serial.println(y);//prints the Y-coordinate
  Serial.print (" Pressed: ");
  Serial.println(sw_press);//prints whether joystick knob has been pressed or not
  btSend();
  /*
  bt_pub.publish(&joystickV);
  nh.spinOnce();
  */
  delay(100);
}

