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

byte btData[3];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  btSerial.begin(9600);
  pinMode(sw, INPUT);
  pinMode(Vx, INPUT);
  pinMode(Vy, INPUT);
  pinMode(9, INPUT);
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
  int tmp =0;
  if (sw_press == 1) tmp = 1 ; 
    else tmp = -1;    
  btData[0] = sw_press;
  btData[1] = x;
  btData[2] = y;
  //joystickV.data = btData;
}

void btSend(int n) {
  if (btSerial.available()) {
    btSerial.write(btData[n]);
  }
  else
    btSerial.println("Bluetooth Communitcation is Unstable");
  delay(50);
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
  Serial.print("Yellow Switch ");
  int tmp  = digitalRead(9);
  Serial.println(tmp);
  delay(300);
  for (int i = 0; i < 3 ; i ++) {
  //btSend(i);
  }
  /*
  bt_pub.publish(&joystickV);
  nh.spinOnce();
  */
}

