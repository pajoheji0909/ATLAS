//#include <ArduinoHardware.h>
#include <ros.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <std_msgs/UInt16.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Servo.h>
typedef enum pins {bt_tx = 2, bt_rx, IN1, IN2, IN3, IN4, mA_EN, sd1,sd2,sc,mB_EN  };
//                        2     ~3         ~5   ~6  sc not using    7    8     ~9   ~10  ~11  12   13
#define sonic1_tr A0
#define sonic1_ec A1
#define sonic2_tr A2
#define sonic2_ec A3

SoftwareSerial btSerial(bt_tx, bt_rx);
Servo servo_door1, servo_door2;

ros::NodeHandle  nh;
sensor_msgs::ChannelFloat32 sonics_msg;
ros::Publisher sonicArr("UltraSonic_Sensors Distance", &sonics_msg);

int btData[3];

//void sonicRead();
void btRead();
void motorControl();

void setup() {
  nh.initNode();
  nh.advertise(sonicArr);
  pinMode(sonic1_tr, OUTPUT);
  pinMode(sonic1_ec, INPUT);
  pinMode(sonic2_tr, OUTPUT);
  pinMode(sonic2_ec, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  digitalWrite(mA_EN, LOW);
  digitalWrite(mB_EN, LOW);
  servo_door1.attach(sd1);
  servo_door2.attach(sd2);
  btSerial.begin(9600);
}
/*
void sonicRead() {
  long duration = 0; float cm_one = 0; float cm_two = 0;
  digitalWrite(sonic1_tr, LOW);
  delayMicroseconds(5);
  digitalWrite(sonic1_tr, HIGH);
  delayMicroseconds(10);
  digitalWrite(sonic1_tr, LOW);
  duration = pulseIn(sonic1_ec, HIGH);
  cm_one = (duration/2) / 29.1;
  duration = 0;
  delayMicroseconds(5);
  digitalWrite(sonic2_tr, LOW);
  delayMicroseconds(5);
  digitalWrite(sonic2_tr, HIGH);
  delayMicroseconds(10);
  digitalWrite(sonic2_tr, LOW);
  duration = pulseIn(sonic1_ec, HIGH);
  cm_two = (duration/2) / 29.1;

  sonics_msg.name = "UltraSonicSensors";
  sonics_msg.values[0] = cm_one;
  sonics_msg.values[1] = cm_two;
  sonicArr.publish(&sonics_msg);
}
*/
void btRead() {
  if (btSerial.available()>3) {
    for (int i = 0; i < 3; i++) {
    btData[i] = btSerial.read();
    }
    digitalWrite(mA_EN, HIGH);
    digitalWrite(mB_EN, HIGH);
    delay(10);
  }
}

void motorControl() {
  int joyX = btData[1]; // since joystick value 0-1024
  int joyY = btData[2];
//  analogWrite(IN1, joyX);
  analogWrite(IN1, 180);
  digitalWrite(IN2, LOW);

//  analogWrite(IN3, joyY);
  analogWrite(IN3, 180);
  digitalWrite(IN4, LOW);
}

void loop() {
//  sonicRead();
  btRead();
  motorControl();
  nh.spinOnce();
  delay(100);
}
