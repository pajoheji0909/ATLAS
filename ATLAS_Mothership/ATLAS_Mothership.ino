//#include <ArduinoHardware.h>
#include <ros.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <SoftwareSerial.h>
#include <Servo.h>

typedef enum pins {bt_tx = 2, bt_rx, sonic1_ec, sonic1_tr, sonic2_tr, sonic2_ec, mA_EN, IN1, IN2, IN3, mB_EN, IN4 };
//                        2     ~3       4           ~5           ~6         7    8     ~9   ~10  ~11  12   13

SoftwareSerial btSerial(bt_tx, bt_rx);

ros::NodeHandle  nh;
sensor_msgs::ChannelFloat32 sonics_msg;
ros::Publisher sonicArr("UltraSonic_Sensors Distance", &sonics_msg);
byte btData;

void sonicRead();
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
  btSerial.begin(9600);
}

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

void btRead() {
  if (btSerial.available()) {
    btData = btSerial.read();
    digitalWrite(mA_EN, HIGH);
    digitalWrite(mB_EN, HIGH);
    delay(10);
  }
}

void motorControl() {
  int joyX = btData / 4; // since joystick value 0-1024
  int joyY = 10000*(btData - (int)btData) / 4;
  analogWrite(IN1, joyX);
  digitalWrite(IN2, LOW);

  analogWrite(IN3, joyY);
  digitalWrite(IN4, LOW);
}

void loop() {
  sonicRead();
  btRead();
  motorControl();
  nh.spinOnce();
  delay(100);
}
