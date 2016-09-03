#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <TinyGPS.h>

typedef enum pin_num {IN1 = 4, IN2, IN3, IN4, mA_EN, sd1, sd2, sc, gpsRx, gpsTx};
//                         4    5    6    7     8     9   10   11  12      13

Servo servo_door1, servo_door2;

ros::NodeHandle  nh;
std_msgs::Float32 web_x;
std_msgs::Float32 web_y;
std_msgs::Float32 door;
void messageX (const std_msgs::Float32 &toggle_msgs) {
  web_x.data = toggle_msgs.data;
}
void messageY (const std_msgs::Float32 &toggle_msgs) {
  web_x.data = toggle_msgs.data;
}
void messageDoor (const std_msgs::Float32 &toggle_msgs) {
  door.data = toggle_msgs.data;
}
ros::Subscriber<std_msgs::Float32> Controller_x ("server_send_x : ", &messageX);
ros::Subscriber<std_msgs::Float32> Controller_y ("server_send_y : ", &messageY);
ros::Subscriber<std_msgs::Float32> Controller_door("server_send_door : ",&messageDoor);

TinyGPS gpsGY;
SoftwareSerial gpss (gpsRx, gpsTx);

void readGPS ();
void motorControl();
void doorControl();

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.subscribe(Controller_x);
  nh.subscribe(Controller_y);
  nh.subscribe(Controller_door);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  digitalWrite(mA_EN, LOW);
  servo_door1.attach(sd1);
  servo_door2.attach(sd2);
  gpss.begin(4800);
}

void readGPS() {
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;
  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (gpss.available())
    {
      char c = gpss.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gpsGY.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }
  if (newData)
  {
    float flat, flon;
    unsigned long age;
  }
  if (chars == 0)
    Serial.println("** No characters received from GPS: check wiring **");
}
void motorControl() {
  int joyX = web_x.data;// since joystick value 0-1024
  int joyY = web_y.data;
  analogWrite(IN1, joyX);
  digitalWrite(IN2, LOW);

  analogWrite(IN3, joyY);
  digitalWrite(IN4, LOW);
}
void doorControl() {
  if (door.data == 1) {
    servo_door1.write(60);
    servo_door2.write(60);
  }
  else if (door.data == 0) {
    servo_door1.write(0);
    servo_door2.write(0);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  motorControl();
  doorControl();
  readGPS();
  nh.spinOnce();

}
