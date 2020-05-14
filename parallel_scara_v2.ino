#include <Arduino.h>
//#include <math.h> //for Arduino
//#include "hardware\\tools\\avr\\avr\\include\\math.h" //for other windows
#include "hardware/tools/avr/avr/include/math.h" //for vscode
#include <Servo.h>

#define S1_PIN 3
#define S2_PIN 4
#define S3_PIN 2
#define DEBUG_LED_CALC 12
#define DEBUG_LED_LIFTED 11
#define DEBUG_LED_LIFTING 10
#define DEBUG_LED_PUTTING_DOWN 9
#define DEBUG_LED_LINE 8

#define S1_POS_X 0
#define S1_POS_Y 0

#define S2_POS_X 50
#define S2_POS_Y 0

#define S1_OFFSET 2.5
#define S2_OFFSET -7.5

#define ARM_LEN_1 75
#define ARM_LEN_2 100
#define ARM_LEN_3 35

#define LIFT_HEIGHT 120
#define BASE_HEIGHT 50
#define LIFTING_SPEED 10 //lower is faster

#define STEPS_PER_MM 1
#define CIRCLE_PRECISION 0.1
#define MS_PER_DEG 1.7

Servo servo1;
Servo servo2;
Servo servo3;

double actual_x = 0;
double actual_y = 0;
double actual_s1_deg = 0;
double actual_s2_deg = 0;
bool lifted = false;

void go_to(double, double);
void lift();
void lift_fast();
void put_down();
void put_down_fast();
void draw_line(double, double, double, double, bool = false);
void draw_circle(double, double, double);

void lift()
{
  digitalWrite(DEBUG_LED_LIFTING, HIGH);

  for(int i = BASE_HEIGHT+1; i <= LIFT_HEIGHT; i++)
  {
    servo3.write(180 - i);
    delay(LIFTING_SPEED);
  }
  delay(50);
  lifted = true;

  digitalWrite(DEBUG_LED_LIFTED, HIGH);
  digitalWrite(DEBUG_LED_LIFTING, LOW);
}

void lift_fast()
{
  digitalWrite(DEBUG_LED_LIFTING, HIGH);
  
  servo3.write(180 - LIFT_HEIGHT);
  delay(50);
  lifted = true;

  digitalWrite(DEBUG_LED_LIFTED, HIGH);
  digitalWrite(DEBUG_LED_LIFTING, LOW);
}

void put_down()
{
  digitalWrite(DEBUG_LED_PUTTING_DOWN, HIGH);

  for(int i = LIFT_HEIGHT-1; i >= BASE_HEIGHT; i--)
  {
    servo3.write(180 - i);
    delay(LIFTING_SPEED);
  }
  delay(100);
  lifted = false;

  digitalWrite(DEBUG_LED_LIFTED, LOW);
  digitalWrite(DEBUG_LED_PUTTING_DOWN, LOW);
}

void put_down_fast()
{
  digitalWrite(DEBUG_LED_PUTTING_DOWN, HIGH);

  servo3.write(180 - BASE_HEIGHT);
  delay(100);
  lifted = false;

  digitalWrite(DEBUG_LED_LIFTED, LOW);
  digitalWrite(DEBUG_LED_PUTTING_DOWN, LOW);
}

void draw_line(double x1, double y1, double x2, double y2, bool without_lifting)
{
  digitalWrite(DEBUG_LED_LINE, HIGH);
  go_to(x1, y1);

  if(lifted)
    put_down();
  
  double dx = x2 - x1;
  double dy = y2 - y1;  
  double c = round(STEPS_PER_MM * sqrt(dx*dx + dy*dy));
  
  for(int i = 0; i <= c; i++)
  {
    go_to(x1 + i*dx/c, y1 + i*dy/c);
  }
  
  digitalWrite(DEBUG_LED_LINE, LOW);

  if(!without_lifting)
    lift();
}

void draw_circle(double x, double y, double radius)
{
  go_to(x+cos(0)*radius, y+sin(0)*radius);
  
  if(lifted)
    put_down();
  
  for(double r = CIRCLE_PRECISION; r <= M_PI*2; r += CIRCLE_PRECISION)
  {
    draw_line(actual_x, actual_y, x+cos(r)*radius, y+sin(r)*radius, true);
  }
  
  lift();
}

inline double cosine_angle_rule(double a, double b, double c)
{
  // cosine rule for angle between c and a
  return acos((a * a + c * c - b * b) / (2 * a * c));
}

inline double cosine_side_rule(double A, double b, double c)
{
  return sqrt(b*b+c*c-2*b*c*cos(A));
}

inline double rad_to_deg(double rad)
{
  return rad * (180.0 / M_PI);
}

inline double pitagoras(double b, double c)
{
  return sqrt(b*b + c*c);
}

void go_to(double x, double y)
{
  if (actual_x == x && actual_y == y)
    return;

  digitalWrite(DEBUG_LED_CALC, HIGH);

  double L4 = cosine_side_rule(M_PI - M_PI/4.0, ARM_LEN_2, ARM_LEN_3);
  double epsilon = cosine_angle_rule(L4, ARM_LEN_2, ARM_LEN_3);
  double d3 = pitagoras(S2_POS_X - x, y - S2_POS_Y);
  double theta2 = atan2(y, (S2_POS_X - x)) + cosine_angle_rule(d3, L4, ARM_LEN_1);
  
  double x4 = S2_POS_X + ARM_LEN_1*cos(M_PI - theta2);
  double y4 = S2_POS_Y + ARM_LEN_1*sin(M_PI - theta2);
  
  double delta = atan2((x4-x), (y-y4));
  
  double x1 = x + ARM_LEN_3*sin(delta-epsilon);
  double y1 = y - ARM_LEN_3*cos(delta-epsilon);
  
  double d1 = pitagoras(x1 - S1_POS_X, y1 - S1_POS_Y);
  double theta1 = atan2((y1 - S1_POS_Y), (x1 - S1_POS_X)) + cosine_angle_rule(d1, ARM_LEN_2, ARM_LEN_1);

  double s1deg = rad_to_deg(theta1);
  double s2deg = rad_to_deg(M_PI - theta2);
   
  servo1.write(s1deg + S1_OFFSET);
  //Serial.print("servo1: ");
  //Serial.println(floor(s1deg));

  servo2.write(s2deg + S2_OFFSET);
  //Serial.print("servo2: ");
  //Serial.println(floor(s2deg));
  
  double max_dist = max(abs(actual_s1_deg - s1deg), abs(actual_s2_deg - s2deg));
  int delay_servo = ceil(MS_PER_DEG*ceil(max_dist));

  if (delay_servo < 10)
    delay_servo = 10;
  
  actual_x = x;
  actual_y = y;
  actual_s1_deg = s1deg;
  actual_s2_deg = s2deg;
  
  digitalWrite(DEBUG_LED_CALC, LOW);
  delay(delay_servo);
}

void setup()
{
  pinMode(DEBUG_LED_CALC, OUTPUT);
  pinMode(DEBUG_LED_LIFTED, OUTPUT);
  pinMode(DEBUG_LED_LIFTING, OUTPUT);
  pinMode(DEBUG_LED_PUTTING_DOWN, OUTPUT);
  pinMode(DEBUG_LED_LINE, OUTPUT);

  digitalWrite(DEBUG_LED_CALC, LOW);
  digitalWrite(DEBUG_LED_LIFTED, HIGH);
  digitalWrite(DEBUG_LED_LIFTING, LOW);
  digitalWrite(DEBUG_LED_PUTTING_DOWN, LOW);
  digitalWrite(DEBUG_LED_LINE, LOW);

  Serial.begin(9600);
  Serial.println("Hello world!");
  
  servo1.attach(S1_PIN);
  servo2.attach(S2_PIN);
  servo3.attach(S3_PIN);
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  
  /*servo1.write(150);
  servo2.write(30);
  servo3.write(LIFT_UP_HEIGHT);*/
  
  //lift_up();
  //go_to(25, 100);

  //servo1.write(90 + S1_OFFSET);
  //servo2.write(90 + S2_OFFSET);

  //delay(5000);

  //triangulo
  /*draw_line(-56, 166, -31, 142, true);
  draw_line(-31, 142, -64, 132, true);
  draw_line(-64, 132, -56, 166);

  draw_line(0, 170, 30, 170, true);
  draw_line(30, 170, 30, 140, true);
  draw_line(30, 140, 0, 140, true);
  draw_line(0, 140, 0, 170);


  draw_circle(80, 151, 16);

  draw_line(66.68, 142.14, 49.89, 124.55);
  draw_line(81.06, 135.04, 81.01, 126.25);
  draw_line(94.53, 144.3, 116.29, 132.85);
  draw_line(92.88, 160.49, 109.02, 171.58);
  draw_line(80.35, 167, 81.01, 183.69);
  draw_line(66.53, 159.63, 51.62, 170.89);

  delay(100);
  go_to(25, 100);*/


  // delay(1000);
  // draw_circle(80, 151, 16);
  // delay(1000);
  // draw_circle(80, 151, 16);
  // delay(1000);
  // draw_circle(80, 151, 16);
  // delay(1000);

  delay(3000);
  lift();
  delay(3000);
  put_down();
  
}

void loop()
{
  
  /*
  for (pos = 0; pos <= 90; pos += 10) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo1.write(pos);              // tell servo to go to position in variable 'pos'
    Serial.println(pos);
    delay(500);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 90; pos >= 0; pos -= 10) { // goes from 180 degrees to 0 degrees
    servo1.write(pos);              // tell servo to go to position in variable 'pos'
    Serial.println(pos);
    delay(500);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 0; pos <= 90; pos += 10) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo2.write(pos);              // tell servo to go to position in variable 'pos'
    Serial.println(pos);
    delay(500);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 90; pos >= 0; pos -= 10) { // goes from 180 degrees to 0 degrees
    servo2.write(pos);              // tell servo to go to position in variable 'pos'
    Serial.println(pos);
    delay(500);                       // waits 15ms for the servo to reach the position
  }*/

  /*servo1.write(90);
  servo2.write(90);
  delay(1000);
  servo1.write(0);
  servo2.write(0);
  delay(1000);*/
  
}
