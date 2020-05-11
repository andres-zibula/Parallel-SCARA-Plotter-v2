#include <Arduino.h>
//#include <math.h>
#include "hardware\\tools\\avr\\avr\\include\\math.h"
#include <Servo.h>

#define S1_PIN 3
#define S2_PIN 4
#define S3_PIN 5

#define S1_POS_X 0
#define S1_POS_Y 0

#define S2_POS_X 50
#define S2_POS_Y 0

#define S1_OFFSET 2.5
#define S2_OFFSET -7.5

#define L1 75
#define L2 100
#define L3 35

#define LIFT_UP_HEIGHT 27
#define BASE_HEIGHT 10
#define LIFTING_SPEED 20 //lower is faster

#define STEPS_PER_MM 1
#define CIRCLE_PRECISION 0.2
#define BEZIER_SEGMENTS 100


/////////////////////////////
/*
struct vec2 {
    double x, y;
    vec2(double x, double y) : x(x), y(y) {}
};

vec2 operator + (vec2 a, vec2 b) {
    return vec2(a.x + b.x, a.y + b.y);
}

vec2 operator - (vec2 a, vec2 b) {
    return vec2(a.x - b.x, a.y - b.y);
}

vec2 operator * (double s, vec2 a) {
    return vec2(s * a.x, s * a.y);
}



vec2 CalculateBezierPoint(double t, vec2 p0, vec2 p1, vec2 p2, vec2 p3)
{
  double u = 1 - t;
  double tt = t*t;
  double uu = u*u;
  double uuu = uu * u;
  double ttt = tt * t;
 
  vec2 p = uuu * p0; //first term
  p = p + 3 * uu * t * p1; //second term
  p = p + 3 * u * tt * p2; //third term
  p = p + ttt * p3; //fourth term
 
  return p;
}

void drawBezierCurve(vec2 p0, vec2 p1, vec2 p2, vec2 p3)
{
  vec2 q0 = CalculateBezierPoint(0, p0, p1, p2, p3);
  vec2 q1(0, 0);
  double t = 0;
  
  for(int i = 1; i <= BEZIER_SEGMENTS; i++)
  {
    t = i / (double) BEZIER_SEGMENTS;
    q1 = CalculateBezierPoint(t, p0, p1, p2, p3);
    draw_line(q0.x, q0.y, q1.x, q1.y, true);
    q0 = q1;
  }
}
*/
/////////////////////////////

Servo servo1;
Servo servo2;
Servo servo3;

double actual_x = 0;
double actual_y = 0;
bool is_up = false;

void test();
void go_to(double, double);
void lift_up();
void put_down();

int pos = 0;

void lift_up()
{
  for(int i = BASE_HEIGHT+1; i <= LIFT_UP_HEIGHT; i++)
  {
    servo3.write(i);
    delay(LIFTING_SPEED);
  }
  delay(50);
  is_up = true;
}

void lift_up_fast()
{
  servo3.write(LIFT_UP_HEIGHT);
  delay(50);
  is_up = true;
}

void put_down()
{
   for(int i = LIFT_UP_HEIGHT-1; i >= BASE_HEIGHT; i--)
  {
    servo3.write(i);
    delay(LIFTING_SPEED);
  }
  delay(50);
  is_up = false;
}

void put_down_fast()
{
  servo3.write(BASE_HEIGHT);
  delay(50);
  is_up = true;
}

void draw_line(double x1, double y1, double x2, double y2, bool without_lifting_up = false)
{
  bool delayed = false;
  
  if(sqrt((x1-actual_x)*(x1-actual_x)+(y1-actual_y)*(y1-actual_y)) > 10)
    delayed = true;
    
  go_to(x1, y1);

  if(delayed)
    delay(100);
  
  if(is_up)
    put_down();
  
  double dx = x2 - x1;
  double dy = y2 - y1;  
  double c = round(STEPS_PER_MM * sqrt(dx*dx + dy*dy)); //quiza floor
  
  for(int i = 0; i <= c; i++) //puede empezar desde 1 si ya se encuentra en el lugar
  {
    go_to(x1 + i*dx/c, y1 + i*dy/c);
  }
  
  if(!without_lifting_up)
    lift_up();
}

void draw_circle(double x, double y, double radius)
{
  go_to(x+cos(0)*radius, y+sin(0)*radius);
  delay(100);
  
  if(is_up)
    put_down();
  
  for(double r = CIRCLE_PRECISION; r <= M_PI*2; r += CIRCLE_PRECISION)
  {
    draw_line(actual_x, actual_y, x+cos(r)*radius, y+sin(r)*radius, true);
  }
  
  lift_up();
  delay(50);
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
  
  double L4 = cosine_side_rule(M_PI - M_PI/4.0, L2, L3);
  double epsilon = cosine_angle_rule(L4, L2, L3);
  double d3 = pitagoras(S2_POS_X - x, y - S2_POS_Y);
  double theta2 = atan2(y, (S2_POS_X - x)) + cosine_angle_rule(d3, L4, L1);

   /*Serial.print("M_PI: ");
   Serial.println(M_PI);
   Serial.print("atan2: ");
   Serial.println(atan2(y, (S2_POS_X - x)));
   Serial.print("cosine: ");
   Serial.println(cosine_angle_rule(d3, L4, L1));

   Serial.print("theta2: ");
   Serial.println(M_PI - (atan2(y, (S2_POS_X - x)) + cosine_angle_rule(d3, L4, L1)));*/
  
  /*servo2.write(floor(rad_to_deg(M_PI - theta2)) + S2_OFFSET);
  Serial.print("servo2: ");
  Serial.println(floor(rad_to_deg(M_PI - theta2)));
  //delay(10);*/

  /*Serial.print("M_PI: ");
   Serial.println(M_PI);
  Serial.print("theta2: ");
   Serial.println(theta2);
  Serial.print("M_PI - theta2: ");
  Serial.println(M_PI - theta2);*/
  
  double x4 = S2_POS_X + L1*cos(M_PI - theta2);
  double y4 = S2_POS_Y + L1*sin(M_PI - theta2);

  /*Serial.print("x4: ");
   Serial.println(x4);
  Serial.print("y4: ");
   Serial.println(y4);*/
  
  double delta = atan2((x4-x), (y-y4));
  
  /*Serial.print("delta: ");
   Serial.println(rad_to_deg(delta));

   Serial.print("epsilon: ");
   Serial.println(rad_to_deg(epsilon));

   Serial.print("delta-epsilon: ");
   Serial.println(rad_to_deg(delta-epsilon));*/
  
  double x1 = x + L3*sin(delta-epsilon);
  double y1 = y - L3*cos(delta-epsilon);

  /*Serial.print("x1: ");
   Serial.println(x1);

   Serial.print("y1: ");
   Serial.println(y1);*/
  
  double d1 = pitagoras(x1 - S1_POS_X, y1 - S1_POS_Y);

  /*Serial.print("d1: ");
   Serial.println(d1);*/
  
  double theta1 = atan2((y1 - S1_POS_Y), (x1 - S1_POS_X)) + cosine_angle_rule(d1, L2, L1);

  /*Serial.print("atan2: ");
   Serial.println(rad_to_deg(atan2((y1 - S1_POS_Y), (x1 - S1_POS_X))));

   Serial.print("cosine: ");
   Serial.println(rad_to_deg(cosine_angle_rule(d1, L2, L1)));

   Serial.print("theta1: ");
   Serial.println(rad_to_deg(theta1));*/

   

    servo2.write((rad_to_deg(M_PI - theta2)) + S2_OFFSET);
  Serial.print("servo2: ");
  Serial.println(floor(rad_to_deg(M_PI - theta2)));
  
  
  servo1.write((rad_to_deg(theta1)) + S1_OFFSET);
  Serial.print("servo1: ");
  Serial.println(floor(rad_to_deg(theta1)));
  
  
  actual_x = x;
  actual_y = y;
  
  //delay(10);
}

void setup()
{
  Serial.begin(9600);
  Serial.println("Hello world!");
  
  servo1.attach(S1_PIN);
  servo2.attach(S2_PIN);
  //servo3.attach(S3_PIN);
  
  /*servo1.write(150);
  servo2.write(30);
  servo3.write(LIFT_UP_HEIGHT);*/
  
  //lift_up();
  //go_to(25, 100);

  servo1.write(90 + S1_OFFSET);
  servo2.write(90 + S2_OFFSET);

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
  
}

void loop()
{/*
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

void test()
{
  /*delay(1000);
  go_to(30, 150);
  delay(1000);
  go_to(30, 200);*/

  /*lift_up();
  delay(1000);
  put_down();*/

  
  

  //draw_line(30, 150, 30, 100);

//draw_circle(80, 151, 16);
  
  delay(1000);
}
