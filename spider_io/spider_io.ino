/* -----------------------------------------------------------------------------
  - Project: Remote control Crawling robot
  - Author:  panerqiang@sunfounder.com
  - Date:  2015/1/27
   -----------------------------------------------------------------------------
  - Overview
  - This project was written for the Crawling robot desigened by Sunfounder.
    This version of the robot has 4 legs, and each leg is driven by 3 servos.
  This robot is driven by a Ardunio Nano Board with an expansion Board.
  We recommend that you view the product documentation before using.
  - Request
  - This project requires some library files, which you can find in the head of
    this file. Make sure you have installed these files.
  - How to
  - Before use,you must to adjust the robot,in order to make it more accurate.
    - Adjustment operation
    1.uncomment ADJUST, make and run
    2.comment ADJUST, uncomment VERIFY
    3.measure real sites and set to real_site[4][3], make and run
    4.comment VERIFY, make and run
  The document describes in detail how to operate.
   ---------------------------------------------------------------------------*/

// modified by Regis for spider project, 2015-09-26
// add remote control by HC-06 bluetooth module


/* Includes ------------------------------------------------------------------*/
#include <Servo.h>    //to define and control servos
#include <FlexiTimer2.h>//to set a timer to manage all servos
// RegisHsu, remote control
//#include <SerialCommand.h>/
//SerialCommand SCmd;   // The /demo SerialCommand object

/* Servos --------------------------------------------------------------------*/
//define 12 servos for 4 legs
Servo servo[4][3];
//define servos' ports
const int servo_pin[4][3] = { {2, 3, 4}, {5, 6, 7}, {8, 9, 10}, {11, 12, 13} };

/* Size of the robot ---------------------------------------------------------*/
const float length_a    = 55;
const float length_b    = 77.5;
const float length_c    = 27.5;
const float length_side = 71;
const float z_absolute  = -28;

/* Constants for movement ----------------------------------------------------*/
const float z_default = -50, z_up = -30, z_boot = z_absolute;
const float x_default = 62, x_offset = 0;
const float y_start   = 0, y_step = 40;

/* variables for movement ----------------------------------------------------*/
volatile float site_now[4][3];    //real-time coordinates of the end of each leg
volatile float site_expect[4][3]; //expected coordinates of the end of each leg
float temp_speed[4][3];   //each axis' speed, needs to be recalculated before each movement
float move_speed;     //movement speed
float speed_multiple = 1; //movement speed multiple
float spot_turn_speed = 4;
float leg_move_speed = 8;
float body_move_speed = 3;
const float stand_seat_speed = 1;
volatile int rest_counter;      //+1/0.02s, for automatic rest
//functions' parameter
const float KEEP = 255;
//define PI for calculation
const float pi = 3.1415926;

/* Constants for turn --------------------------------------------------------*/
//temp length
const float temp_a      = sqrt(pow(2 * x_default + length_side, 2) + pow(y_step, 2));
const float temp_b      = 2 * (y_start + y_step) + length_side;
const float temp_c      = sqrt(pow(2 * x_default + length_side, 2) + pow(2 * y_start + y_step + length_side, 2));
const float temp_alpha  = acos((pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / 2 / temp_a / temp_b);
//site for turn
const float turn_x1     = (temp_a - length_side) / 2;
const float turn_y1     = y_start + y_step / 2;
const float turn_x0     = turn_x1 - temp_b * cos(temp_alpha);
const float turn_y0     = temp_b * sin(temp_alpha) - turn_y1 - length_side;

// pin-A0
#define IR_Detect_IO 14

/* ---------------------------------------------------------------------------*/

/*
  - setup function
   ---------------------------------------------------------------------------*/
void setup()
{
  //start serial for debug
  Serial.begin(9600);
  Serial.println("Robot starts initialization");
  // config IR_Detect_IO pin as input
  pinMode(IR_Detect_IO, INPUT);

  // RegisHsu, remote control
  // Setup callbacks for SerialCommand commands
  // action command 0-6,
  // w 0 1: stand
  // w 0 0: sit
  // w 1 x: forward x step
  // w 2 x: back x step
  // w 3 x: right turn x step
  // w 4 x: left turn x step
  // w 5 x: hand shake x times
  // w 6 x: hand wave x times
  ///  SCmd.addCommand("w", action_cmd);

  ///  SCmd.setDefaultHandler(unrecognized);

  //initialize default parameter
  set_site(0, x_default - x_offset, y_start , z_boot);
  set_site(1, x_default - x_offset, y_start + y_step, z_boot);
  set_site(2, x_default + x_offset, y_start, z_boot);
  set_site(3, x_default + x_offset, y_start, z_boot);
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      site_now[i][j] = site_expect[i][j];
    }
  }
  //start servo service
  FlexiTimer2::set(20, servo_service);
  FlexiTimer2::start();
  Serial.println("Servo service started");
  //initialize servos
  servo_attach();
  Serial.println("Servos initialized");
  Serial.println("Robot initialization Complete");
}


void servo_attach(void)
{
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      servo[i][j].attach(servo_pin[i][j]);
      delay(100);
    }
  }
}

void servo_detach(void)
{
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      servo[i][j].detach();
      delay(100);
    }
  }
}

// RegisHsu
// w 0 1: stand
// w 0 0: sit
// w 1 x: forward x step
// w 2 x: back x step
// w 3 x: right turn x step
// w 4 x: left turn x step
// w 5 x: hand shake x times
// w 6 x: hand wave x times
#define W_STAND_SIT    '0'
#define W_FORWARD      '1'
#define W_BACKWARD     '2'
#define W_LEFT         '3'
#define W_RIGHT        '4'
#define W_SHAKE        '5'
#define W_WAVE         '6'

/*
  - loop function
   ---------------------------------------------------------------------------*/
int flag_obstacle = 0;
int mode_left_right = 0;

void readSerial() {
  char c = Serial.read();
  //  int action = atoi(c);
  int n_step = 1;
  Serial.println(c);
  switch (c)
  {
    case W_FORWARD:
      Serial.println("Step forward");
      if (!is_stand())
        stand();
      step_forward(n_step);
      break;
    case W_BACKWARD:
      Serial.println("Step back");
      if (!is_stand())
        stand();
      step_back(n_step);
      break;
    case W_LEFT:
      Serial.println("Turn left");
      if (!is_stand())
        stand();
      turn_left(n_step);
      break;
    case W_RIGHT:
      Serial.println("Turn right");
      if (!is_stand())
        stand();
      turn_right(n_step);
      break;
    case W_STAND_SIT:
      Serial.println("1:up,0:dn");
      if (n_step)
        stand();
      else
        sit();
      break;
    case W_SHAKE:
      Serial.println("Hand shake");
      hand_shake(n_step);
      break;
    case W_WAVE:
      Serial.println("Hand wave");
      hand_wave(n_step);
      break;
    default:
      Serial.println("Error");
      break;
  }
}
//command given to the robot
int cmd = 1;

void loop()
{
  if (cmd == 1) {
    do_test();
  }
  else if (cmd == 2) {
    readSerial();
  }
  /*
    int tmp_turn, tmp_leg, tmp_body;
    //Regis, 2015-07-15, for Bluetooth command
    //  /SCmd.readSerial();
    if (!digitalRead(IR_Detect_IO) && is_stand())
    {
    tmp_turn = spot_turn_speed;
    tmp_leg = leg_move_speed;
    tmp_body = body_move_speed;
    spot_turn_speed = leg_move_speed = body_move_speed = 20;
    if (flag_obstacle < 3)
    {
      step_back(1);
      flag_obstacle++;
    }
    else
    {
      if (mode_left_right)
        turn_right(1);
      else
        turn_left(1);
      mode_left_right = 1 - mode_left_right;
      flag_obstacle = 0;
    }
    spot_turn_speed = tmp_turn;
    leg_move_speed = tmp_leg;
    body_move_speed = tmp_body;
    }*/
}

void action_cmd(void)
{
  char *arg;
  int action_mode, n_step;
  Serial.println("Action:");
  //  arg = SCmd.next();/
  action_mode = atoi(arg);
  ///  arg = SCmd.next();
  n_step = atoi(arg);
  delay(1000);
  switch (action_mode)
  {
    case W_FORWARD:
      Serial.println("Step forward");
      if (!is_stand())
        stand();
      step_forward(n_step);
      break;
    case W_BACKWARD:
      Serial.println("Step back");
      if (!is_stand())
        stand();
      step_back(n_step);
      break;
    case W_LEFT:
      Serial.println("Turn left");
      if (!is_stand())
        stand();
      turn_left(n_step);
      break;
    case W_RIGHT:
      Serial.println("Turn right");
      if (!is_stand())
        stand();
      turn_right(n_step);
      break;
    case W_STAND_SIT:
      Serial.println("1:up,0:dn");
      if (n_step)
        stand();
      else
        sit();
      break;
    case W_SHAKE:
      Serial.println("Hand shake");
      hand_shake(n_step);
      break;
    case W_WAVE:
      Serial.println("Hand wave");
      hand_wave(n_step);
      break;
    default:
      Serial.println("Error");
      break;
  }
}



// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command) {
  Serial.println("What?");
}

/*
  - microservos service /timer interrupt function/50Hz
  - when set site expected,this function move the end point to it in a straight line
  - temp_speed[4][3] should be set before set expect site,it make sure the end point
   move in a straight line,and decide move speed.
   ---------------------------------------------------------------------------*/
void servo_service(void)
{
  sei();
  static float alpha, beta, gamma;

  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      if (abs(site_now[i][j] - site_expect[i][j]) >= abs(temp_speed[i][j]))
        site_now[i][j] += temp_speed[i][j];
      else
        site_now[i][j] = site_expect[i][j];
    }

    cartesian_to_polar(alpha, beta, gamma, site_now[i][0], site_now[i][1], site_now[i][2]);
    polar_to_servo(i, alpha, beta, gamma);
  }

  rest_counter++;
}

/*
  - set one of end points' expect site
  - this founction will set temp_speed[4][3] at same time
  - non - blocking function
   ---------------------------------------------------------------------------*/
void set_site(int leg, float x, float y, float z)
{
  float length_x = 0, length_y = 0, length_z = 0;

  if (x != KEEP)
    length_x = x - site_now[leg][0];
  if (y != KEEP)
    length_y = y - site_now[leg][1];
  if (z != KEEP)
    length_z = z - site_now[leg][2];

  float length = sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));

  temp_speed[leg][0] = length_x / length * move_speed * speed_multiple;
  temp_speed[leg][1] = length_y / length * move_speed * speed_multiple;
  temp_speed[leg][2] = length_z / length * move_speed * speed_multiple;

  if (x != KEEP)
    site_expect[leg][0] = x;
  if (y != KEEP)
    site_expect[leg][1] = y;
  if (z != KEEP)
    site_expect[leg][2] = z;
}

/*
  - wait one of end points move to expect site
  - blocking function
   ---------------------------------------------------------------------------*/
void wait_reach(int leg)
{
  while (1)
    if (site_now[leg][0] == site_expect[leg][0])
      if (site_now[leg][1] == site_expect[leg][1])
        if (site_now[leg][2] == site_expect[leg][2])
          break;
}

/*
  - wait all of end points move to expect site
  - blocking function
   ---------------------------------------------------------------------------*/
void wait_all_reach(void)
{
  for (int i = 0; i < 4; i++)
    wait_reach(i);
}

/*
  - trans site from cartesian to polar
  - mathematical model 2/2
   ---------------------------------------------------------------------------*/
void cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z)
{
  //calculate w-z degree
  float v, w;
  w = (x >= 0 ? 1 : -1) * (sqrt(pow(x, 2) + pow(y, 2)));
  v = w - length_c;
  alpha = atan2(z, v) + acos((pow(length_a, 2) - pow(length_b, 2) + pow(v, 2) + pow(z, 2)) / 2 / length_a / sqrt(pow(v, 2) + pow(z, 2)));
  beta = acos((pow(length_a, 2) + pow(length_b, 2) - pow(v, 2) - pow(z, 2)) / 2 / length_a / length_b);
  //calculate x-y-z degree
  gamma = (w >= 0) ? atan2(y, x) : atan2(-y, -x);
  //trans degree pi->180
  alpha = alpha / pi * 180;
  beta = beta / pi * 180;
  gamma = gamma / pi * 180;
}

/*
  - trans site from polar to microservos
  - mathematical model map to fact
  - the errors saved in eeprom will be add
   ---------------------------------------------------------------------------*/
void polar_to_servo(int leg, float alpha, float beta, float gamma)
{
  if (leg == 0)
  {
    alpha = 90 - alpha;
    beta = beta;
    gamma += 90;
  }
  else if (leg == 1)
  {
    alpha += 90;
    beta = 180 - beta;
    gamma = 90 - gamma;
  }
  else if (leg == 2)
  {
    alpha += 90;
    beta = 180 - beta;
    gamma = 90 - gamma;
  }
  else if (leg == 3)
  {
    alpha = 90 - alpha;
    beta = beta;
    gamma += 90;
  }

  servo[leg][0].write(alpha);
  servo[leg][1].write(beta);
  servo[leg][2].write(gamma);
}
