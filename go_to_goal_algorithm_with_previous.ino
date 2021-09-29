
#include <math.h>

#define KP            25
#define KI            0.5
#define KD             0

#define cons          50

#define N             48    //Number of ticks per revolution
#define pi            3.1614  // Value of pi

#define  input_1       23    //For Motor(1)
#define  input_2       25
#define  enable_1      2
#define  input_3       29    //For Motor(1)
#define  input_4       27
#define  enable_2      3

#define Channel_A_Motor_1       18
#define Channel_B_Motor_1       19
#define Channel_A_Motor_2       20
#define Channel_B_Motor_2       21

#define Interrupt_Channel_A_Motor_1       5
#define Interrupt_Channel_B_Motor_1       4
#define Interrupt_Channel_A_Motor_2       3
#define Interrupt_Channel_B_Motor_2       2

long int count_1 = 0 , count_2 = 0;
boolean A , B , C , D;
byte state_1 , statep_1 , state_2 , statep_2 ;
volatile int QEM[16] = { 0, -1, 0, 1, 1, 0, -1, 0, 0, 1, 0, -1, -1, 0, 1, 0 };
volatile int index_1 , index_2;

float L = 155 , R = 21;                  //Distance of robot wheel and radius of the wheel
float v = 3500;                          //Input linear velocity 
float vel_r, vel_l;
float d_left, d_right, d_center, phi;
float theta = 0, x=0, y=0;
float theta_dt, x_dt, y_dt;
float m_per_tick = (2*pi*R)/N;

float error_old = 0.0, error_new, input;
float p = 0.0, i = 0.0, d = 0.0;
float u_x , u_y;
float theta_g;
float e_k;

int x_g = -1000;    //This is the goal point 
int y_g = -1000;

int right_ticks, left_ticks;
int prev_right_ticks = 0, prev_left_ticks = 0;
int pwm_r, pwm_l;
int PWM[101] = {255,255,255,255,254,253,251,249,247,245,243,240,237,235,232,230,225,220,215,210,200,190,180,175,175,170,170,167,165,163,160,157,155,153,150,145,140,137,135,133,130,129,128,127,126,126,125,125,124,123,122,121,121,120,120,120,119,118,117,116,115,115,114,114,114,113,113,112,111,111,110,108,106,104,102,100,100,99,98,97,96,95,94,93,92,91,90,90,90,89,89,89,88,88,88,87,87,87,86,86,0};

void Achange();
void Bchange();
void Cchange();
void Dchange();

void setup()
{
  Serial.begin(57600); 

  pinMode(input_1 , OUTPUT);    //For Motor(1)
  pinMode(input_2 , OUTPUT);
  pinMode(enable_1 , OUTPUT);

  pinMode(input_3 , OUTPUT);    //For Motor(2)
  pinMode(input_4 , OUTPUT);
  pinMode(enable_2 , OUTPUT);
  
  pinMode(Channel_A_Motor_1, INPUT);
  pinMode(Channel_B_Motor_1, INPUT); 
  pinMode(Channel_A_Motor_2, INPUT);   
  pinMode(Channel_B_Motor_2, INPUT);
       
  attachInterrupt(Interrupt_Channel_A_Motor_1, Achange, CHANGE);
  attachInterrupt(Interrupt_Channel_B_Motor_1, Bchange, CHANGE);
  attachInterrupt(Interrupt_Channel_A_Motor_2, Cchange, CHANGE);
  attachInterrupt(Interrupt_Channel_B_Motor_2, Dchange, CHANGE);
   
  //Read the initial state value of A , B , C and D
  A = digitalRead(Channel_A_Motor_1);
  B = digitalRead(Channel_B_Motor_1);
  C = digitalRead(Channel_A_Motor_2);
  D = digitalRead(Channel_B_Motor_2);

  //Set initial state value of Motor_1
  if( (A==HIGH)&&(B==HIGH) )  
  {
    statep_1 = 0;
  }
  else if( (A==HIGH)&&(B==LOW) )
  {
    statep_1 = 1;
  }
  else if( (A==LOW)&&(B==LOW) )
  {
    statep_1 = 2;
  }
  else if( (A==LOW)&&(B==HIGH) )
  {
    statep_1 = 3;
  }

  //Set initial state value of Motor_2
  if( (C==HIGH)&&(D==HIGH) )  
  {
    statep_2 = 0;
  }
  else if( (C==HIGH)&&(D==LOW) )
  {
    statep_2 = 1;
  }
  else if( (C==LOW)&&(D==LOW) )
  {
    statep_2 = 2;
  }
  else if( (C==LOW)&&(D==HIGH) )
  {
    statep_2 = 3;
  }
}

void loop()
{  
  right_ticks = count_1;
  left_ticks = count_2;

  delay(10);

  d_right = m_per_tick * (right_ticks - prev_right_ticks);
  d_left = m_per_tick * (left_ticks - prev_left_ticks);

  //Serial.print(d_right);
  //Serial.print("    ");
  //Serial.println(d_left);

  prev_right_ticks = right_ticks;
  prev_left_ticks = left_ticks;

  d_center = (d_right + d_left)/2;
  phi = (d_right - d_left)/L;

  //Serial.print(d_center);
  //Serial.print("    ");
  //Serial.println(phi);

  x_dt = d_center*cos(theta);
  y_dt = d_center*sin(theta);
  theta_dt = phi;

  theta = theta + theta_dt;
  theta = atan2(sin(theta), cos(theta));
  x = x + x_dt;
  y = y + y_dt;

  Serial.print(abs(x_g-x));
  Serial.print("    ");
  Serial.print(abs(y_g-y));
  Serial.print("    ");

  if( (abs(x_g-x) < cons) && (abs(y_g-y) < cons) )
  {
    digitalWrite(input_1 , LOW);
    digitalWrite(input_2 , LOW);
    digitalWrite(enable_1 , LOW);
    
    digitalWrite(input_3 , LOW);
    digitalWrite(input_4 , LOW);
    digitalWrite(enable_2 , LOW); 
  }
  
  else
  {
    Serial.print(x);
    Serial.print("    ");
    Serial.print(y);
    Serial.print("    ");
    Serial.print(theta);
    Serial.print("    ");
  
    u_x = x_g-x;
    u_y = y_g-y;   
    theta_g = atan2(u_y,u_x);
  
    error_new = theta_g - theta;
    error_new = atan2(sin(error_new),cos(error_new));
  
    Serial.print(theta_g);
    Serial.print("    ");
    Serial.println(error_new);
  
    p = (KP * error_new);
    i = (KI * (i + error_new));
    d = (KD * (error_new - error_old));
  
    input = (p + i + d);
    error_old = error_new;
  
    //Serial.println(error_new);
  
    vel_r = ((2*v)+(input*L)) /(2*R);
    vel_l = ((2*v)-(input*L))/(2*R);
  
    //Serial.print(vel_r);
    //Serial.print("    ");
    //Serial.println(vel_l);
  
    vel_r = constrain(vel_r , 100 , 200);
    vel_l = constrain(vel_l , 100 , 200);
    
    //Serial.print(x);
    //Serial.print("    ");
    //Serial.print(y);
    //Serial.print("    ");
    //Serial.println(theta);
    
    int m_r = 200-vel_r;
    int a_r = PWM[m_r];
  
    int m_l = 200-vel_l;
    int a_l = PWM[m_l];
  
    //Serial.print(a_r);
    //Serial.print("    ");
    //Serial.println(a_l); 
  
    digitalWrite(input_1 , HIGH);
    digitalWrite(input_2 , LOW);
    digitalWrite(enable_1 , LOW);
    analogWrite(enable_1 , a_r);
      
    digitalWrite(input_3 , HIGH);
    digitalWrite(input_4 , LOW);
    digitalWrite(enable_2 , LOW);
    analogWrite(enable_2 , a_l);
  } 
}

void Achange()
{
  A = digitalRead(Channel_A_Motor_1);
  B = digitalRead(Channel_B_Motor_1);

  //Determine State Value
  if( (A==HIGH)&&(B==HIGH) )  
  {
    state_1 = 0;
  }
  else if( (A==HIGH)&&(B==LOW) )
  {
    state_1 = 1;
  }
  else if( (A==LOW)&&(B==LOW) )
  {
    state_1 = 2;
  }
  else if( (A==LOW)&&(B==HIGH) )
  {
    state_1 = 3;
  }

  index_1 = 4*state_1 + statep_1;
  count_1 = count_1 + QEM[index_1];
  statep_1 = state_1;
}

void Bchange()
{
  A = digitalRead(Channel_A_Motor_1);
  B = digitalRead(Channel_B_Motor_1);

  //Determine State Value
  if( (A==HIGH)&&(B==HIGH) )  
  {
    state_1 = 0;
  }
  else if( (A==HIGH)&&(B==LOW) )
  {
    state_1 = 1;
  }
  else if( (A==LOW)&&(B==LOW) )
  {
    state_1 = 2;
  }
  else if( (A==LOW)&&(B==HIGH) )
  {
    state_1 = 3;
  }

  index_1 = 4*state_1 + statep_1;
  count_1 = count_1 + QEM[index_1];
  statep_1 = state_1;
}

void Cchange()
{
  C = digitalRead(Channel_A_Motor_2);
  D = digitalRead(Channel_B_Motor_2);

  //Determine State Value
  if( (C==HIGH)&&(D==HIGH) )  
  {
    state_2 = 0;
  }
  else if( (C==HIGH)&&(D==LOW) )
  {
    state_2 = 1;
  }
  else if( (C==LOW)&&(D==LOW) )
  {
    state_2 = 2;
  }
  else if( (C==LOW)&&(D==HIGH) )
  {
    state_2 = 3;
  }

  index_2 = 4*state_2 + statep_2;
  count_2 = count_2 + QEM[index_2];
  statep_2 = state_2;
}

void Dchange()
{
  C = digitalRead(Channel_A_Motor_2);
  D = digitalRead(Channel_B_Motor_2);

  //Determine State Value
  if( (C==HIGH)&&(D==HIGH) )  
  {
    state_2 = 0;
  }
  else if( (C==HIGH)&&(D==LOW) )
  {
    state_2 = 1;
  }
  else if( (C==LOW)&&(D==LOW) )
  {
    state_2 = 2;
  }
  else if( (C==LOW)&&(D==HIGH) )
  {
    state_2 = 3;
  }

  index_2 = 4*state_2 + statep_2;
  count_2 = count_2 + QEM[index_2];
  statep_2 = state_2;
}
