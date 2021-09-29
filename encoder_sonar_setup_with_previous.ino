
#include <math.h>

#define cons          0.01

#define N             48    //Number of ticks per revolution

#define pi            3.1416 // Value of pi

#define  input_1       7    //For Motor(2)
#define  input_2       6
#define  enable_1      5

#define  input_3       10  //For Motor(1)
#define  input_4       11
#define  enable_2      9

#define  switchPin     8    //For Switch

#define Channel_A_Motor_1       2
#define Channel_B_Motor_1       3
#define Channel_A_Motor_2       20
#define Channel_B_Motor_2       21

#define Interrupt_Channel_A_Motor_1       0
#define Interrupt_Channel_B_Motor_1       1
#define Interrupt_Channel_A_Motor_2       3
#define Interrupt_Channel_B_Motor_2       2

long int count_1 = 0 , count_2 = 0 ;
boolean A , B , C , D;
byte state_1 , statep_1 , state_2 , statep_2 ;
volatile int QEM[16] = { 0, -1, 0, 1, 1, 0, -1, 0, 0, 1, 0, -1, -1, 0, 1, 0 };
volatile int index_1 , index_2;

float L = 150 , R = 21;   //Distance of robot wheel and radius of the wheel
float v = 0.1 , w = (pi/4);   //Input linear velocity and angular velocity
float  vel_r , vel_l;
int pwm_r , pwm_l;

int right_ticks, left_ticks;
int prev_right_ticks = 0, prev_left_ticks = 0;
float d_left, d_right, d_center, phi;
float theta=0.0, x=0.0, y=0.0;
float theta_dt, x_dt, y_dt;

float m_per_tick = (2*pi*R)/N;

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

  pinMode(switchPin, INPUT);    //For Switch
  
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
  if(digitalRead(switchPin) == HIGH)
  { 
    right_ticks = count_1;
    left_ticks = count_2;

    d_right = m_per_tick * (right_ticks - prev_right_ticks);
    d_left = m_per_tick * (left_ticks - prev_left_ticks);

    prev_right_ticks = right_ticks;
    prev_left_ticks = left_ticks;
  
    d_center = (d_right + d_left)/2;
    phi = (d_right - d_left)/L;        

    x_dt = d_center*cos(theta);
    y_dt = d_center*sin(theta);
    theta_dt = phi;

    theta = theta + theta_dt;
    Serial.println(theta);
    x = x + x_dt;
    y = y + y_dt;

    vel_r = (2*v+w*L)/(2*R);
    vel_l = (2*v-w*L)/(2*R);

    pwm_r = 5*15*exp(0.002*abs(vel_r));
    pwm_l = 5*15*exp(0.002*abs(vel_l));

    Serial.print(pwm_r);
    Serial.print("    ");
    Serial.println(pwm_l);
    
    if(vel_r >= 0)
    {
      digitalWrite(input_1 , HIGH);
      digitalWrite(input_2 , LOW);
      digitalWrite(enable_1 , LOW);
      
      analogWrite(enable_1 , pwm_r);
    }
    
    else
    {
      digitalWrite(input_1 , LOW);
      digitalWrite(input_2 , HIGH);
      digitalWrite(enable_1 , LOW);
      
      analogWrite(enable_1 , pwm_r);      
    }

    if(vel_l >= 0)
    {
      digitalWrite(input_3 , HIGH);
      digitalWrite(input_4 , LOW);
      digitalWrite(enable_2 , LOW);
      
      analogWrite(enable_2 , pwm_l);
    }
    
    else
    {
      digitalWrite(input_3 , LOW);
      digitalWrite(input_4 , HIGH);
      digitalWrite(enable_2 , LOW);
      
      analogWrite(enable_2 , pwm_l);      
    }
  }
  
  else
  {
    digitalWrite(input_1 , LOW);
    digitalWrite(input_2 , LOW);
    digitalWrite(enable_1 , LOW);
  
    digitalWrite(input_3 , LOW);
    digitalWrite(input_4 , LOW);
    digitalWrite(enable_2 , LOW);     
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
