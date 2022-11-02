
#define kp_30 0.16
#define ki_30 0.0003
#define kd_30 0.5
#define kp2_30 0.16
#define ki2_30 0.00035
#define kd2_30 0.5

#define kp_50 0.22
#define ki_50 0.00005
#define kd_50 0.5
#define kp2_50 0.25
#define ki2_50 0.00005
#define kd2_50 0.5

#define kp_70 0.24
#define ki_70 0.00009
#define kd_70 0.5
#define kp2_70 0.23
#define ki2_70 0.00009
#define kd2_70 0.5
#define prox_pin 15

float v1filt = 0;
float v1prev= 0;
float v2filt = 0;
float v2prev= 0;

#include <ros.h>
#include <std_msgs/Float32.h>
 
ros::NodeHandle nh;

std_msgs::Float32 str_msg;
ros::Publisher chatter("chatter", &str_msg);

float var;
 
float camera_flag ;

float prox_flag ;

 

//////////////////////////////////////////////////// communication /////////////////////////////////////////


float kp = 0.08;
float ki = 0.0002;
float kd = 0.5;
float kp2 = 0.05;
float ki2 = 0.0002;
float kd2 = 0.5;
unsigned long t;
unsigned long t_prev = 0;
//////////////////////////////////
const byte interruptPinA = 2;
const byte interruptPinB = 3;   ///////////////////////////////////////////////////////////////////////////////*MOHEEEEEEEEEEEEMMMM/////////////
const byte interruptPinC = 18;
const byte interruptPinD = 19;
volatile long EncoderCount1 = 0;
volatile long EncoderCount2 = 0;

const byte PWMPin1 = 8;
const byte PWMPin2 = 9;
const byte DirPin1 = 11;
const byte DirPin2 = 12;
const byte DirPin3 = 5;
const byte DirPin4 = 6;
///////////////////////////////
volatile unsigned long count = 0;
unsigned long count_prev = 0;

float Theta, RPM, RPM_d;
float Theta2, RPM2, RPM_d2;
float Theta_prev = 0;
float Theta_prev2 = 0 ;
int dt;
float RPM_max = 230;

#define pi 3.1416
float Vmax = 12;
float Vmin = -12;
float V = 0.1;
float V2 = 0.1;

float e, e_prev = 0, inte, inte_prev = 0;
float e2, e_prev2 = 0, inte2, inte_prev2 = 0;


 void ISR_EncoderA(){
  int b = digitalRead(interruptPinB);
  if(b>0){
     EncoderCount1++;
    }
else{
  EncoderCount1--;
  }
}

void ISR_EncoderC(){
  int d = digitalRead(interruptPinD);
  if(d>0){
     EncoderCount2++;
    }
else{
   EncoderCount2--;
  }
}


float sign(float x) {
  if (x > 0) {
    return 1;
  } else if (x < 0) {
    return -1;
  } else {
    return 0;
  }
}


//***Motor Driver Functions*****
int PWMval = 0;
void WriteDriverVoltage1(float V, float Vmax) {    /// eh heya el v??   el vmax = 6 volt
  PWMval = int(255 * abs(V) / Vmax);
  if (PWMval > 255) {
    PWMval = 255;
  }
  if (V > 0) {
    digitalWrite(DirPin1, HIGH);
    digitalWrite(DirPin2, LOW);

  }
  else if (V < 0) {
    digitalWrite(DirPin1, LOW);

    digitalWrite(DirPin2, HIGH);

  }
  else {
    digitalWrite(DirPin1, LOW);
    digitalWrite(DirPin2, LOW);
  }
  analogWrite(PWMPin1, PWMval);
}


void WriteDriverVoltage2(float V, float Vmax) {    /// eh heya el v??   el vmax = 6 volt
  PWMval = int(255 * abs(V) / Vmax);
  if (PWMval > 255) {
    PWMval = 255;
  }
  if (V > 0) {
    digitalWrite(DirPin3, HIGH);
    digitalWrite(DirPin4, LOW);

  }
  else if (V < 0) {
    digitalWrite(DirPin3, LOW);

    digitalWrite(DirPin4, HIGH);
  }

  else {
    digitalWrite(DirPin3, LOW);
    digitalWrite(DirPin4, LOW);
  }
  analogWrite(PWMPin2, PWMval);
}





void messageCb(const std_msgs::Float32 &msg)
{
  var=msg.data; 
  if(var == 1){
    RPM_d = 70;
    RPM_d2= 30;
    kp = kp_70;
    ki = ki_70;
    kd = kd_70;
    kp2 = kp2_30;
    ki2 = ki2_30;
    kd2= kd2_30;
    str_msg.data = 1;
   chatter.publish( &str_msg );
   }
   if(var == 2){
    RPM_d = 70;
    RPM_d2= 50;
    kp = kp_70;
    ki = ki_70;
    kd = kd_70;
    kp2 = kp2_50;
    ki2 = ki2_50;
    kd2= kd2_50;
    str_msg.data = 2;
   chatter.publish( &str_msg );
   }
   if(var == 3){
    
    RPM_d = 50;
    RPM_d2= 50;

    
    kp = kp_50;
    ki = ki_50;
    kd = kd_50;
    kp2 = kp2_50;
    ki2 = ki2_50;
    kd2= kd2_50;
     str_msg.data = 3;
   chatter.publish( &str_msg );
   }
   if(var == 4){
    RPM_d = 50;
    RPM_d2= 70;
    kp = kp_50;
    ki = ki_50;
    kd = kd_50;
    kp2 = kp2_70;
    ki2 = ki2_70;
    kd2= kd2_70;
    
   str_msg.data = 4;
   chatter.publish( &str_msg );
  
   }
   if(var == 5 ){
    
    RPM_d = 30;
    RPM_d2= 70;
    kp = kp_30;
    ki = ki_30;
    kd = kd_30;
    kp2 = kp2_70;
    ki2 = ki2_70;
    kd2= kd2_70;
   str_msg.data = 5;
   chatter.publish( &str_msg );
   }

   if (var== 0){
    
    RPM_d = 50;
    RPM_d2= 50;
    
    kp = kp_80;
    ki = ki_80;
    kd = kd_80;
    kp2 = kp2_80;
    ki2 = ki2_80;
    kd2= kd2_80;
   str_msg.data = 0;
   chatter.publish( &str_msg ); 
} 
}
void messageCb2(const std_msgs::Float32 &msg1){
   camera_flag=msg1.data; 
  }

ros::Subscriber<std_msgs::Float32> sub("line_follower", &messageCb);
ros::Subscriber<std_msgs::Float32> sub2("camera_vision", &messageCb2);


  

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
  nh.subscribe(sub2);  
   pinMode(LED_BUILTIN, OUTPUT);
   
  pinMode(interruptPinA, INPUT);
  pinMode(interruptPinB, INPUT);
   attachInterrupt(digitalPinToInterrupt(interruptPinA), ISR_EncoderA,  RISING);
 // attachInterrupt(digitalPinToInterrupt(interruptPinA), ISR_EncoderA, CHANGE);
 // attachInterrupt(digitalPinToInterrupt(interruptPinB), ISR_EncoderB, CHANGE);
  pinMode(DirPin1, OUTPUT);
  pinMode(DirPin2, OUTPUT);
  // for second driver
  pinMode(interruptPinC, INPUT);
  pinMode(interruptPinD, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPinC), ISR_EncoderC, RISING);
  //attachInterrupt(digitalPinToInterrupt(interruptPinD), ISR_EncoderD, CHANGE);
  pinMode(DirPin3, OUTPUT);
  pinMode(DirPin4, OUTPUT);
  
  pinMode(prox_pin, INPUT);
  
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 12499; //Prescaler = 64
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11 | 1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  sei();
}

void loop() {

  digitalWrite(LED_BUILTIN, LOW);
 if(camera_flag == 10 or camera_flag == 11 or prox_flag== 0){
   analogWrite(PWMPin2, 0);
    analogWrite(PWMPin1, 0);
   digitalWrite(DirPin3, HIGH);
   digitalWrite(DirPin4, LOW);
   digitalWrite(DirPin1, HIGH);
   digitalWrite(DirPin2, LOW);
   digitalWrite(LED_BUILTIN, HIGH);
  delay(3000);
  WriteDriverVoltage1(6, Vmax);
   WriteDriverVoltage2(8, Vmax);

  delay(1500);
  analogWrite(PWMPin2, 0);
  analogWrite(PWMPin1, 0);
  }
  
 /* if(prox_flag == 1){
  analogWrite(PWMPin2, 0);
  analogWrite(PWMPin1, 0);
   digitalWrite(DirPin3, HIGH);
   digitalWrite(DirPin4, LOW);
   digitalWrite(DirPin1, HIGH);
   digitalWrite(DirPin2, LOW);
   delay(5000);
   
   }*/

   
    if(prox_flag ==1 ){  
   if(camera_flag == 12){
  if (count > count_prev) {
    t = millis();    // b7sb el wa2t mn bdayt el loop
    Theta = EncoderCount1 / 300.0;    // el one revolution feeha 900 count yb2a a2dr ageeb el theta
    Theta2 = EncoderCount2 / 300.0;
    dt = (t - t_prev);  // theta 3ady
    //RPM_d = -80  ;     
    //RPM_d2 = -80;
    
    RPM = (Theta - Theta_prev) / (dt / 1000.0) * 60;  // b2sm 3la alf 34an a7wl milly to seconds and then * 60 to get per menuite

    RPM2 = (Theta2 - Theta_prev2) / (dt / 1000.0) * 60;

    
   v1filt = 0.8548*v1filt+ RPM*0.0728 +0.0728*v1prev ;
    v1prev = RPM;
    v2filt = 0.8548*v2filt+ RPM2*0.0728 +0.0728*v2prev ;
    v2prev = RPM2;

    
    e = RPM_d - v1filt;
    e2 = RPM_d2 - v2filt;
    
    inte = inte_prev + (dt * (e + e_prev) / 2);
    inte2 = inte_prev2 + (dt * (e2 + e_prev2) / 2);


    V = kp * e + ki * inte + (kd * (e - e_prev) / dt) ;

    V2 = kp2 * e2 + ki2 * inte2 + (kd2 * (e2 - e_prev2) / dt) ;

    if (V > Vmax) {
      V = Vmax;
      inte = inte_prev;
    }
    if (V < Vmin) {
      V = Vmin;
      inte = inte_prev;
      
    }
    if (V2 > Vmax) {
      V2 = Vmax;

      inte2 = inte_prev2;
    }
    if (V2 < Vmin) {
      V2 = Vmin;
      inte2 = inte_prev2;
    }


   WriteDriverVoltage1(V, Vmax);
   WriteDriverVoltage2(V2, Vmax);


    //Serial.print(RPM_d2); Serial.print(" \t");
    //Serial.print(RPM2); Serial.print(" \t ");
    //Serial.print(V2); Serial.println("\t  ");
    
      Serial.print(RPM_d2); Serial.print(" \t");
      Serial.print(RPM2); Serial.print(" \t ");
      Serial.print(V2); Serial.println("\t  ");
      //Serial.print(e); Serial.println("  ");

    Theta_prev = Theta;
    count_prev = count;
    t_prev = t;
    inte_prev = inte;
    e_prev = e;


    Theta_prev2 = Theta2;
    inte_prev2 = inte2;
    e_prev2 = e2;
   
  }}}
  
nh.spinOnce();
prox_flag =digitalRead(prox_pin);
  delay(1);
}



ISR(TIMER1_COMPA_vect) {
  count++;
  Serial.print(count * 0.05); Serial.print(" \t");
}
