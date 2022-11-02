float kp = 0.01;
float ki = 0.00015;
float kd = 0.5;

unsigned long t;
unsigned long t_prev = 0;
////////////////////////////////// 
const byte interruptPinA = 3;
const byte interruptPinB = 2;
volatile long EncoderCount = 0;
const byte PWMPin = 8;
const byte DirPin1 = 11;
const byte DirPin2 = 12;
///////////////////////////////
volatile unsigned long count = 0;
unsigned long count_prev = 0;

float Theta, RPM, RPM_d;
float Theta_prev = 0;
int dt;
#define pi 3.1416
float Vmax = 12;
float Vmin = -12;
float V = 0.1;
float e, e_prev = 0, inte, inte_prev = 0;

//**********FUNCTIONS******************
//     Void ISR_EncoderA
//     Void ISR_EncoderB
//     Void Motor Driver Write
//     Timer Interrupt
//*************************************


void ISR_EncoderA() {
  bool PinB = digitalRead(interruptPinB);
  bool PinA = digitalRead(interruptPinA);

  if (PinB == LOW) {
    if (PinA == HIGH) {
      EncoderCount++;
    }
    else {
      EncoderCount--;
    }
  }

  else {
    if (PinA == HIGH) {
      EncoderCount--;
    }
    else {
      EncoderCount++;
    }
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

void WriteDriverVoltage(float V, float Vmax) {    /// eh heya el v??   el vmax = 6 volt
  int PWMval = int(255 * abs(V) / Vmax);
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
  analogWrite(PWMPin, PWMval);
}

void setup() {
  Serial.begin(9600);
  pinMode(interruptPinA, INPUT_PULLUP);
  pinMode(interruptPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPinA), ISR_EncoderA, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(interruptPinB), ISR_EncoderB, CHANGE);
  pinMode(DirPin1, OUTPUT);
  pinMode(DirPin2, OUTPUT);

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
  if (count > count_prev) {
    t = millis();    // b7sb el wa2t mn bdayt el loop
    Theta = EncoderCount / 600.0;    // el one revolution feeha 900 count yb2a a2dr ageeb el theta 
    dt = (t - t_prev);  // theta 3ady
    RPM_d = 120  ;     // desired RPM i want 
    
    RPM = (Theta - Theta_prev) / (dt / 1000.0) * 60;  // b2sm 3la alf 34an a7wl milly to seconds and then * 60 to get per menuite 
    e = RPM_d - RPM;
    inte = inte_prev + (dt * (e + e_prev) / 2);
    V = kp * e + ki * inte + (kd * (e - e_prev) / dt) ;
    if (V > Vmax) {
      V = Vmax;
      inte = inte_prev;
    }
    if (V < Vmin) {
      V = Vmin;
      inte = inte_prev;
    }


    WriteDriverVoltage(V, Vmax);

    Serial.print(RPM_d); Serial.print(" \t");
    Serial.print(RPM); Serial.println(" \t ");
    Serial.print(V); Serial.print("\t  ");
    //Serial.print(e); Serial.println("  ");

    Theta_prev = Theta;
    count_prev = count;
    t_prev = t;
    inte_prev = inte;
    e_prev = e;
  }

}



ISR(TIMER1_COMPA_vect) {
  count++;
  Serial.print(count * 0.05); Serial.print(" \t");
}
