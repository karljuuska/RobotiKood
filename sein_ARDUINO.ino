#include <PID_v1.h>

#define SPD 180
#define PIDLIMIT 120

#define M1A 3
#define M1B 2
#define M2A 5
#define M2B 4

int WAIT_FOR_BTN = 1;
int WAIT_FOR_BN_REL = 2;
int RUNNING = 3;
int STOPPING = 4;
unsigned int state = WAIT_FOR_BTN;

const int switchPin = 9;
int kiirus = SPD;
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint,2,0,0, DIRECT);
boolean car_state=true;
int aeg=0;
int alusta = 0;
void motor(byte which, byte spd, byte dir){
  byte a = M1A;
  byte b = M1B; 
  if (which == 1){
    a = M2A;
    b = M2B;
  }
  
  if (dir == 1){
    analogWrite(a, spd);  
    digitalWrite(b, LOW);
  }
  else{
    analogWrite(a, 255-spd);
  digitalWrite(b, HIGH);
  }
}

int measureDistance(char pin)
{
  pinMode(pin, OUTPUT);

  digitalWrite(pin, HIGH);
  delayMicroseconds(20);
  digitalWrite(pin, LOW);

  pinMode(pin, INPUT);
  int pulse = pulseIn(pin, HIGH, 20000);

  // Chinese timeout fix, yay!
  if (pulse == 0)
  {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delayMicroseconds(20);
    pulse = 25000;
  }

  return (int) pulse / 58;
}


void stoping(){
  motor(0, 0, 1);
  motor(1, 0, 0);
  alusta=0;
}

void setup(){
  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2B, OUTPUT); 
  pinMode(6, INPUT_PULLUP);

  pinMode(switchPin, INPUT);
  digitalWrite(switchPin, HIGH);

  Input = measureDistance(12)-measureDistance(11);
  Setpoint = 1;
  myPID.SetMode(AUTOMATIC);
}

void loop(){

    if(state == WAIT_FOR_BTN){
        if(digitalRead(switchPin) == LOW){
        state = WAIT_FOR_BN_REL;
      }
    }else if(state == WAIT_FOR_BN_REL){
      if(digitalRead(switchPin) == HIGH){
        state = RUNNING;
      }
    }else if(state == RUNNING){

      alusta++;

      if(alusta>=50){
       
      int leftCamera = measureDistance(12);//VASAK
      int rightCamera = measureDistance(11);//PAREM
      int midCamera = measureDistance(10);//KESKMINE

      Serial.println(midCamera);
  
      Input = leftCamera-rightCamera;
      
      
      myPID.SetOutputLimits(-PIDLIMIT, PIDLIMIT);    
      myPID.Compute();
      analogWrite(3,Output);

      if(midCamera <= 5){
         car_state=false;
          motor(0, 100, 0);
          motor(1, 100, 0);
      }
      else if(car_state==true){
        motor(0, constrain((int) (kiirus-Output), 0, 255), 1);
        motor(1, constrain((int) (kiirus+Output), 0, 255), 1);
      }
      else if(car_state==false){
        aeg++;
        motor(0, 180/*constrain((int) (kiirus+Output), 0, 255)*/, 0);
        motor(1, 180/*constrain((int) (kiirus-Output), 0, 255)*/, 0);
        if(aeg==4){
          car_state=true;
          aeg=0;
        }
      }
      }

      if(digitalRead(switchPin) == LOW){
        state = STOPPING;
      }
    }else if(state == STOPPING){
       stoping();
      if(digitalRead(switchPin) == HIGH){
        state = WAIT_FOR_BTN;
      }
    }
  
    delay(100);
   
}

