#define BAUD 9600

#include <Servo.h>
int xJoyPin=A1;
int yJoyPin=A0;
int xValue=0,yValue=0;
int outValueX=0,outValueY=0;
int s1pin=3;
int s2pin=5;
int testPin=2;

Servo s1;
Servo s2;

int data;

void setup() {
  // put your setup code here, to run once:
  //pinMode(xJoyPin,INPUT);
  //pinMode(s1pin,OUTPUT);
  s1.attach(s1pin);
  s2.attach(s2pin);
  pinMode(testPin,OUTPUT);

  //Serial.print(xValue);
  Serial.begin(9600);
}

void readData(byte c){
  byte h = c >> 4;
  byte v = c & 15;
  int mag=0;
  for(byte i=0,t=h;t%2==1&&i<2;t>>1,i++){
      mag+=45;
  }
  if(h==0||h==4){
      outValueX=90;      
      //digitalWrite(testPin,HIGH);       
  }
  else if(h<=3){        
      outValueX=90-mag;
      //digitalWrite(testPin,LOW); 
  }
  else{
      outValueX=90+mag;
      //digitalWrite(testPin,LOW);       
  }
  mag=0;
  for(byte i=0,t=v;t%2==1&&i<2;t>>1,i++){
      mag+=45;
  }
  if(v==0||v==4){
      outValueY=91;      
  }
  else if(v>3){
      outValueY=90+mag;
  }
  else if(v<=3){
      outValueY=90-mag;
  }
}

void loop() {
  if(Serial.available()>0){
      byte data= Serial.read();
      //Serial.print(data);      
      readData(data);
  }
  else{ 
      outValueY=91;
      outValueX=90;
  }
  s1.write(outValueX);                  
  s2.write(outValueY);  
  delay(30);
}
