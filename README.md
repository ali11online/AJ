#include <SoftwareSerial.h>

SoftwareSerial serial(2, 3);                  // RX, TX
int enable1= 10 ;                       //H-bridge pinout
int in_put1 = 9;                     
int  in_put2 = 8;                     
int in_put3 = 7;                    
int in_put4 = 6;                    
int enable2 = 5 ;                    
const int Right = A0;                  //ir sensor LRight for the left side of robot
const int Left = A2;                 //ir sensor Left for the left side of robot

int bluetooth_data;
int Speed = 130;  

int state=0;                                // it will change the state from line follow to app /bluetooth


void setup()
{                                               // put your setup code here, to run once

Serial.begin(9600);                       // start serial communication at 9600bps
serial.begin(9600); 

pinMode(enable1, OUTPUT); 
pinMode(in_put1, OUTPUT); 
pinMode(in_put3, OUTPUT); 
pinMode(in_put4, OUTPUT); 
pinMode(in_put2 , OUTPUT); 
pinMode(enable2, OUTPUT); 
pinMode(Right, INPUT);  
pinMode(Left, INPUT); 


delay(500);
}


void loop(){  
  
if(serial.available() > 0){            //if  data is sent from app to bluetooth ,then it reads it and saves in states     

bluetooth_data = serial.read();      

if(bluetooth_data > 25)
{
  Speed = bluetooth_data;
  }      
}

     if(bluetooth_data== 8)           //Auto Line Follower Command
     {
      state=1; 
      Speed=150;
      }   
else if(bluetooth_data == 9)         //Manual Android Application Control Command
{
  state=0; 
  Stop();
  }    

analogWrite(enable1, Speed);             // you may control the speed of  0 to 255 to both  Enable pins for Motor1 Speed 
analogWrite(enable2, Speed); 

if(state==0){     

     if(bluetooth_data == 1)               // if bluetooth_data is 1 the DC motor will go forwar
     {
      forword();
      }  
else if(bluetooth_data == 2)              // if bluetooth_data is 2 the DC motor will go Reverse
{
  backword();
  }  
else if(bluetooth_data == 3)             // if bluetooth_data is 3 the DC motor will go turn left
{
  turnLeft();
  }  
else if(bluetooth_data == 4)            // if bluetooth_data is 4 the DC motor will go turn right
{
  turnRight();
  } 
else if(bluetooth_data== 5)             // if bluetooth_data is 5 the DC motor will  Stop
{
  Stop(); 
  }     
   
if((digitalRead(Right) == 0)&&(digitalRead(Left) == 0))     //if Right Sensor and Left Sensor are not detect black line
{
  forword();                                                     // it will be move forward
  
  } 


if((digitalRead(Right) == 1)&&(digitalRead(Left) == 0))     //if Right Sensor is detect Black and Left Sensor is not detect
{
  turnRight();                                                       // it will be move turn right
  }  
if((digitalRead(Right) == 0)&&(digitalRead(Left) == 1))    //if Right Sensor is not detect black and Left Sensor is detect Black 
{
  turnLeft();                                                        // it will be move turn left
  } 
if((digitalRead(Right) == 1)&&(digitalRead(Left) == 1))    //if Right Sensor and Left Sensor are both detect Black color 
{
  Stop();}                                                           // it will stop
} 

delay(10);
}

void forword()                 //to move your robot in forword direction
{ 
digitalWrite(in_put1, HIGH); 
digitalWrite(in_put2, LOW);    // for forward one right side  and left side motor will be on 
digitalWrite(in_put3, LOW);  
digitalWrite(in_put4, HIGH);  
}

void backword(){               //to move your robot in reversie backword
digitalWrite(in_put1, LOW);  
digitalWrite(in_put2, HIGH);       // for backward one right side  and left side motor will be on 
digitalWrite(in_put3, HIGH); 
digitalWrite(in_put4, LOW);  
}

void turnRight(){             //turn or rotate your robot to turnRight
digitalWrite(in_put1, LOW);  
digitalWrite(in_put2, HIGH); 
digitalWrite(in_put3, LOW);  // for turn left the right side two motor will be off 
digitalWrite(in_put4, HIGH);  
}

void turnLeft(){               //turn or rotate your robot to Left
digitalWrite(in_put1, HIGH); 
digitalWrite(in_put2, LOW);   
digitalWrite(in_put3, HIGH);  // for turn left the left side two motor will be off 
digitalWrite(in_put4, LOW);  
}

void Stop()
{                             //to stop the robot ,here all the motors goes off 
digitalWrite(in_put1, LOW); 
digitalWrite(in_put2, LOW);   // so for stop all the motor goes off 
digitalWrite(in_put3, LOW); 
digitalWrite(in_put4, LOW);  
}
