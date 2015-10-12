#include <Servo.h>

Servo pan;
Servo tilt;

int pan_pos;
int tilt_pos;
int movement=5;  //Amount of angle change for each movement 

void setup(){
    Serial.begin(9600);
    pan.attach(11);    // Change this port number if your are using other port 
    tilt.attach(10);   // Change this port number if your are using other port
    
    //Initial position of pan-tilt motor
    pan_pos=90;
    tilt_pos=90;
    pan.write(pan_pos);
    tilt.write(tilt_pos);
}

void loop(){

    if(Serial.available()){   //Check is any input from raspberry pi

    char c=Serial.read();
    if(c=='a')  // move to LEFT  when 'a' is pressed
    {
      pan_pos +=movement; 
      pan.write(pan_pos);
    }
    
     if(c=='d')// move to RIGHT  when 'd' is pressed
    {
      pan_pos -=movement; 
      pan.write(pan_pos);
    }
    
      if(c=='s')  // move to DOWN  when 's' is pressed
    {
      tilt_pos +=movement; 
      tilt.write(tilt_pos);
    }
    
     if(c=='w')  // move to UP when 'w' is pressed
    {
      tilt_pos -=movement; 
      tilt.write(tilt_pos);
    } 
        
   }
    delay(10);
    
}

