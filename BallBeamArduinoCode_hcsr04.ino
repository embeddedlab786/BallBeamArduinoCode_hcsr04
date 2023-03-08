#include<Servo.h>
#include<PID_v1.h>

#define echopin 6 // echo pin
#define trigpin 7 // Trigger pin

const int servoPin = 2;                                               //Servo Pin
 
float Kp = 0.5;                                                    //Initial Proportional Gain
float Ki = 0.0;                                                      //Initial Integral Gain
float Kd = 0.1;                                                    //Intitial Derivative Gain
double Setpoint, Input, Output, ServoOutput;                                       

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);           //Initialize PID object, which is in the class PID.
Servo myServo;   //Initialize Servo.

void setup() {
  Serial.begin(9600);                                                //Begin Serial 
  myServo.attach(servoPin);                                          //Attach Servo

  pinMode (trigpin, OUTPUT);
  pinMode (echopin, INPUT);

  Input = readPosition();                                            //Calls function readPosition() and sets the balls
                                                                     //  position as the input to the PID algorithm 
  myPID.SetMode(AUTOMATIC);                                          //Set PID object myPID to AUTOMATIC 
  myPID.SetOutputLimits(-40,40);                                     //Set Output limits to -80 and 80 degrees. 
}

void loop(){
 int adc = analogRead(A0);
 Setpoint = map(adc,0,1023,10,60);
 Input = readPosition();                                             
 myPID.Compute();                                                   //computes Output in range of -80 to 80 degrees
 ServoOutput=102+Output;                                            // 102 degrees is my horizontal 
 myServo.write(ServoOutput);                                        //Writes value of Output to servo
 }
   
float readPosition() {
delay(80);  //Don't set too low or echos will run into eachother.      
  
long duration, cm;
unsigned long now = millis();
 digitalWrite(trigpin,LOW);
 delayMicroseconds(2);
 digitalWrite(trigpin,HIGH);
 delayMicroseconds(10);
 duration = pulseIn (echopin,HIGH);
 cm = duration/ 29 / 2; 
  if(cm > 80)    // 30 cm is the maximum position for the ball
  {cm=80;}
  
  Serial.println(cm); 
  return cm;    //Returns distance value.
}
