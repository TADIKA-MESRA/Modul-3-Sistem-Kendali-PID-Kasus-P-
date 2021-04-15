int trigger = 5;
int echo = 6;

//motor directory
#define CW  0
#define CCW 1
 
//motor control pin
#define motorDirPin 7
#define motorPWMPin 9
#define enablePin 8
 
//encoder pin
#define encoderPinA 2
#define encoderPinB 4
 
//encoder var
int encoderPos = 0;
 
//P control
float Kp          = 5;
float Ki          = 0;
float Kd          = 0;
int   targetPos;
int   error;
int   prevError;
float integral;
float derivative;
float dt = 0.01;//10 milisecond
int   control;
int   velocity;
 
//external interrupt encoder
void doEncoderA()
{
  digitalRead(encoderPinB)?encoderPos--:encoderPos++;
}
 
void setup()
{
  //setup interrupt
    pinMode(encoderPinA, INPUT_PULLUP);
    pinMode(encoderPinB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderA,RISING);
   
    //setup motor driver
    pinMode(motorDirPin, OUTPUT);
    pinMode(enablePin, OUTPUT);
    pinMode(trigger,OUTPUT);
    pinMode(echo,INPUT);
    digitalWrite(enablePin, HIGH);
   
    Serial.begin(9600);
}
 
void loop()
{
    targetPos = analogRead(A0)/10; //ADC max 1023
    error = targetPos - encoderPos;
    integral += error * dt;
    derivative = (error - prevError)/dt;
    control = (Kp*error) + (Ki*integral) + (Kd*derivative);
   
    velocity = min(max(control, -255), 255);
    if(velocity >= 0)
    {
        digitalWrite(motorDirPin, CW);
        analogWrite(motorPWMPin, velocity); 
    }
    else
    {
        digitalWrite(motorDirPin, CCW);
        analogWrite(motorPWMPin, 255+velocity);
    }
    //Serial.println(encoderPos);
    prevError = error;
    delay(dt*1000);             
}
