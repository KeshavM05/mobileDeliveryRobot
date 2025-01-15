//Code to test the direction of mobileDeliveryRobot

int pinI1=1;//define I1 port
int pinI2=2;//define I2 port
int pinI3=4;//define I3 port
int pinI4=8;//define I4 port
int speedpinA=5;//define EA(PWM speed regulation)port
int speedpinB=6;//define EB(PWM speed regulation)port
void setup()
{
pinMode(pinI1,OUTPUT);//define this port as output
pinMode(pinI2,OUTPUT);//define this port as output
pinMode(pinI3,OUTPUT);//define this port as output
pinMode(pinI4,OUTPUT);//define this port as output
pinMode(speedpinA,OUTPUT);
pinMode(speedpinB,OUTPUT);
}
void loop()
{
//Forward
analogWrite(speedpinA,100);//input a value to set the speed
analogWrite(speedpinB,100);//input a value to set the speed
delay(2000);
digitalWrite(pinI1,HIGH);
digitalWrite(pinI2,LOW);
digitalWrite(pinI3,LOW);
digitalWrite(pinI4,HIGH);

//Backward
analogWrite(speedpinA,100);//input a value to set the speed
analogWrite(speedpinB,100);//input a value to set the speed
delay(2000);
digitalWrite(pinI1,LOW);
digitalWrite(pinI2,HIGH);
digitalWrite(pinI3,HIGH);
digitalWrite(pinI4,LOW);

//Turn Left
analogWrite(speedpinA,100);//input a value to set the speed
analogWrite(speedpinB,100);//input a value to set the speed
delay(2000);
digitalWrite(pinI1,HIGH);
digitalWrite(pinI2,LOW);
digitalWrite(pinI3,HIGH);
digitalWrite(pinI4,LOW);

//Turn Right
analogWrite(speedpinA,100);//input a value to set the speed
analogWrite(speedpinB,100);//input a value to set the speed
delay(2000);
digitalWrite(pinI1,LOW);
digitalWrite(pinI2,HIGH);
digitalWrite(pinI3,LOW);
digitalWrite(pinI4,HIGH);

}

