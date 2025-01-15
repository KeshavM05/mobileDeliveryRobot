//Code to test the direction of mobileDeliveryRobot

int I1=1;//define I1 port
int I2=2;//define I2 port
int I3=4;//define I3 port
int I4=8;//define I4 port
int EA=5;//define EA(PWM speed regulation)port
int EB=6;//define EB(PWM speed regulation)port
void setup()
{
pinMode(I1,OUTPUT);//define this port as output
pinMode(I2,OUTPUT);//define this port as output
pinMode(I3,OUTPUT);//define this port as output
pinMode(I4,OUTPUT);//define this port as output
pinMode(EA,OUTPUT);
pinMode(EB,OUTPUT);
}
void loop()
{
  int u; // A variable for the motor PWM command [0-255]

    // Play with this code to write open loop commands to a wheel motor
    for (u = 125; u <= 255; u += 125)
    {
      //Forward
      analogWrite(EA,u);//input a value to set the speed
      analogWrite(EB,u);//input a value to set the speed
      digitalWrite(I1,HIGH);
      digitalWrite(I2,LOW);
      digitalWrite(I3,LOW);
      digitalWrite(I4,HIGH);
      delay(2000);
      
      //Backward
      analogWrite(EA,u);//input a value to set the speed
      analogWrite(EB,u);//input a value to set the speed
      digitalWrite(I1,LOW);
      digitalWrite(I2,HIGH);
      digitalWrite(I3,HIGH);
      digitalWrite(I4,LOW);
      delay(2000);
      
      //Turn Left
      analogWrite(EA,u);//input a value to set the speed
      analogWrite(EB,u);//input a value to set the speed
      digitalWrite(I1,HIGH);
      digitalWrite(I2,LOW);
      digitalWrite(I3,HIGH);
      digitalWrite(I4,LOW);
      delay(2000);
      
      //Turn Right
      analogWrite(EA,u);//input a value to set the speed
      analogWrite(EB,u);//input a value to set the speed
      
      digitalWrite(I1,LOW);
      digitalWrite(I2,HIGH);
      digitalWrite(I3,LOW);
      digitalWrite(I4,HIGH);
      delay(2000);
    }

}
