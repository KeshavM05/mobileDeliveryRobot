/**
 * @file motor-angular-rate.ino
 * @author Joshua Marshall (joshua.marshall@queensu.ca)
 * @brief Arduino program to estimate motor speed from encoder.
 * @version 2.1
 * @date 2022-12-09
 *
 * @copyright Copyright (c) 2021-2022
 *
 */

// Wheel PWM pin (must be a PWM pin)
int EA = 2;
int EB = 5;

// Wheel direction digital pins
int I1 = 3;
int I2 = 4;

int I3 = 6;
int I4 = 7;

// Motor PWM command variable [0-255]
byte u = 0;

// Left wheel encoder digital pins
const byte SIGNAL_LA = 10;
const byte SIGNAL_LB = 11;

// Right wheel encoder digital pins
const byte SIGNAL_RA = 8;
const byte SIGNAL_RB = 9;

// Encoder ticks per (motor) revolution (TPR)
const int TPR = 3000;

// Wheel radius [m]
const double RHO = 0.0625;

// Counter to keep track of encoder ticks [integer]
volatile long left_encoder_ticks = 0;
volatile long right_encoder_ticks = 0;

// Variable to store estimated angular rate of left wheel [rad/s]
double omega_L = 0.0;

// Variable to store estimated angular rate of right wheel [rad/s]
double omega_R = 0.0;

// Sampling interval for measurements in milliseconds
const int T = 1000;

// Counters for milliseconds during interval
long t_now = 0;
long t_last = 0;

// This function is called when SIGNAL_LA goes HIGH
void decodeLeftEncoderTicks()
{
    if (digitalRead(SIGNAL_LB) == LOW)
    {
        // SIGNAL_LA leads SIGNAL_LB, so count one way
        left_encoder_ticks--;
    }
    else
    {
        // SIGNAL_LB leads SIGNAL_LA, so count the other way
        left_encoder_ticks++;
    }
}

// This function is called when SIGNAL_RA goes HIGH
void decodeRightEncoderTicks()
{
    if (digitalRead(SIGNAL_RB) == LOW)
    {
        // SIGNAL_RA leads SIGNAL_RB, so count one way
        right_encoder_ticks--;
    }
    else
    {
        // SIGNAL_RB leads SIGNAL_RA, so count the other way
        right_encoder_ticks++;
    }
}

void setup()
{
    // Open the serial port at 9600 bps
    Serial.begin(9600);

    // Set the pin modes for the motor driver
    pinMode(EA, OUTPUT);
    pinMode(I1, OUTPUT);
    pinMode(I2, OUTPUT);
    pinMode(EB, OUTPUT);
    pinMode(I3, OUTPUT);
    pinMode(I4, OUTPUT);

    // Set the pin modes for the encoders
    pinMode(SIGNAL_LA, INPUT);
    pinMode(SIGNAL_LB, INPUT);

    // Set the pin modes for the encoders
    pinMode(SIGNAL_RA, INPUT);
    pinMode(SIGNAL_RB, INPUT);

    // Every time the pin goes high, this is a pulse
    attachInterrupt(digitalPinToInterrupt(SIGNAL_LA), decodeLeftEncoderTicks, RISING);
    attachInterrupt(digitalPinToInterrupt(SIGNAL_RA), decodeRightEncoderTicks, RISING);

    // Print a message
    Serial.print("Program initialized.");
    Serial.print("\n");
}

void loop()
{
    // Get the elapsed time [ms]
    t_now = millis();

    if (t_now - t_last >= T)
    {
        // Estimate the rotational speed [rad/s]
        omega_L = 2.0 * PI * ((double)left_encoder_ticks / (double)TPR) * 1000.0 / (double)(t_now - t_last);
        omega_R = 2.0 * PI * ((double)right_encoder_ticks / (double)TPR) * 1000.0 / (double)(t_now - t_last);

        // Print some stuff to the serial monitor
        Serial.print("Left Encoder ticks: ");
        Serial.print(left_encoder_ticks);
        Serial.print("\n");
        Serial.print("Estimated left wheel speed: ");
        Serial.print(omega_L);
        Serial.print(" rad/s");
        Serial.print("\n");
        Serial.print("Right Encoder ticks: ");
        Serial.print(right_encoder_ticks);
        Serial.print("\n");
        Serial.print("Estimated right wheel speed: ");
        Serial.print(omega_R);
        Serial.print(" rad/s");
        Serial.print("\n\n");

        // Record the current time [ms]
        t_last = t_now;

        // Reset the encoder ticks counter
        left_encoder_ticks = 0;
        right_encoder_ticks = 0;
    }

    // Set the wheel motor PWM command [0-255]
    u = 128;

    // Select a direction
    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);
    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);

    // PWM command to the motor driver
    analogWrite(EA, u);
    analogWrite(EB, u);
}
