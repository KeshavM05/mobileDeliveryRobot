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

#include <Arduino_LSM6DS3.h>

// Variables to store angular rates from the gyro [degrees/s]
float omega_x, omega_y, omega_z;

// Variables to store gyro bias values [degrees/s]
float omega_x_bias = 0.21;
float omega_y_bias = 0.091;
float omega_z_bias = -0.34;

// Variables to store accelerations [g's]
float a_x, a_y, a_z;

// Variables to store sample rates from sensor [Hz]
float a_f, g_f;

// Wheel PWM pin (must be a PWM pin)
int EA = 2;
int EB = 5;

// Wheel direction digital pins
int I1 = 3, I2 = 4;
int I3 = 6, I4 = 7;

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

// Estimated angular rates and translational speeds
double omega_L = 0.0, omega_R = 0.0;
double v_L = 0.0, v_R = 0.0;

// Sampling interval for measurements in milliseconds
const int T = 1000;

// Vehicle's track length
const double ELL = 0.2775;

// Counters for milliseconds during interval
long t_now = 0, t_last = 0;

// Interrupt service routines for encoders
void decodeLeftEncoderTicks() {
    left_encoder_ticks += (digitalRead(SIGNAL_LB) == LOW) ? -1 : 1;
}

void decodeRightEncoderTicks() {
    right_encoder_ticks += (digitalRead(SIGNAL_RB) == LOW) ? -1 : 1;
}

// Compute vehicle speed [m/s]
double compute_vehicle_speed(double v_L, double v_R) {
    return 0.5 * (v_L + v_R);
}

// Compute vehicle turning rate [rad/s]
double compute_vehicle_rate(double v_L, double v_R) {
    return (v_R - v_L) / ELL;
}

void setup() {
    Serial.begin(9600);
    while (!Serial) {
        delay(10);
    }

    // Motor driver pin modes
    pinMode(EA, OUTPUT);
    pinMode(EB, OUTPUT);
    pinMode(I1, OUTPUT);
    pinMode(I2, OUTPUT);
    pinMode(I3, OUTPUT);
    pinMode(I4, OUTPUT);

    // Encoder pin modes
    pinMode(SIGNAL_LA, INPUT);
    pinMode(SIGNAL_LB, INPUT);
    pinMode(SIGNAL_RA, INPUT);
    pinMode(SIGNAL_RB, INPUT);

    attachInterrupt(digitalPinToInterrupt(SIGNAL_LA), decodeLeftEncoderTicks, RISING);
    attachInterrupt(digitalPinToInterrupt(SIGNAL_RA), decodeRightEncoderTicks, RISING);

    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU :(");
        while (1) delay(10);
    }

    Serial.println("Program initialized.");
}

void loop() {
    t_now = millis();

    if (t_now - t_last >= T) {
        // Estimate the rotational speed [rad/s]
        omega_L = 2.0 * PI * (left_encoder_ticks / (double)TPR) * 1000.0 / (t_now - t_last);
        omega_R = 2.0 * PI * (right_encoder_ticks / (double)TPR) * 1000.0 / (t_now - t_last);

        // Estimate the translational speed [m/s]
        v_L = RHO * omega_L;
        v_R = RHO * omega_R;

        Serial.print("Estimated left wheel speed: ");
        Serial.print(omega_L);
        Serial.println(" rad/s");
        Serial.print("Estimated left translational speed: ");
        Serial.print(v_L);
        Serial.println(" m/s");

        Serial.print("Estimated right wheel speed: ");
        Serial.print(omega_R);
        Serial.println(" rad/s");
        Serial.print("Estimated right translational speed: ");
        Serial.print(v_R);
        Serial.println(" m/s\n");

        t_last = t_now;
        left_encoder_ticks = 0;
        right_encoder_ticks = 0;

        if (IMU.accelerationAvailable()) {
            IMU.readAcceleration(a_x, a_y, a_z);
            Serial.print(a_x); Serial.print("\t");
            Serial.print(a_y); Serial.print("\t");
            Serial.print(a_z); Serial.print(" g\t\t");
        } else {
            Serial.println("IMU Acceleration NOT WORKING");
        }

        if (IMU.gyroscopeAvailable()) {
            IMU.readGyroscope(omega_x, omega_y, omega_z);
            Serial.print(omega_x - omega_x_bias); Serial.print("\t");
            Serial.print(omega_y - omega_y_bias); Serial.print("\t");
            Serial.print(omega_z - omega_z_bias); Serial.println(" deg/s\n");
        } else {
            Serial.println("IMU Gyroscope NOT WORKING");
        }
    }

    // Set motor speed and direction
    u = 228;
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);
    analogWrite(EA, u);
    analogWrite(EB, u);
}
