// Wheel PWM pin (must be a PWM pin)
int EA = 2;
int EB = 5;

// Wheel direction digital pins
int I1 = 3;
int I2 = 4;
int I3 = 6;
int I4 = 7;

// Motor PWM command variable [0-255]
byte u_L = 0, u_R = 0;

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

// Encoder tick counters
volatile long left_encoder_ticks = 0;
volatile long right_encoder_ticks = 0;

// Wheel speeds
double omega_L = 0.0, omega_R = 0.0;
double v_L = 0.0, v_R = 0.0;

// Desired speed and turning rate
double v_d = 0.0, omega_d = 0.0;

// PID errors
double e_L = 0.0, e_R = 0.0;
double e_int_L = 0.0, e_int_R = 0.0;

// PID gains
double k_P = 10.0, k_I = 1.0;

// Track width
const double ELL = 0.2775;

// Time variables
const int T = 100;
long t_now = 0, t_last = 0;

// Interrupt functions for encoders
void decodeLeftEncoderTicks() {
    left_encoder_ticks += (digitalRead(SIGNAL_LB) == LOW) ? -1 : 1;
}

void decodeRightEncoderTicks() {
    right_encoder_ticks += (digitalRead(SIGNAL_RB) == LOW) ? -1 : 1;
}

// PID Controller Function
short PI_controller(double e_now, double e_int, double k_P, double k_I) {
    short u = (short)(k_P * e_now + k_I * e_int);
    return constrain(u, -255, 255);
}

void setup() {
    Serial.begin(115200);

    // Motor driver pin setup
    pinMode(EA, OUTPUT);
    pinMode(I1, OUTPUT);
    pinMode(I2, OUTPUT);
    pinMode(EB, OUTPUT);
    pinMode(I3, OUTPUT);
    pinMode(I4, OUTPUT);

    // Encoder pin setup
    pinMode(SIGNAL_LA, INPUT);
    pinMode(SIGNAL_LB, INPUT);
    pinMode(SIGNAL_RA, INPUT);
    pinMode(SIGNAL_RB, INPUT);

    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(SIGNAL_LA), decodeLeftEncoderTicks, RISING);
    attachInterrupt(digitalPinToInterrupt(SIGNAL_RA), decodeRightEncoderTicks, RISING);

    Serial.println("Arduino Initialized");
}

void loop() {
    t_now = millis();

    if (Serial.available() > 0) {
        // Read serial input from Raspberry Pi
        v_d = Serial.parseFloat();
        omega_d = Serial.parseFloat();
    }

    if (t_now - t_last >= T) {
        // Compute wheel speeds
        omega_L = 2.0 * PI * (double)left_encoder_ticks / (double)TPR * 1000.0 / (t_now - t_last);
        omega_R = 2.0 * PI * (double)right_encoder_ticks / (double)TPR * 1000.0 / (t_now - t_last);

        v_L = RHO * omega_L;
        v_R = RHO * omega_R;

        left_encoder_ticks = 0;
        right_encoder_ticks = 0;
        t_last = t_now;

        // Compute desired wheel speeds
        double v_Ld = v_d - (0.5 * ELL * omega_d);
        double v_Rd = v_d + (0.5 * ELL * omega_d);

        // Compute PID errors
        e_L = v_Ld - v_L;
        e_R = v_Rd - v_R;
        e_int_L += e_L * (T / 1000.0);
        e_int_R += e_R * (T / 1000.0);

        // Compute motor commands
        u_L = PI_controller(e_L, e_int_L, k_P, k_I);
        u_R = PI_controller(e_R, e_int_R, k_P, k_I);

        // Set motor directions
        digitalWrite(I1, u_L >= 0);
        digitalWrite(I2, u_L < 0);
        digitalWrite(I3, u_R >= 0);
        digitalWrite(I4, u_R < 0);

        // Send PWM commands
        analogWrite(EA, abs(u_L));
        analogWrite(EB, abs(u_R));

        // Send feedback to Raspberry Pi
        Serial.print(v_L);
        Serial.print(",");
        Serial.println(v_R);
    }
}
