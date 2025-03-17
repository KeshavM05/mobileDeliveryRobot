// Left wheel encoder digital pins
const byte SIGNAL_LA = 10;
const byte SIGNAL_LB = 11;

// Right wheel encoder digital pins
const byte SIGNAL_RA = 8;
const byte SIGNAL_RB = 9;

// Encoder ticks per revolution
const int TPR = 3000;

// Wheel radius [m]
const double RHO = 0.0625;

// Encoder tick counters
volatile long left_encoder_ticks = 0;
volatile long right_encoder_ticks = 0;

// Variables to store speeds
double omega_L = 0.0;
double omega_R = 0.0;
double v_L = 0.0;
double v_R = 0.0;

// Sampling interval [ms]
const int T = 1000;
long t_now = 0;
long t_last = 0;

// Interrupt service routines for encoders
void decodeLeftEncoderTicks() {
    if (digitalRead(SIGNAL_LB) == LOW) left_encoder_ticks--;
    else left_encoder_ticks++;
}

void decodeRightEncoderTicks() {
    if (digitalRead(SIGNAL_RB) == LOW) right_encoder_ticks--;
    else right_encoder_ticks++;
}

void setup() {
    Serial.begin(9600);

    pinMode(SIGNAL_LA, INPUT);
    pinMode(SIGNAL_LB, INPUT);
    pinMode(SIGNAL_RA, INPUT);
    pinMode(SIGNAL_RB, INPUT);

    attachInterrupt(digitalPinToInterrupt(SIGNAL_LA), decodeLeftEncoderTicks, RISING);
    attachInterrupt(digitalPinToInterrupt(SIGNAL_RA), decodeRightEncoderTicks, RISING);
}

void loop() {
    t_now = millis();
    if (t_now - t_last >= T) {
        omega_L = 2.0 * PI * ((double)left_encoder_ticks / TPR) * 1000.0 / (t_now - t_last);
        omega_R = 2.0 * PI * ((double)right_encoder_ticks / TPR) * 1000.0 / (t_now - t_last);
        v_L = RHO * omega_L;
        v_R = RHO * omega_R;

        Serial.print(v_L);
        Serial.print(",");
        Serial.println(v_R);

        left_encoder_ticks = 0;
        right_encoder_ticks = 0;
        t_last = t_now;
    }
}
