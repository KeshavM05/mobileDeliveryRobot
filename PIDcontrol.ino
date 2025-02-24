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

// Variable to store estimated angular wheel speeds [rad/s]
double omega_L = 0.0;
double omega_R = 0.0;

// Variable to store estimated wheel speeds [m/s]
double v_L = 0.0;
double v_R = 0.0;

// Variables to store vehicle speed and turning rate
double v = 0.0;   // [m/s]
double omega = 0.0; // [rad/s]

// Variables to store desired vehicle speed and turning rate
double v_d = 0.0;   // [m/s]
double omega_d = 0.0; // [rad/s]

// Variable to store desired wheel speeds [m/s]
double v_Ld = 0.0;
double v_Rd = 0.0;

// Sampling interval for measurements in milliseconds
const int T = 1000;

// Variable to store vehicle's track length
const double ELL = 0.2775;

// Counters for milliseconds during interval
long t_now = 0;
long t_last = 0;

// Variables to store errors for controller
double e_now = 0.0;
double e_now = 0.0;
double e_int = 0.0;
double e_int = 0.0;


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

// Compute vehicle speed [m/s]
double compute_vehicle_speed(double v_L, double v_R)
{
  double v;
  v = 0.5 * (v_L + v_R);
  return v;
}
// Compute vehicle turning rate [rad/s]
double compute_vehicle_rate(double v_L, double v_R)
{
  double omega;
  omega = 1.0 / ELL * (v_R - v_L);
  return omega;
}

// Compute left wheel speed [m/s]
double compute_left_wheel_speed(double v, double omega)
{
  double v_L = v - (0.5 * ELL * omega);
  return v_L;
}

// Compute right wheel speed [m/s]
double compute_right_wheel_speed(double v, double omega)
{
  double v_R = v + (0.5 * ELL * omega);
  return v_R;
}

// Wheel speed PI controller function
short PI_controller(double e_now, double e_int, double k_P, double k_I)
{
  short u;
  u = (short)(k_P * e_now + k_I * e_int);

  if (u > 255)
  {
    u = 255;
  }
  else if (u < -255)
  {
    u = -255;
  }
  return u;
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

        // Estimate the translational speed [m/s]
        v_L = RHO * omega_L;
        v_R = RHO * omega_R;

        // Record the current time [ms]
        t_last = t_now;

        // Reset the encoder ticks counter
        left_encoder_ticks = 0;
        right_encoder_ticks = 0;
    }

    // Set the wheel motor PWM command [0-255]
    u = 228;

    // Select a direction
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);

    // PWM command to the motor driver
    analogWrite(EA, u);
    analogWrite(EB, u);
}