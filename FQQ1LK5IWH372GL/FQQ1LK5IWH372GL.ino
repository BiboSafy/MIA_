// Pin Definitions
const int INPUT_PIN = A0;   
const int OUTPUT_PIN = 3;   

const int IN1 = 7;
const int IN2 = 6;
const int IN3 = 5;
const int IN4 = 4;
const int ENCODER_PIN = 2;

// PID class for controlling the motor
class PIDController {
  public:
    double kp, ki, kd;      
    double integral, previous_error, last_time;

    PIDController(double p, double i, double d) {
      kp = p;
      ki = i;
      kd = d;
      integral = 0;
      previous_error = 0;
      last_time = millis();
    }

    double compute(double setpoint, double actual) 
    {
      double now = millis();
      double dt = (now - last_time) / 1000.0;  
      if (dt <= 0) return 0;    
      last_time = now;
      double error = setpoint - actual;
      
      // Proportional term
      double proportional = kp * error;
      
      // Integral term
      integral += error * dt;
      double integral_term = ki * integral;

      // Derivative term
      double derivative = (error - previous_error) / dt;
      previous_error = error;
      double derivative_term = kd * derivative;

      // PID output
      double output = proportional + integral_term + derivative_term;
      return constrain(output, 0, 255);  // Constrain output to PWM range
    }
};

PIDController pid(0.8, 0.2, 0.001);  // Tuning
double setpoint = 75.0;              // Motor speed 

// Exponential Smoothing Filter 
double applyExponentialSmoothing(double current_output, double previous_output, double alpha) {
  return (alpha * current_output) + ((1 - alpha) * previous_output);
}

double alpha = 0.1;
double smoothed_output = 0;

// Encoder
volatile int encoderCount = 0;
int previousEncoderCount = 0;
unsigned long lastEncoderReadTime = 0;
const unsigned long encoderReadInterval = 100; // Read encoder every 100 ms

void setup() {
  Serial.begin(9600); 
  pinMode(INPUT_PIN, INPUT);           
  pinMode(OUTPUT_PIN, OUTPUT);  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENCODER_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, CHANGE);

  analogWrite(OUTPUT_PIN, 0);   

  for (int i = 0; i < 50; i++) {
    Serial.print(setpoint);
    Serial.print(",");
    Serial.println(0);
    delay(100);
  }
}

void loop() 
{
  unsigned long currentTime = millis();
  
  if (currentTime - lastEncoderReadTime >= encoderReadInterval) {
    lastEncoderReadTime = currentTime;

    // Calculate actual speed (count per unit time)
    int deltaEncoderCount = encoderCount - previousEncoderCount;
    previousEncoderCount = encoderCount;

    // Assume speed is mapped from encoder count
    double actual_speed = map(deltaEncoderCount, 0, 1023, 0, 255);

    // PID output
    double pid_output = pid.compute(setpoint, actual_speed);

    // Apply exponential smoothing for soft start
    smoothed_output = applyExponentialSmoothing(pid_output, smoothed_output, alpha);

    analogWrite(OUTPUT_PIN, smoothed_output);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    Serial.print("Setpoint: ");
    Serial.print(setpoint);
    Serial.print(", Actual Speed: ");
    Serial.print(actual_speed);
    Serial.print(", PID Output: ");
    Serial.print(pid_output);
    Serial.print(", Smoothed Output: ");
    Serial.println(smoothed_output);
  }

  delay(100); // Small delay to prevent rapid polling
}

void encoderISR() {
  encoderCount++;
}