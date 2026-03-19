// Pin assignments
#define sensor_left A0
#define sensor_right A1

#define motor_left 10
#define motor_right 3

// PID constants
float kp_value = 2.0;
float ki_value = 0.01;
float kd_value = 0.5;

// Threshold for sensor sum
int threshold = 50; // adjust based on your sensors

// Sensor readings and PID variables
float sensor_left_val = 0;
float sensor_right_val = 0;
unsigned long prev_time = 0;
float current_error, cumulative_error, prev_error = 0;

void setup() {
  pinMode(motor_left, OUTPUT);
  pinMode(motor_right, OUTPUT);
}

void loop() {
  // Read sensors
  sensor_left_val = analogRead(sensor_left);
  sensor_right_val = analogRead(sensor_right);

  // Stop motors if sensor sum is below threshold
  if ((sensor_left_val + sensor_right_val) < threshold) {
    analogWrite(motor_left, 0);
    analogWrite(motor_right, 0);
    return; // skip PID
  }

  // Compute PID
  float pid_signal = compute_pid(sensor_left_val, sensor_right_val);

  // Constrain PWM to valid range (0-255)
  int pwm_left = constrain(pid_signal, 0, 255);
  int pwm_right = constrain(-pid_signal, 0, 255);

  // Drive motors
  analogWrite(motor_left, pwm_left);
  analogWrite(motor_right, pwm_right);

  delay(10);
}

// PID computation function
float compute_pid(float left_val, float right_val) {
  unsigned long current_time = millis();
  float elapsed_time = (float)(current_time - prev_time) / 1000.0; // seconds

  if (elapsed_time <= 0) return 0;

  current_error = left_val - right_val;
  cumulative_error += current_error * elapsed_time;
  float rate_error = (current_error - prev_error) / elapsed_time;

  float pid_output = (kp_value * current_error) + (ki_value * cumulative_error) + (kd_value * rate_error);

  prev_error = current_error;
  prev_time = current_time;

  return pid_output;
}