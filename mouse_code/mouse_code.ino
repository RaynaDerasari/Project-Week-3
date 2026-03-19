// Pin assignments
#define sensor_left A0
#define sensor_right A1

#define motor_left 10
#define motor_right 3

// PID constants
float kp_value = 2.0;
float ki_value = 0.05;
float kd_value = 1.5;

// Threshold for sensor sum
int threshold = 10;

// Sensor readings and PID variables
float sensor_left_val = 0;
float sensor_right_val = 0;
unsigned long prev_time = 0;
float current_error, cumulative_error, prev_error = 0;

void setup() {
  pinMode(motor_left, OUTPUT);
  pinMode(motor_right, OUTPUT);

  Serial.begin(9600);
  Serial.println("---- Debugging Lines ----");
}

void loop() {
  // Read sensors
  sensor_left_val = analogRead(sensor_left);
  sensor_right_val = analogRead(sensor_right);

  // Print sensor values
  Serial.print("L: ");
  Serial.print(sensor_left_val);
  Serial.print(" | R: ");
  Serial.print(sensor_right_val);

  // Threshold check
  int sensor_sum = sensor_left_val + sensor_right_val;
  Serial.print(" | Sum: ");
  Serial.print(sensor_sum);

  if (sensor_sum < threshold) {
    Serial.println(" | STOP (below threshold)");

    analogWrite(motor_left, 0);
    analogWrite(motor_right, 0);
    return;
  }

  // Compute PID
  float pid_signal = compute_pid(sensor_left_val, sensor_right_val);

  // Print PID info
  Serial.print(" | Error: ");
  Serial.print(current_error);
  Serial.print(" | PID: ");
  Serial.print(pid_signal);

  // Motor outputs (no base speed)
  int pwm_left = constrain(pid_signal, 0, 255);
  int pwm_right = constrain(-pid_signal, 0, 255);

  Serial.print(" | PWM_L: ");
  Serial.print(pwm_left);
  Serial.print(" | PWM_R: ");
  Serial.print(pwm_right);

  Serial.println(); // new line

  // Drive motors
  analogWrite(motor_left, pwm_left);
  analogWrite(motor_right, pwm_right);

  delay(50); // slow down output as needed so you can read it
}

// PID function
float compute_pid(float left_val, float right_val) {
  unsigned long current_time = millis();
  float elapsed_time = (float)(current_time - prev_time) / 1000.0;

  if (elapsed_time <= 0) return 0;

  current_error = left_val - right_val;
  cumulative_error += current_error * elapsed_time;
  float rate_error = (current_error - prev_error) / elapsed_time;

  float pid_output = (kp_value * current_error) +
                     (ki_value * cumulative_error) +
                     (kd_value * rate_error);

  prev_error = current_error;
  prev_time = current_time;

  return pid_output;
}