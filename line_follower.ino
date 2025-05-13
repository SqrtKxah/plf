#include <PID_v1.h>
#include <math.h>

#define left_spd_pin 3
#define right_spd_pin 2
#define left_mtr_forward 33
#define left_mtr_back 35
#define right_mtr_forward 37
#define right_mtr_back 39

double input = 0;
double output = 0;
double setpoint = 0;
double kp = 6, ki = 0.2, kd = 1;
double reduction_rate = 45;

int sensor_pins[5] = {23, 25, 27, 29, 31}; // pinos
float sensor_values[5] = {-4, -2, 0, 2, 4}; // valores de peso agregados aos sensores

PID speed_pid(&input, &output, &setpoint, kp, ki, kd, DIRECT);

int invert(int num) {
  return pow(num, 0) - num;
}

void setup() {
  Serial.begin(1200);
  pinMode(left_mtr_forward, OUTPUT);
  pinMode(left_mtr_back, OUTPUT);
  pinMode(right_mtr_forward, OUTPUT);
  pinMode(right_mtr_back, OUTPUT);
  pinMode(left_spd_pin, OUTPUT);
  pinMode(right_spd_pin, OUTPUT);
  for (int i = 0; i <= 4; i++) {
    pinMode(sensor_pins[i], INPUT);
  }
  speed_pid.SetMode(AUTOMATIC);
  speed_pid.SetOutputLimits(-8, 8);
  digitalWrite(left_mtr_forward, LOW);
  digitalWrite(right_mtr_forward, LOW);
  digitalWrite(left_mtr_back, HIGH);
  digitalWrite(right_mtr_back, HIGH);
}

void loop() {
  double max_spd = 255;
  double left_spd = max_spd;
  double right_spd = max_spd;
  input = 0;

  // ler sensor, calcular soma de erro e calcular velocidade
  for (int i = 0; i <= 4; i++) {
    int s_input = invert(digitalRead(sensor_pins[i]));
    input += s_input * sensor_values[i];
    // Serial.println(i);
    // Serial.print(sensor_pins[i]);
    // Serial.println(i);
    // Serial.print(sensor_values[i]);
    // Serial.println(i);
  }

  speed_pid.Compute();

  right_spd -= max(0, min(fabs(min(0, output)) * reduction_rate, max_spd));
  left_spd -= max(0, min(fabs(max(0, output)) * reduction_rate, max_spd));

  // atualizar velocidade
  analogWrite(left_spd_pin, left_spd);
  analogWrite(right_spd_pin, right_spd);

  // Serial.println(output);
  Serial.println(input);
  Serial.println(output);
}
