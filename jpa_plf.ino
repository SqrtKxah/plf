#include <PID_v1.h>
#include <math.h>

#define left_spd_pin 3
#define right_spd_pin 2
#define left_mtr_forward 33
#define left_mtr_back 35
#define right_mtr_forward 37
#define right_mtr_back 39
const double pi = 3.14159;
double input = 0;
double output = 0;
double setpoint = 0;
double kp = 6, ki = 0.2, kd = 1;

int sensor_pins[5] = { 23, 25, 27, 29, 31 };                       // pinos
float sensor_values[5] = { -pi / 2, -pi / 5, 0, pi / 5, pi / 2 };  // valores de peso agregados aos sensores

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
  const double max_spd = 255;
  double left_spd = max_spd;
  double right_spd = max_spd;
  input = 0;

  // ler sensor, calcular soma de erro e calcular velocidade
  for (int i = 0; i <= 4; i++) {
    int s_input = invert(digitalRead(sensor_pins[i]));
    input += s_input * sensor_values[i];
  }

  speed_pid.Compute();

  const double alpha = cos(output);

  // direção de motor dependendo do alpha

  right_spd -= min(-min(output, 0), 1) * alpha * max_spd;
  left_spd -= min(max(output, 0), 1) * alpha * max_spd;

  if (fabs(output) > (pi / 2)) {
    if (max(output, 0)) {
      digitalWrite(left_mtr_forward, LOW);
      digitalWrite(right_mtr_forward, HIGH);
      digitalWrite(left_mtr_back, HIGH);
      digitalWrite(right_mtr_back, LOW);
    } else {
      digitalWrite(left_mtr_forward, HIGH);
      digitalWrite(right_mtr_forward, LOW);
      digitalWrite(left_mtr_back, LOW);
      digitalWrite(right_mtr_back, HIGH);
    }
  } else {
      digitalWrite(left_mtr_forward, LOW);
      digitalWrite(right_mtr_forward, LOW);
      digitalWrite(left_mtr_back, HIGH);
      digitalWrite(right_mtr_back, HIGH);
  }

  // atualizar velocidade
  analogWrite(left_spd_pin, left_spd);
  analogWrite(right_spd_pin, right_spd);

  // Serial.println(output);
  Serial.println(input);
  Serial.println(output);
}
