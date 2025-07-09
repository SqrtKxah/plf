#include <ArduPID.h>
#include <math.h>

#define left_spd_pin 3
#define right_spd_pin 2
#define left_mtr_forward 53
#define left_mtr_back 52
#define right_mtr_forward 51
#define right_mtr_back 50
const double pi = 3.14159;
double input = 0;
double output = 0;
double setpoint = 0;
double kp = 3, ki = 0.03, kd = 1;
const double sensor_threshold = 900;

const double max_spd = 200;

int sensor_pins[4] = { 22, 23, 25, 19};                       // pinos
float sensor_values[4] = { -pi / 2, -pi / 5, pi / 5, pi/2 };  // valores de peso agregados aos sensores

ArduPID speed_pid;

int invert(int num) {
  return pow(num, 0) - num;
}

void setup() {
  Serial.begin(1200);
  pinMode(20, OUTPUT);
  digitalWrite(20,HIGH);
  pinMode(left_mtr_forward, OUTPUT);
  pinMode(left_mtr_back, OUTPUT);
  pinMode(right_mtr_forward, OUTPUT);
  pinMode(right_mtr_back, OUTPUT);
  pinMode(left_spd_pin, OUTPUT);
  pinMode(right_spd_pin, OUTPUT);
  for (int i = 0; i <= 3; i++) {
    pinMode(sensor_pins[i], INPUT);
  }

  digitalWrite(left_mtr_forward, LOW);
  digitalWrite(right_mtr_forward, LOW);
  digitalWrite(left_mtr_back, HIGH);
  digitalWrite(right_mtr_back, HIGH);
  speed_pid.begin(&input, &output, &setpoint, kp, ki, kd);
  speed_pid.setOutputLimits(-pi / 1.5, pi / 1.5);
  speed_pid.setSampleTime(5);
  speed_pid.start();
}

void loop() {
  double left_spd = max_spd;
  double right_spd = max_spd;
  input = 0;

  // ler sensor, calcular soma de erro e calcular velocidade
  for (int i = 0; i <= 3; i++) {
    int s_input = digitalRead(sensor_pins[i]);
    // input += (s_input > sensor_threshold) * sensor_values[i];
    // input += sensor_values[i] * invert(s_input);
    input += sensor_values[i] * s_input;
    // Serial.println(String("Sensor " + String(i, DEC) + " Valor " + String((sensor_values[i] * s_input), DEC)));
  }

  speed_pid.compute();

  const double alpha = cos(output);

  // direção de motor dependendo do alpha

  left_spd -= min(-min(output, 0), 1) * alpha * max_spd;
  right_spd -= min(max(output, 0), 1) * alpha * max_spd;

  // if (fabs(output) > (pi / 2)) {
  //   if (max(output, 0)) {
  //     digitalWrite(left_mtr_forward, HIGH);
  //     digitalWrite(right_mtr_forward, LOW);
  //     digitalWrite(left_mtr_back, LOW);
  //     digitalWrite(right_mtr_back, HIGH);
  //   } else {
  //     digitalWrite(left_mtr_forward, LOW);
  //     digitalWrite(right_mtr_forward, HIGH);
  //     digitalWrite(left_mtr_back, HIGH);
  //     digitalWrite(right_mtr_back, LOW);
  //   }
  // } else {
  //     digitalWrite(left_mtr_forward, HIGH);
  //     digitalWrite(right_mtr_forward, HIGH);
  //     digitalWrite(left_mtr_back, LOW);
  //     digitalWrite(right_mtr_back, LOW);
  // }
  if (fabs(output) > (pi / 2)) {
    if (max(output, 0) == 0) {
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
  Serial.println(String("Entrada" + String(input, DEC)));
  Serial.println(String("Saida" + String(output, DEC)));
}
