#include <Adafruit_TCS34725.h>
#include <math.h>

#define left_spd_pin 3
#define right_spd_pin 2
#define left_mtr_forward 53
#define left_mtr_back 52
#define right_mtr_forward 51
#define right_mtr_back 50
double input = 0;
double output = 0;
double last = 0;
int first_run = 1;
const double sensor_threshold = 900;

const double max_spd = 255;
const double max_turn_spd = 155;

int sensor_pins[4] = { 22, 23, 25, 19};                  // pinos
float sensor_values[4] = { -2, -1, 1, 2 };  // valores de peso agregados aos sensores

bool is_within_range(double desired, double value, double minimum, double maximum) {
  double dist = value - desired;
  return (dist >= minimum) && (dist <= maximum);
}

void setup() {
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

}

void loop() {
  double left_spd = max_spd;
  double right_spd = max_spd;
  input = 0;

  // ler sensor, calcular soma de erro e calcular velocidade
  for (int i = 0; i <= 3; i++) {
    int s_input = digitalRead(sensor_pins[i]);
    input += sensor_values[i] * s_input;
  }

  if ((last == input) && (!first_run)) {
    return;
  }

  if ((input == 0) && (last != 1)) {
    input = last;
  }

  const double alpha = min(max(input, -1), 1);

  if (fabs(input) != 0) {
    if (max(input, 0) == 0) {
      digitalWrite(left_mtr_forward, HIGH);
      digitalWrite(right_mtr_forward, LOW);
      digitalWrite(left_mtr_back, LOW);
      digitalWrite(right_mtr_back, HIGH);
      // Serial.println("HARD RIGHT");
    } else {
      digitalWrite(left_mtr_forward, LOW);
      digitalWrite(right_mtr_forward, HIGH);
      digitalWrite(left_mtr_back, HIGH);
      digitalWrite(right_mtr_back, LOW);
      // Serial.println("HARD LEFT");
    }
  } else {
      digitalWrite(left_mtr_forward, LOW);
      digitalWrite(right_mtr_forward, LOW);
      digitalWrite(left_mtr_back, HIGH);
      digitalWrite(right_mtr_back, HIGH);
      // Serial.println("HARD FORWARD");
  }

  // direção de motor dependendo do alpha

  left_spd -= 1 - (-min(alpha, 0)) * max_turn_spd;
  right_spd -= 1 - max(alpha, 0) * max_turn_spd;

  // atualizar velocidade
  analogWrite(left_spd_pin, left_spd);
  analogWrite(right_spd_pin, right_spd);

  first_run = 0;
  last = input;
}
