const int Motor1_DIR = 7;
const int Motor2_DIR = 8;
const int Motor1_PWM = 9;
const int Motor2_PWM = 10;
float speed = 0;
float threshold = 55;
void set_speed(float vel) {
  speed = abs(((255-threshold)/100)*abs(vel)+threshold);
  if (vel > 0) {
    digitalWrite(Motor1_DIR, HIGH);
    digitalWrite(Motor2_DIR, HIGH);
    analogWrite(Motor1_PWM, speed);
    analogWrite(Motor2_PWM, speed);
  }
  else if (vel < 0) {
    digitalWrite(Motor1_DIR, LOW);
    digitalWrite(Motor2_DIR, LOW);
    analogWrite(Motor1_PWM, speed);
    analogWrite(Motor2_PWM, speed);
  }
  else {
    digitalWrite(Motor1_DIR, HIGH);
    digitalWrite(Motor2_DIR, HIGH);
    analogWrite(Motor1_PWM, 0);
    analogWrite(Motor2_PWM, 0);
  }
}

void setup() {
  pinMode(Motor1_PWM, OUTPUT);
  pinMode(Motor2_PWM, OUTPUT);
  set_speed(0);

  Serial.begin(115200);
}

void loop() {
    if (Serial.available() > 0) {
      speed = Serial.parseFloat();
      Serial.print("Speed set to: ");
      Serial.println(speed);
      set_speed(speed);
    }
}

