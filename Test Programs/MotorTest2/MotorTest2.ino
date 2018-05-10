//const int Motor1_DIR = 7;
//const int Motor2_DIR = 8;

//const int Motor1_forward = 10;
//const int Motor1_backward = 9;
//const int Motor2_forward = 6;
//const int Motor2_backward = 5;

const int Motor1_forward = 7;
const int Motor1_backward = 6;
const int Motor2_forward = 5;
const int Motor2_backward = 4;

const int Motor1_PWM = 11;
const int Motor2_PWM = 3;


int speed = 0;

void setup() {
  pinMode(Motor1_forward, OUTPUT);
  pinMode(Motor1_backward, OUTPUT);
  pinMode(Motor2_forward, OUTPUT);
  pinMode(Motor2_backward, OUTPUT);
  pinMode(Motor1_PWM, OUTPUT);
  pinMode(Motor2_PWM, OUTPUT);

  Serial.begin(9600);
}

void move_forward() {
  digitalWrite(Motor1_backward, LOW);
  digitalWrite(Motor1_forward, HIGH);
  digitalWrite(Motor2_backward, LOW);
  digitalWrite(Motor2_forward, HIGH);

  analogWrite(Motor1_PWM, abs(speed));
  analogWrite(Motor2_PWM, abs(speed));
}

void move_backward() {
  digitalWrite(Motor1_backward, HIGH);
  digitalWrite(Motor1_forward, LOW);
  digitalWrite(Motor2_backward, HIGH);
  digitalWrite(Motor2_forward, LOW);

  analogWrite(Motor1_PWM, abs(speed));
  analogWrite(Motor2_PWM, abs(speed));
}

void loop() {
  if (Serial.available() > 0) {
    speed = Serial.parseInt();
    Serial.print("Speed set to: ");
    Serial.println(speed);
  }
  if (speed > 0) {
    move_forward();
  }
  if (speed < 0) {
    move_backward();
  }
  else{
    analogWrite(Motor1_PWM, abs(speed));
    analogWrite(Motor2_PWM, abs(speed));
    digitalWrite(Motor1_backward, LOW);
    digitalWrite(Motor1_forward, LOW);
    digitalWrite(Motor1_backward, LOW);
    digitalWrite(Motor1_forward, LOW);
    //analogWrite(Motor1_backward, 0);
    //analogWrite(Motor1_forward, 0);
    //analogWrite(Motor2_backward, 0);
    //analogWrite(Motor2_forward, 0);
  }
  //analogWrite(Motor1_PWM, speed);

}
