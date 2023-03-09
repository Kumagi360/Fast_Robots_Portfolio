const int RightPin1 = 0;
const int RightPin2 = 15;
const int LeftPin1 = 3;
const int LeftPin2 = 16;

void setup() {
  pinMode(RightPin1, OUTPUT);
  pinMode(RightPin2, OUTPUT);
  pinMode(LeftPin1, OUTPUT);
  pinMode(LeftPin2, OUTPUT);

  analogWrite(RightPin1, 0);
  analogWrite(RightPin2, 0);
  analogWrite(LeftPin1, 0);
  analogWrite(LeftPin2, 0);
}

void driveRightForward(){
  int startTime = millis();

  while (millis() - startTime < 2000){
    analogWrite(RightPin1, 150);
  }

  analogWrite(RightPin1, 0);
}

void driveRightBackward(){
  int startTime = millis();

  while (millis() - startTime < 2000){
    analogWrite(RightPin2, 150);
  }

  analogWrite(RightPin2, 0);
}

void driveLeftForward(){
  int startTime = millis();

  while (millis() - startTime < 2000){
    analogWrite(LeftPin1, 150);
  }

  analogWrite(LeftPin1, 0);
}

void driveLeftBackward(){
  int startTime = millis();

  while (millis() - startTime < 2000){
    analogWrite(LeftPin2, 150);
  }

  analogWrite(LeftPin2, 0);
}
void loop() {
  driveRightForward();
  delay(1000);
  driveRightBackward();
  delay(1000);
  driveLeftForward();
  delay(1000);
  driveLeftBackward();
  delay(1000);
}
