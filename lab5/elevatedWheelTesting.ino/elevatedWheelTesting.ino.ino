const int Drive1Pin1 = 14;
const int Drive1Pin2 = 15;

void setup() {
  pinMode(Drive1Pin1, OUTPUT);
  pinMode(Drive1Pin2, OUTPUT);

  analogWrite(Drive1Pin1, 0);
  analogWrite(Drive1Pin2, 0);
}

void driveForward(){
  int startTime = millis();

  while (millis() - startTime < 5000){
    analogWrite(Drive1Pin1, 200);
  }

  analogWrite(Drive1Pin1, 0);
}

void driveBackward(){
  int startTime = millis();

  while ((millis() - startTime) < 5000){
    analogWrite(Drive1Pin2, 200);
  }

  analogWrite(Drive1Pin2, 0);
}

void loop() {
  driveForward();
  delay(2000);

}
