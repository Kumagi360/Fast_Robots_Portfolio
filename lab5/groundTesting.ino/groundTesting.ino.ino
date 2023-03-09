const int LeftPin1 = 0;
const int LeftPin2 = 15;
const int RightPin1 = 3;
const int RightPin2 = 16;

const int calibFactor = 1.7;

void setup() {
  pinMode(RightPin1, OUTPUT);
  pinMode(RightPin2, OUTPUT);
  pinMode(LeftPin1, OUTPUT);
  pinMode(LeftPin2, OUTPUT);

  analogWrite(RightPin1, 0);
  analogWrite(RightPin2, 0);
  analogWrite(LeftPin1, 0);
  analogWrite(LeftPin2, 0);

  delay(5000);
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

void driveForward(){

  analogWrite(LeftPin1, 100);
  analogWrite(RightPin1, 100);

  delay(1000);

  analogWrite(LeftPin1, 0);
  analogWrite(RightPin1, 0);
}

void driveBackward(){

  analogWrite(LeftPin2, 100);
  analogWrite(RightPin2, 100);

  delay(1000);

  analogWrite(LeftPin2, 0);
  analogWrite(RightPin2, 0);
}

// 50s were both failing to turn
// 30s on both was failing to move forw and back

void driveLeft(){

  analogWrite(LeftPin1, 100);

  delay(1000);

  analogWrite(LeftPin1, 0);
}

void driveRight(){

  analogWrite(RightPin1, 100);

  delay(1000);

  analogWrite(RightPin1, 0);
}

void driveForwardandBackwardCalib(){

  analogWrite(LeftPin1, 100);
  analogWrite(RightPin1, min(255, floor(100 * calibFactor)));

  delay(1500);

  analogWrite(LeftPin1, 0);
  analogWrite(RightPin1, floor(0 * calibFactor));


  analogWrite(LeftPin2, 100);
  analogWrite(RightPin2, min(255, floor(100 * calibFactor)));

  delay(1500);

  analogWrite(LeftPin2, 0);
  analogWrite(RightPin2, floor(0 * calibFactor));

}

void spotSpin(){

    analogWrite(LeftPin1, 100);
    analogWrite(RightPin2, 100);

    delay(1000);

    analogWrite(LeftPin1, 0);
    analogWrite(RightPin2, 0);
}

void forwardSpinBackward(){

  driveForward();
  spotSpin();
  driveBackward();

}


void loop() {
  // driveForward();
  // delay(1000);
  // driveBackward();
  // delay(1000);

  //driveLeft();

  //driveRight();

  //driveForwardCalib();

  forwardSpinBackward();
  
  while(true){}
}
