void setup() {
  pinMode(14, OUTPUT);
  pinMode(15, OUTPUT);
  analogWrite(14, 0);
  analogWrite(15, 0);
  
}

void loop() {
  analogWrite(14, 0);
  delay(2000);
  analogWrite(14, 50);
  delay(2000);
  analogWrite(14, 100);
  delay(2000);
  analogWrite(14, 150);
  delay(2000);
  analogWrite(14, 200);
  delay(2000);
  analogWrite(14, 250);
  delay(2000);
  analogWrite(14, 0);

  analogWrite(15, 0);
  delay(2000);
  analogWrite(14, 50);
  delay(2000);
  analogWrite(15, 100);
  delay(2000);
  analogWrite(15, 150);
  delay(2000);
  analogWrite(15, 200);
  delay(2000);
  analogWrite(15, 250);
  delay(2000);
  analogWrite(15, 0);

}
