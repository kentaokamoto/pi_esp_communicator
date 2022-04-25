#define MOTOR0_PIN 14

void setup() {

  pinMode(MOTOR0_PIN, OUTPUT);
  
  delay(3000);
  analogWriteResolution(12);
  analogWriteFrequency(MOTOR0_PIN, 25);
  delay(100);
  //analogWrite(MOTOR0_PIN, 4095);
  //delay(100);
  //analogWrite(MOTOR0_PIN, 0);
  ///////////////////////////////////////////////////
}

void loop() {
  delay(100);
  analogWrite(MOTOR0_PIN, 1000);
}
