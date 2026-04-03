
const int buzzer = A0;

void startChime(int buzzerPin) {

  tone(buzzerPin, 3150);
  delay(175);
  noTone(buzzerPin);

  delay(75);

  tone(buzzerPin, 2350);
  delay(175);
  noTone(buzzerPin);

  delay(25);

  tone(buzzerPin, 2650);
  delay(175);
  noTone(buzzerPin);

  delay(25);

  tone(buzzerPin, 1975);
  delay(175);
  noTone(buzzerPin);

  delay(25);

  tone(buzzerPin, 1325);
  delay(175);
  noTone(buzzerPin);
}

void setup() {
  startChime(buzzer);  // Change the pin number as needed
}

void loop() {
  // no need to repeat the melody.
}
