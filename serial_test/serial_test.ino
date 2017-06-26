void setup() {
  Serial.begin(9600);
}

void loop() {
  if(Serial.available() > 0) {
    byte data = Serial.read();
    Serial.write(data);
  }
}
