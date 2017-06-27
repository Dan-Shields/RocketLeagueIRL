void setup() {
  Serial.begin(9600);
}

void loop() {
  if(Serial.available() == 3) {
    unsigned char data[3];
    data[0] = Serial.read();
    data[1] = Serial.read();
    data[2] = Serial.read();
    Serial.write(data);
  }
}
