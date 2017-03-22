const int In1 = 3;
const int In2 = 4;
const int In3 = 5;
const int In4 = 6;
int incomingByte = 0;

void setup(){
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);
  Serial.begin(9600);
}

void loop(){
  if (Serial.available())  {
    incomingByte = Serial.read();
    switch(incomingByte) {
      case ('1'):
        forward();
        break;
      case ('2'):
        backward();
        break;
      case ('3'):
        left();
        break;
      case ('4'):
        right();
        break;
    }
  }
  delay(5);
}

void forward() {
  digitalWrite (In1, LOW);
  digitalWrite (In2, HIGH);
  digitalWrite (In3, HIGH);
  digitalWrite (In4, LOW);
}

void backward() {
  digitalWrite (In1, HIGH);
  digitalWrite (In2, LOW);
  digitalWrite (In3, LOW);
  digitalWrite (In4, HIGH);
}

void left() {
  digitalWrite (In1, LOW);
  digitalWrite (In2, HIGH);
  digitalWrite (In3, LOW);
  digitalWrite (In4, HIGH);
}

void right() {
  digitalWrite (In1, HIGH);
  digitalWrite (In2, LOW);
  digitalWrite (In3, HIGH);
  digitalWrite (In4, LOW);
}
