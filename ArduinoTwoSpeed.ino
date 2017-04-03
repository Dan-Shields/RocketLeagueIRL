const int In1 = 3;
const int In2 = 4;
const int In3 = 5;
const int In4 = 6;
const int EnableLeft = 9;
const int EnableRight = 10;
int incomingByte = 0;

void setup(){
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);
  pinMode(EnableLeft, OUTPUT);
  pinMode(EnableRight, OUTPUT);
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
      case ('5'):
        rightpivot();
        break;
      case ('6'):
        leftpivot();
        break;
    }
  }
}

void forward() {
  analogWrite  (EnableLeft, 255);
  analogWrite  (EnableRight, 255);
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
  analogWrite  (EnableLeft, 150);
  analogWrite  (EnableRight, 255);
  digitalWrite (In1, LOW);
  digitalWrite (In2, HIGH);
  digitalWrite (In3, HIGH);
  digitalWrite (In4, LOW);
}

void right() {
  analogWrite  (EnableRight, 150);
  analogWrite  (EnableLeft, 255);
  digitalWrite (In1, LOW);
  digitalWrite (In2, HIGH);
  digitalWrite (In3, HIGH);
  digitalWrite (In4, LOW);
}


void leftpivot() {
  analogWrite  (EnableRight, 255);
  analogWrite  (EnableLeft, 0);
  digitalWrite (In1, LOW);
  digitalWrite (In2, HIGH);
  digitalWrite (In3, HIGH);
  digitalWrite (In4, LOW);
}


void rightpivot() {
  analogWrite  (EnableRight, 0);
  analogWrite  (EnableLeft, 255);
  digitalWrite (In1, LOW);
  digitalWrite (In2, HIGH);
  digitalWrite (In3, HIGH);
  digitalWrite (In4, LOW);
}



