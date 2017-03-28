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
    int byte = stoi(incomingByte);
    if(byte <= 64){
      analogWrite  (EnableLeft, 255);
      analogWrite  (EnableRight, int(byte*3.98));
      digitalWrite (In1, LOW);
      digitalWrite (In2, HIGH);
      digitalWrite (In3, HIGH);
      digitalWrite (In4, LOW);
  }
    else if(byte >= 64){
      analogWrite  (EnableLeft, int(-4.05*byte + 514));
      analogWrite  (EnableRight, 255);
      digitalWrite (In1, LOW);
      digitalWrite (In2, HIGH);
      digitalWrite (In3, HIGH);
      digitalWrite (In4, LOW);
  }
    else{
      analogWrite  (EnableLeft, 150);
      analogWrite  (EnableRight, 255);
      digitalWrite (In1, HIGH);
     digitalWrite (In2, LOW);
      digitalWrite (In3, LOW);
      digitalWrite (In4, HIGH);
      
    }
}

/*void forward() {
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
  digitalWrite (In3, LOW);
  digitalWrite (In4, HIGH);
}

void right() {
  analogWrite  (EnableRight, 150);
  analogWrite  (EnableLeft, 255);
  digitalWrite (In1, HIGH);
  digitalWrite (In2, LOW);
  digitalWrite (In3, HIGH);
  digitalWrite (In4, LOW);
}*/


