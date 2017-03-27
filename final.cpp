const int In1 = 3;
const int In2 = 4;
const int In3 = 5;
const int In4 = 6;
const int enableRight = 10;
const int enableLeft = 9;
byte controlByte,storedControlByte;
int speedByte,x,enable,storedSpeedByte,move,L,R,F_B;

void forwardLeft(){
  analogWrite(enableLeft, speedByte);
  analogWrite(255-enableRight, 255);
  digitalWrite (In1, LOW);
  digitalWrite (In2, HIGH);
  digitalWrite (In3, HIGH);
  digitalWrite (In4, LOW);
}
void forwardRight(){
  analogWrite(255-enableLeft, 255);
  analogWrite(enableRight, speedByte);
  digitalWrite (In1, LOW);
  digitalWrite (In2, HIGH);
  digitalWrite (In3, HIGH);
  digitalWrite (In4, LOW);
}
void backwardLeft(){
  analogWrite(255-enableLeft, 255);
  analogWrite(enableRight, speedByte);
  digitalWrite (In1, HIGH);
  digitalWrite (In2, LOW);
  digitalWrite (In3, LOW);
  digitalWrite (In4, HIGH);
}
void backwardRight(){
  analogWrite(enableLeft, speedByte);
  analogWrite(255-enableRight, 255);
  digitalWrite (In1, HIGH);
  digitalWrite (In2, LOW);
  digitalWrite (In3, LOW);
  digitalWrite (In4, HIGH);
}
void forward(){
  analogWrite(enableLeft, 255);
  analogWrite(enableRight, 255);
  digitalWrite (In1, LOW);
  digitalWrite (In2, HIGH);
  digitalWrite (In3, HIGH);
  digitalWrite (In4, LOW);
}
void backward(){
  analogWrite(enableLeft, 255);
  analogWrite(enableRight, 255);
  digitalWrite (In1, HIGH);
  digitalWrite (In2, LOW);
  digitalWrite (In3, LOW);
  digitalWrite (In4, HIGH);
}
void stationary(){
  digitalWrite (In1, LOW);
  digitalWrite (In2, LOW);
  digitalWrite (In3, LOW);
  digitalWrite (In4, LOW);
}
void setup(){
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);
  Serial.begin(9600);
}
void loop(){
  if (Serial.available())  {
    controlByte = Serial.read();
    speedByte = Serial.read();
    enable = bitRead(controlByte, 7);
    if(enable == 0){
      controlByte = storedControlByte;
      speedByte = storedSpeedByte;
    }
    move = bitRead(controlByte, 6);
    F_B = bitRead(controlByte, 5);
    L = bitRead(controlByte, 4);
    R = bitRead(controlByte,3);
    storedControlByte = controlByte;
    storedSpeedByte = speedByte;
    if(move == 1){
      if(F_B == 1 && L == 1 && R == 0){    //Forward and Left
        x = 1;
      }
      if(F_B == 0 && L == 1 && R == 0){    //Backward and Left
        x = 2;
      }
      if(F_B == 1 && L == 0 && R == 1){    //Forward and Right
        x = 3;
      }
      if(F_B == 0 && L == 0 && R == 1){    //Backward and Right
        x = 4;
      }
      if(F_B == 1 && L == 1 && R == 1){    //Forward
        x = 5;
      }
      if(F_B == 0 && L == 1 && R == 1){    //Backward
        x = 6;
      }
    else{
      x = 7;                            //Stationary
    }
    switch(x) {
      case ('1'):
        forwardLeft();
        break;
      case ('2'):
        forwardRight();
        break;
      case ('3'):
        backwardLeft();
        break;
      case ('4'):
        backwardRight();
        break;
      case ('5'):
        stationary();
        break;
    }
  }
}
}
