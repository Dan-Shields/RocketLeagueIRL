const int In1 = 3;
const int In2 = 4;
const int In3 = 5;
const int In4 = 6;
const int enableRight = 10;
const int enableLeft = 9;

bool move,L,R,F_B,enable;
int speedByte,x,storedSpeedByte,i,n,controlByte,storedControlByte, inChar;
char inString

void forwardLeft(){
  analogWrite(enableLeft, speedByte);
  analogWrite(enableRight, 255);
  digitalWrite (In1, LOW);
  digitalWrite (In2, HIGH);
  digitalWrite (In3, HIGH);
  digitalWrite (In4, LOW);
}
void forwardRight(){
  analogWrite(enableLeft, 255);
  analogWrite(enableRight, speedByte);
  digitalWrite (In1, LOW);
  digitalWrite (In2, HIGH);
  digitalWrite (In3, HIGH);
  digitalWrite (In4, LOW);
}
void backwardLeft(){
  analogWrite(enableLeft, 255);
  analogWrite(enableRight, speedByte);
  digitalWrite (In1, HIGH);
  digitalWrite (In2, LOW);
  digitalWrite (In3, LOW);
  digitalWrite (In4, HIGH);
}
void backwardRight(){
  analogWrite(enableLeft, speedByte);
  analogWrite(enableRight, 255);
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
  pinMode(enableRight, OUTPUT);
  pinMode(enableLeft, OUTPUT);
  Serial.begin(9600);
}
void loop(){
  if(Serial.available()){
    controlByte = Serial.read();
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char
      // and add it to the string:
      inString += (char)inChar;
    }
    // if you get a newline, print the string,
    // then the string's value:
    if (inChar == '\n') {
      Serial.print("Value:");
      Serial.println(inString.toInt());
      Serial.print("String: ");
      Serial.println(inString);
      // clear the string for new input:
      inString = "";
    }
    Serial.print('control Byte = %d',controlByte);
    enable = bitRead(controlByte, 7);
    if(enable == 0){
      controlByte = storedControlByte;
      speedByte = storedSpeedByte;
    }
    move = bitRead(controlByte, 6);
    F_B = bitRead(controlByte, 5);
    L = bitRead(controlByte, 4);
    R = bitRead(controlByte, 3);
    storedControlByte = controlByte;
    storedSpeedByte = speedByte;
    if(move == 1){
    
      if(F_B == 1){
        if(L == 1){
          if(R == 1){
            stationary();
          }
          if(R == 0){
            forwardLeft();
          }
        }
        if(L == 0){
          if(R == 1){
            backwardLeft();
           
          }
          if(R == 0){
            
          }
        }
      }
      if(F_B == 0){
        if(L == 1){
          if(R == 1){
            backward();
            
          }
        }
        
        if(L == 0){
          if(R == 1){
            backwardRight();
            
          }
          if(R == 0){
            stationary();
            
          }
        }
        
      }
      
    }
  }
}
  

