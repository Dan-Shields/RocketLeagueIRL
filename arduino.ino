const int In1 = 3;
const int In2 = 4;
const int In3 = 5;
const int In4 = 6;
const int enableRight = 10;
const int enableLeft = 9;
bool readFin = false;
int turnSpeed = 255,globalSpeed = 0,x,i,n,controlByte, inChar,move,L,R,F_B,enable;
double speedRatio = 1;
String inString = "";

void forwardLeft() {
  analogWrite(enableLeft, (int) (turnSpeed * speedRatio));
  analogWrite(enableRight, globalSpeed);
  digitalWrite (In1, LOW);
  digitalWrite (In2, HIGH);
  digitalWrite (In3, HIGH);
  digitalWrite (In4, LOW);
}

void forwardRight() {
  analogWrite(enableLeft, globalSpeed);
  analogWrite(enableRight, (int) (turnSpeed * speedRatio));
  digitalWrite (In1, LOW);
  digitalWrite (In2, HIGH);
  digitalWrite (In3, HIGH);
  digitalWrite (In4, LOW);
}
void backwardLeft(){
  analogWrite(enableLeft, globalSpeed);
  analogWrite(enableRight, (int) (turnSpeed * speedRatio));
  digitalWrite (In1, HIGH);
  digitalWrite (In2, LOW);
  digitalWrite (In3, LOW);
  digitalWrite (In4, HIGH);
}
void backwardRight(){
  analogWrite(enableLeft, (int) (turnSpeed * speedRatio));
  analogWrite(enableRight, globalSpeed);
  digitalWrite (In1, HIGH);
  digitalWrite (In2, LOW);
  digitalWrite (In3, LOW);
  digitalWrite (In4, HIGH);
}
void forward(){
  analogWrite(enableLeft, globalSpeed);
  analogWrite(enableRight, globalSpeed);
  digitalWrite (In1, LOW);
  digitalWrite (In2, HIGH);
  digitalWrite (In3, HIGH);
  digitalWrite (In4, LOW);
}
void backward(){
  analogWrite(enableLeft, globalSpeed);
  analogWrite(enableRight, globalSpeed);
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
    while (readFin != true) {
      inChar = Serial.read();
      if (inChar != '\n') {
          inString += (char)inChar;
      } 
      else {
        controlByte = inString.toInt();
        inString = "";
        inChar = "";
        readFin = true;
      }
    }
    readFin = false;
    while (readFin != true) {
      inChar = Serial.read();
      if (inChar != '\n') {
        inString += (char)inChar;
      } 
      else {
        turnSpeed = 255 - inString.toInt();
        inString = "";
        inChar = "";
        readFin = true;
      }
    }
	readFin = false;
	while (readFin != true) {
		inChar = Serial.read();
		if (inChar != '\n') {
			inString += (char)inChar;
		} 
		else {
			globalSpeed = inString.toInt();
			speedRatio = globalSpeed / 255;
			inString = "";
			inChar = "";
			readFin = true;
		}
    }
    readFin = false;
    
    enable = bitRead(controlByte, 7);
    move = bitRead(controlByte, 6);
    F_B = bitRead(controlByte, 5);
    L = bitRead(controlByte, 4);
    R = bitRead(controlByte, 3);
    if(move == 1) {
    
      if(F_B == 1) {
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
            forwardRight();
           
          }
          if(R == 0){
            forward();
          }
        }
      }
      if(F_B == 0){
        if(L == 1 && R == 1){
            backward();
        }
        else if(L == 0){
          if(R == 1){
            backwardRight();
          }
          if(R == 0){
            backward();
          }
        }
        else{
           backwardLeft();
        }
        
      }
      
    }
    else{
      stationary();
    }
  }
}
  
