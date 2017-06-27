const int In1 = 3;
const int In2 = 4;
const int In3 = 5;
const int In4 = 6;
const int enableRight = 10;
const int enableLeft = 9;
bool readFin = false;
int turnSpeed = 255,globalSpeed = 0,x,i,n,move,L,R,F_B,enable;
byte controlByte;
double speedRatio = 1;

#define MAX_MILLIS_TO_WAIT 1000
unsigned long starttime;

void forwardLeft()
{
  analogWrite(enableLeft, (int) (turnSpeed * speedRatio));
  analogWrite(enableRight, globalSpeed);
  digitalWrite (In1, LOW);
  digitalWrite (In2, HIGH);
  digitalWrite (In3, HIGH);
  digitalWrite (In4, LOW);
}

void forwardRight()
{
  analogWrite(enableLeft, globalSpeed);
  analogWrite(enableRight, (int) (turnSpeed * speedRatio));
  digitalWrite (In1, LOW);
  digitalWrite (In2, HIGH);
  digitalWrite (In3, HIGH);
  digitalWrite (In4, LOW);
}
void backwardLeft()
{
  analogWrite(enableLeft, globalSpeed);
  analogWrite(enableRight, (int) (turnSpeed * speedRatio));
  digitalWrite (In1, HIGH);
  digitalWrite (In2, LOW);
  digitalWrite (In3, LOW);
  digitalWrite (In4, HIGH);
}
void backwardRight()
{
  analogWrite(enableLeft, (int) (turnSpeed * speedRatio));
  analogWrite(enableRight, globalSpeed);
  digitalWrite (In1, HIGH);
  digitalWrite (In2, LOW);
  digitalWrite (In3, LOW);
  digitalWrite (In4, HIGH);
}
void forward()
{
  analogWrite(enableLeft, globalSpeed);
  analogWrite(enableRight, globalSpeed);
  digitalWrite (In1, LOW);
  digitalWrite (In2, HIGH);
  digitalWrite (In3, HIGH);
  digitalWrite (In4, LOW);
}
void backward()
{
  analogWrite(enableLeft, globalSpeed);
  analogWrite(enableRight, globalSpeed);
  digitalWrite (In1, HIGH);
  digitalWrite (In2, LOW);
  digitalWrite (In3, LOW);
  digitalWrite (In4, HIGH);
}
void stationary()
{
  digitalWrite (In1, LOW);
  digitalWrite (In2, LOW);
  digitalWrite (In3, LOW);
  digitalWrite (In4, LOW);
}
void setup()
{
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);
  pinMode(enableRight, OUTPUT);
  pinMode(enableLeft, OUTPUT);
  Serial.begin(9600);
}
void loop(){
  starttime = millis();
  
  while ( (Serial.available()<3) && ((millis() - starttime) < MAX_MILLIS_TO_WAIT) )
  {
    // hang in this loop until we either get 3 bytes of data or 1 second
    // has gone by
  }
  if(Serial.available() < 3)
  {
    // the data didn't come in - handle that problem here
    //Serial.println("ERROR - Didn't get 9 bytes of data!");
    while(Serial.available()){Serial.read();}
  }
  else
  {
    controlByte = (byte)Serial.read();
    turnSpeed = (int)Serial.read();
    globalSpeed = (int)Serial.read();
    
    Serial.write(controlByte, 1);
    Serial.write((byte)turnSpeed, 1);
    Serial.write((byte)globalSpeed, 1);
    
    
    enable = bitRead(controlByte, 7);
    move = bitRead(controlByte, 6);
    F_B = bitRead(controlByte, 5);
    L = bitRead(controlByte, 4);
    R = bitRead(controlByte, 3);
    
    if(move == 1) {
      if(F_B == 1) {
        if(L == 1) {
          if(R == 1) {
            stationary();
          }
          if(R == 0) {
            forwardLeft();
          }
        }
        if(L == 0) {
          if(R == 1) {
            forwardRight();
           
          }
          if(R == 0) {
            forward();
          }
        }
      }
      if(F_B == 0) {
        if(L == 1 && R == 1) {
            backward();
        }
        else if(L == 0) {
          if(R == 1) {
            backwardRight();
          }
          if(R == 0) {
            backward();
          }
        }
        else {
           backwardLeft();
        }
      }
    }
    else {
      stationary();
    }
  }
}