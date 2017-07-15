#include <Servo.h>

const int COMMAND_OUTPUT = 0x01;
const int COMMAND_SERVO = 0x02;
const int COMMAND_BUZZER = 0x08;
const int COMMAND_DISPLAY = 0x09;
const int COMMAND_MOTOR = 0x0a;
const int COMMAND_RGBLED = 0x0f;

const int M7219DIN = A0;
const int M7219LOAD = A1;
const int M7219CLK = A2;

const int Servo1 = 6;
const int Servo2 = 5;
const int Servo3 = 9;
const int Servo4 = 10;
const int ServoPins[] = {Servo1, Servo2, Servo3, Servo4};
int ServoDegree[4] = {30};
int ServoSpeed[4] = {0};
int ServoTarget[4] = {30};
int ServoMax = 160;
int ServoMin = 30;

const int MA1 = 11;
const int MA2 = 12;
const int MB2 = 13;
const int MB1 = 3;
const int MotorPins[] = {MA1, MA2, MB1, MB2};

const int BUZZER = A3;
const int pin1 = 8;
const int pin2 = 4;
const int pin3 = 7;
const int pin4 = 2;
const int pin5 = 4;
const int pin6 = 5;
const int DigitalOutputPins[] = {pin1, pin2, pin3, pin4, pin5, pin6};

const int pinR = 6;
const int pinG = 5;
const int pinB = 9;
const int RGBPins[] = {pinR, pinG, pinB};
int RGBColor[3] = {0};
int RGBSpeed = 0;
int RGBTarget[3] = {0};
bool isRGBon = false;

Servo Servos[4];

const int OutputPins[] = {MA1, MA2, MB2, MB1, BUZZER, pin1, pin2, 
pin3, pin4, pin5, pin6, pinR, pinG, pinB, M7219DIN, M7219LOAD, M7219CLK};


const int LBUTTON = A6;
const int RBUTTON = A7;

const int InputPins[] = {LBUTTON, RBUTTON};
int InputValue[2] = {0};


byte SerialReadBuffer[11] = {0};
byte SerialWriteBuffer[10] = {0};

byte checksum = 0;

int calcchecksum(byte *array){ //체크섬 계산
  checksum = 0;
  for (int i=0;i<10;i=i+1){
    checksum ^= SerialReadBuffer[i];
  }
  return checksum;
}

unsigned char Bit_Reverse( unsigned char x ) 
{ 
    x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa); 
    x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc); 
    x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0); 
    return x;    
}

void sendleddata(byte addr, byte data){
  digitalWrite(M7219LOAD, LOW);
  shiftOut(M7219DIN,M7219CLK,MSBFIRST,addr);
  shiftOut(M7219DIN,M7219CLK,MSBFIRST,data);
  digitalWrite(M7219LOAD, HIGH);
}

unsigned long LastUpdatecounter;
int UpdateDelay = 100;

void servoupdate(){
  for (int i=0;i<4;i++){
    if (Servos[i].attached() == true){ //연결되어 있는 서보면
      if (ServoTarget[i] > ServoDegree[i]){ //목표각이 현재각보다 큰가
        if (ServoSpeed[i] > (ServoTarget[i] - ServoDegree[i])){ //더가야하나
          ServoDegree[i] = ServoTarget[i]; //다옴
        }
        else{
          ServoDegree[i] = ServoDegree[i] + ServoSpeed[i]; //더가야됨
        }
      }
      else if (ServoTarget[i] < ServoDegree[i]){//목표각이 현재각보다 작으면
        if (ServoSpeed[i] > (ServoDegree[i] - ServoTarget[i])){ //더가야하나
          ServoDegree[i] = ServoTarget[i];  //다옴
        }
        else{
          ServoDegree[i] = ServoDegree[i] - ServoSpeed[i]; //더가야됨
        }
      }
      
    Servos[i].write(ServoDegree[i]);
    }
  }
  
}

void rgbledupdate(){
  for (int i=0;i<3;i++){
    if (RGBTarget[i] != 0){
      if (RGBTarget[i] > RGBColor[i]){ //목표량이 현재량보다 큰가
        if (RGBSpeed > RGBTarget[i] - RGBColor[i]){ //더가야하나
          RGBColor[i] = RGBTarget[i]; //다옴
        }
        else{
          RGBColor[i] = RGBColor[i] + RGBSpeed; //더가야됨
        }
      }
     
      else if (RGBTarget[i] < RGBColor[i]){ //목표량이 현재량보다 작은가
        if (RGBSpeed > RGBColor[i] - RGBTarget[i]){ //더가야하나
          RGBColor[i] = RGBTarget[i]; //다옴
        }
        else{
          RGBColor[i] = RGBColor[i] - RGBSpeed; //더가야됨
        }
      }
      analogWrite(RGBPins[i], 255 - RGBColor[i]);
    }
  }
}

void setup() {
  Serial.begin(38400);

  for (int i=0;i<17;i++){
    pinMode(i, OUTPUT);
  }

  for (int i=0;i<2;i++){
    pinMode(i, INPUT);
  }
  
  //led 배열 준비
  sendleddata(0x0c, 0x01); //배열의 대기모드 해제
  sendleddata(0x0b, 0x07);
  sendleddata(0x0a, 0x07); //배열의 밝기 설정
  for (byte i=0x01;i<0x09;i++){
    sendleddata(i, 0x00); //배열을 비운다
  }

}

void loop() {
  //받기
  if (Serial.available() >= 2) { //만약 정보가 있으면
    if (Serial.read() == 0xff) { //시작점 확인
      Serial.readBytes(SerialReadBuffer, 11); //정보를 읽음
      checksum = calcchecksum(SerialReadBuffer);//체크섬 계산
      if (SerialReadBuffer[10] == checksum){ //만약 체크섬이 맞다면
        switch(SerialReadBuffer[0]){
        
          case COMMAND_OUTPUT: //출력 명령       
            digitalWrite(DigitalOutputPins[SerialReadBuffer[1]-1], bool(SerialReadBuffer[2]));
            break;

          case COMMAND_SERVO: //서보 명령 (확인 필요)
            if (SerialWriteBuffer[2] != 0x00){ //서보 분리 명령이 아닌가
              if (isRGBon == true){ //만약 RGB LED가 사용 중이면
                for (int i=0;i<3;i++){
                  pinMode(RGBPins[i], LOW); //RGB LED를 끈다(켠다)
                }
                isRGBon = false; //RGBLED = 꺼짐
              }
              if (Servos[SerialWriteBuffer[1]-1].attached() == false){ //서보가 연결되어 있지 않은가
                Servos[SerialWriteBuffer[1]-1].attach(ServoPins[SerialWriteBuffer[1]-1]); //서보 연결
              }
              ServoTarget[SerialWriteBuffer[1]-1] = map(SerialWriteBuffer[2],1,100,ServoMin,ServoMax);
              ServoSpeed[SerialWriteBuffer[1]-1] = map(SerialWriteBuffer[3],0,100,0,128);            
            }
            else{ //서보 분리
              if (Servos[SerialWriteBuffer[1]-1].attached() == true){ //서보가 연결되어 있는가
                Servos[SerialWriteBuffer[1]-1].attach(ServoPins[SerialWriteBuffer[1]-1]); //서보 해제
              }
            }
            break;

          case COMMAND_BUZZER: //버저 명령
            if (SerialReadBuffer[2] == 0){ //만약 0을 받았다면
              noTone(BUZZER); //버저 중지
            }
            else{
              tone(BUZZER, 8000000/64/SerialReadBuffer[2]); //버저 제어
            }
            break;

          case COMMAND_DISPLAY: //LED 배열 명령
            for(int r=0;r<8;r=r+1) { //LED 배열 제어
              sendleddata(8-r, Bit_Reverse(SerialReadBuffer[2+r]));
            }
            break;
          
          case COMMAND_MOTOR: //DC 모터 명령
            if (SerialWriteBuffer[2] == 0){ //방향이 정지라면
              for (int i=0;i<2;i++){
                digitalWrite(MotorPins[i + 2 * (0x02 & SerialWriteBuffer[1])], 0);
              }
            }
            else{
              if (SerialWriteBuffer[2] == 1){ //방향이 시계방향이라면
                analogWrite(MotorPins[0 + 2 * (0x02 & SerialWriteBuffer[1])], map(SerialWriteBuffer[3],0, 100, 0, 255));
                digitalWrite(MotorPins[1 + 2 * (0x02 & SerialWriteBuffer[1])], 0);
              }
              else{ //방향이 반시계방향이라면
                digitalWrite(MotorPins[0 + 2 * (0x02 & SerialWriteBuffer[1])], 0);
                analogWrite(MotorPins[1 + 2 * (0x02 & SerialWriteBuffer[1])], map(SerialWriteBuffer[3],0, 100, 0, 255));
              }
            }
            break;

          case COMMAND_RGBLED: //RGB LED 명령
            if (isRGBon == false){ //만약 RGB LED가 사용 중이지 않으면
              for (int i=0;i<4;i++){
                Servos[i].detach(); //서보 분리
              }
              isRGBon = true; //RGBLED = 켜짐
            }
            RGBSpeed = SerialWriteBuffer[5];
            for (int i=0;i<3;i++){
              if (SerialWriteBuffer[i+2] == 0 ){
                RGBColor[i] = 0;
                digitalWrite(RGBPins[i], HIGH);
              }
              RGBTarget[i] = SerialWriteBuffer[i+2];
            }
            break;
        }
      }
    }
  }
  if (abs(millis() - LastUpdatecounter) > UpdateDelay){ //서보, RGB LED 업데이트 속도 제어
    servoupdate();
    rgbledupdate();
    LastUpdatecounter = millis();
  }
  //보내기 (읽기가 너무 빨라 값이 부정확함)
  SerialWriteBuffer[0] = 0xff;
  InputValue[0] = analogRead(LBUTTON);
  InputValue[1] = analogRead(RBUTTON);
  for (int i=0;i<2;i=i+1){
    SerialWriteBuffer[1+i*2] = Bit_Reverse(InputValue[i] << 8);
    SerialWriteBuffer[2+i*2] = Bit_Reverse(InputValue[i] & 0xff);
    SerialWriteBuffer[7+i] = SerialWriteBuffer[2+i*2] ^ 0x80;
  }
  SerialWriteBuffer[5] = 0x10;
  SerialWriteBuffer[6] = 0x22;
  for (int i=1;i<9;i=i+1){
    SerialWriteBuffer[9] ^= SerialWriteBuffer[i];
  }
  Serial.write(SerialWriteBuffer, 10);
}
