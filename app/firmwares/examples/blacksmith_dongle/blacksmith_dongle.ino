#include <Servo.h>            //헤더 호출
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include "U8glib.h"

// Module Constant //핀설정
#define ALIVE 0
#define DIGITAL 1
#define ANALOG 2
#define PWM 3
#define SERVO_PIN 4
#define TONE 5
#define PULSEIN 6
#define ULTRASONIC 7
#define TIMER 8
#define READ_BLUETOOTH 9
#define WRITE_BLUETOOTH 10
#define LCD 11
#define RGBLED 12
#define DCMOTOR 13
#define OLED 14

// State Constant
#define GET 1
#define SET 2
#define MODULE 3
#define RESET 4

Servo servos[8];
Servo sv;
LiquidCrystal_I2C lcd(0x27, 16, 2);
SoftwareSerial softSerial(2,3);
U8GLIB_SSD1306_128X64 oled(U8G_I2C_OPT_NONE);

// val Union        //??
union {
  byte byteVal[4];
  float floatVal;
  long longVal;
} val;

// valShort Union       //??
union {
  byte byteVal[2];
  short shortVal;
} valShort;

int analogs[6] = {0, 0, 0, 0, 0, 0};   // 아날로그 디지털 핀 값저장
int digitals[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int servo_pins[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// Ultrasonic             //초음파 센서
float lastUltrasonic = 0;
int trigPin = 13;
int echoPin = 12;

// bluetooth                //블루투스
String makeBtString;
int softSerialRX = 2;
int softSerialTX = 3;

// LCD
String lastLcdDataLine0;
String lastLcdDataLine1;

// Buffer
char buffer[52];
unsigned char prevc = 0;
byte index = 0;
byte dataLen;

double lastTime = 0.0;
double currentTime = 0.0;

uint8_t command_index = 0;

boolean isStart = false;
boolean isUltrasonic = false;
boolean isBluetooth = false;
// End Public Value

void mydelay_us(unsigned int time_us)
{
    register unsigned int i;
    /* 1us */
    for(i = 0; i < time_us; i++)          /* 4 cycle +        */
    {
      asm volatile(" PUSH  R0 ");       /* 2 cycle +        */
      asm volatile(" POP   R0 ");       /* 2 cycle +        */
      asm volatile(" PUSH  R0 ");       /* 2 cycle +        */
      asm volatile(" POP   R0 ");       /* 2 cycle +        */
      asm volatile(" PUSH  R0 ");       /* 2 cycle +        */
      asm volatile(" POP   R0 ");       /* 2 cycle    =  16 cycle*/
    }
}

void initPorts () {
  for (int pinNumber = 4; pinNumber < 14; pinNumber++) {
    pinMode(pinNumber, OUTPUT);
    digitalWrite(pinNumber, LOW);
  }
}

// void initLCD(){}

int searchServoPin(int pin) {
  for (int i = 0; i < 8; i++) {
    if (servo_pins[i] == pin) {
      return i;
    }
    if (servo_pins[i] == 0) {
      servo_pins[i] = pin;
      return i;
    }
  }
  return 0;
}

void setPortWritable(int pin) {
  if (digitals[pin] == 0) {
    digitals[pin] = 1;
    pinMode(pin, OUTPUT);
  }
}

unsigned char readBuffer(int index) {
  return buffer[index];
}
void writeBuffer(int index, unsigned char c) {
  buffer[index] = c;
}

void writeHead() {
  writeSerial(0xff);
  writeSerial(0x55);
}
void writeSerial(unsigned char c) {
  softSerial.write(c);
}
void writeEnd() {
  softSerial.println();
}

void sendString(String s) {
  int l = s.length();
  writeSerial(4);
  writeSerial(l);
  for (int i = 0; i < l; i++) {
    writeSerial(s.charAt(i));
  }
}

void sendFloat(float value) {
  writeSerial(2);
  val.floatVal = value;
  writeSerial(val.byteVal[0]);
  writeSerial(val.byteVal[1]);
  writeSerial(val.byteVal[2]);
  writeSerial(val.byteVal[3]);
}

void sendShort(double value) {
  writeSerial(3);
  valShort.shortVal = value;
  writeSerial(valShort.byteVal[0]);
  writeSerial(valShort.byteVal[1]);
}

short readShort(int idx) {
  valShort.byteVal[0] = readBuffer(idx);
  valShort.byteVal[1] = readBuffer(idx + 1);
  return valShort.shortVal;
}

float readFloat(int idx) {
  val.byteVal[0] = readBuffer(idx);
  val.byteVal[1] = readBuffer(idx + 1);
  val.byteVal[2] = readBuffer(idx + 2);
  val.byteVal[3] = readBuffer(idx + 3);
  return val.floatVal;
}

long readLong(int idx) {
  val.byteVal[0] = readBuffer(idx);
  val.byteVal[1] = readBuffer(idx + 1);
  val.byteVal[2] = readBuffer(idx + 2);
  val.byteVal[3] = readBuffer(idx + 3);
  return val.longVal;
}

void callOK() {          //상태 확인용
  writeSerial(0xff);     //테일
  writeSerial(0x55);    //테일2
  writeEnd();           //다음줄로 넘기기
}

void sendAnalogValue(int pinNumber) {
  writeHead();
  sendFloat(analogRead(pinNumber));
  writeSerial(pinNumber);
  writeSerial(ANALOG);
  writeEnd();
}

void sendDigitalValue(int pinNumber) {
  pinMode(pinNumber, INPUT);
  writeHead();
  sendFloat(digitalRead(pinNumber));
  writeSerial(pinNumber);
  writeSerial(DIGITAL);
  writeEnd();
}

void sendPinValues() {
  int pinNumber = 0;
  for (pinNumber = 4; pinNumber < 14; pinNumber++) {
  // Send Only Switch Inputs
    if (digitals[pinNumber]==0){
      sendDigitalValue(pinNumber);
      callOK();
    }
  }
  for (pinNumber = 0; pinNumber < 6; pinNumber++) {
  //Send Analog Inputs
    sendAnalogValue(pinNumber);
    callOK();
  }
}


void setup(){
  softSerial.begin(57600);
  initPorts();
  // initLCD();
}


void loop() {
  while (softSerial.available()) {
      unsigned char c = softSerial.read();
      setPinValue(c);
  }
  delay(15);
  sendPinValues();
  delay(10);
}

void parseData() {
  isStart = false;
  int idx = readBuffer(3);
  command_index = (uint8_t)idx;
  int action = readBuffer(4);
  int device = readBuffer(5);
  int port = readBuffer(6);

  switch (action) {
    case GET: {
        // if (device == ULTRASONIC) {
        // }
        // else {
        //   digitals[port] = 0;
        // }
      }
      break;
    case SET: {
        runSet(device);
        callOK();
      }
      break;
    case MODULE: {
        runModule(device);
        callOK();
      }
    case RESET: {
        callOK();
      }
      break;
  }
}

void setPinValue (char c) {
  if (c == 0x55 && isStart == false) {
    if (prevc == 0xff) {
      index = 1;
      isStart = true;
    }
  } else {
    prevc = c;
    if (isStart) {
      if (index == 2) {
        dataLen = c;
      } else if (index > 2) {
        dataLen--;
      }
      writeBuffer(index, c);
    }
  }

  index++;

  if (index > 51) {
    index = 0;
    isStart = false;
  }

  if (isStart && dataLen == 0 && index > 3) {
    isStart = false;
    parseData();
    index = 0;
  }
}

void runSet(int device) {
  //0xff 0x55 0x6 0x0 0x1 0xa 0x9 0x0 0x0 0xa

  int port = readBuffer(6);
  unsigned char pin = port;
  switch (device) {
    case DIGITAL: {
        setPortWritable(pin);
        int v = readBuffer(7);
        digitalWrite(pin, v);
      }
      break;
    case PWM: {
        setPortWritable(pin);
        int v = readBuffer(7);
        analogWrite(pin, v);
      }
      break;
    case TONE: {
        setPortWritable(pin);
        int hz = readShort(7);
        int ms = readShort(9);
        if (ms > 0) {
          tone(pin, hz, ms);
        } else {
          noTone(pin);
        }
      }
      break;
    case SERVO_PIN: {
        setPortWritable(pin);
        int v = readBuffer(7);
        if (v >= 0 && v <= 180) {
          byte rg[]={TCCR1A,TCCR1B,OCR1A,TIMSK1};
          delay(5);
          sv = servos[searchServoPin(pin)];
          sv.attach(pin);
          sv.write(v);
          delay(100);
          sv.detach();
          TCCR1A=rg[0];
          TCCR1B=rg[1];
          TIMSK1=rg[3];
          OCR1A=rg[2];
        }
      }
      break;
    case TIMER: {
        lastTime = millis() / 1000.0;
      }
      break;
    case RGBLED: {
        // 지정된 색깔을 제대로 표현하기 위해 강제로 3회 반복 함
        //if (pin == 3 || pin == 8 || pin == 9) rgbLedVer2(pin);
        //else
        // rgbLedVer1(pin);
        delay(10);
        //if (pin == 3 || pin == 8 || pin == 9) rgbLedVer2(pin);
        //else
        // rgbLedVer1(pin);
        delay(10);
        //if (pin == 3 || pin == 8 || pin == 9) rgbLedVer2(pin);
       // else
       // rgbLedVer1(pin);
        delay(10);
      }
      break;
    case DCMOTOR: {
        int directionPort = readBuffer(7);
        int speedPort = readBuffer(9);
        int directionValue = readBuffer(11);
        int speedValue = readBuffer(13);
        setPortWritable(directionPort);
        setPortWritable(speedPort);
        digitalWrite(directionPort, directionValue);
        analogWrite(speedPort, speedValue);
      }
      break;
    default:
      break;
  }
}

void runModule(int device) {
  //0xff 0x55 0x6 0x0 0x1 0xa 0x9 0x0 0x0 0xa
  //head head                        pinNUM
  //                                      A/D

  int port = readBuffer(6);
  unsigned char pin = port;
  switch (device) {
    case LCD: {
        // String makeLcdString;
        // int arrayNum = 7;
        // for (int i = 0; i < 17; i++) {
        //   char lcdRead = readBuffer(arrayNum);
        //   if (lcdRead > 0) makeLcdString += lcdRead;
        //   arrayNum += 2;
        // }
        // if (makeLcdString.equals(lastLcdDataLine0) == false || makeLcdString.equals(lastLcdDataLine1) == false)
        // {
        //   lcd.setCursor(0, pin);
        //   lcd.print("                ");
        // }
        // lcd.setCursor(0, pin);
        // if (readBuffer(7) == 1) {
        //   int lcdInt = readShort(9);
        //   lcd.print(lcdInt);
        // }
        // else {
        //   lcd.print(makeLcdString);
        // }
        // if (pin == 0) lastLcdDataLine0 = makeLcdString;
        // else if (pin == 1) lastLcdDataLine1 = makeLcdString;
      }
      break;
    case OLED: {
        // int x = readBuffer(7);
        // int y = readBuffer(9);
        // String makeOledString;
        // int arrayNum = 11;
        // for (int i = 0; i < 17; i++) {
        //   char oledRead = readBuffer(arrayNum);
        //   if (oledRead > 0) makeOledString += oledRead;
        //   arrayNum += 2;
        // }

        // if (readBuffer(11) == 1) {
        //   int oledInt = readShort(13);
        //   oled.firstPage();
        //   do {
        //     oled.setFont(u8g_font_unifont);
        //     oled.setPrintPos(x, y);
        //     oled.print(oledInt);
        //   } while (oled.nextPage());
        // }
        // else {
        //   oled.firstPage();
        //   do {
        //     oled.setFont(u8g_font_unifont);
        //     oled.setPrintPos(x, y);
        //     oled.print(makeOledString);
        //   } while (oled.nextPage());
        // }
      }
      break;
    default:
      break;
  }
}
