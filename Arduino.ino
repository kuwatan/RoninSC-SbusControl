#include "BMC_SBUS.h"
#include <SoftwareSerial.h>

BMC_SBUS mySBUS;

// RS485用 SoftwareSerial (例: D7=RX, D8=TX)
SoftwareSerial RS485Serial(4, 8);

const int RE_DE = 2;   // MAX485 DE/RE ピン
int xVal = 0;
int yVal = 0;
const int xOffset = 511;  // ← 例えば実測で中立が500なら
const int yOffset = 512;

const int sbusWAIT = 7; // SBUSフレーム間隔 msec

// SBUSのチャンネル
int panChannel = 1;
int tiltChannel = 2;

int applyDeadZone(int value, int center, int deadZoneSize) {
  if (abs(value - center) < deadZoneSize) {
    return center;
  }
  return value;
}

void setup() {
  pinMode(RE_DE, OUTPUT);
  digitalWrite(RE_DE, LOW); // 常に受信モード

  RS485Serial.begin(9600);  // RS485用シリアル
  mySBUS.begin();           // SBUS送信 (TXピンから出力)
}

void loop() {
  // --- RS485受信処理 ---
  if (RS485Serial.available()) {
    String data = RS485Serial.readStringUntil('\n'); // 改行まで受信
    int commaIndex = data.indexOf(',');
    if (commaIndex > 0) {
      xVal = data.substring(0, commaIndex).toInt();
      yVal = data.substring(commaIndex + 1).toInt();
    }
  }

  // --- SBUSに反映 ---
  int Xvalue = xVal;
  int Yvalue = yVal;

  Xvalue = applyDeadZone(Xvalue, 500, 80);  // 500±20の範囲を中立に
  Yvalue = applyDeadZone(Yvalue, 500, 20);
  Xvalue = constrain(Xvalue, 270, 750);
  Yvalue = constrain(Yvalue, 265, 750);
  Xvalue = Xvalue - (xOffset - 500);
  Yvalue = Yvalue - (yOffset - 500);

  int sendValueX = map(Xvalue,256,750,0,2047);
  int sendValueY = map(Yvalue,270,750,0,2047);

  mySBUS.Servo(panChannel,  sendValueX);
  mySBUS.Servo(tiltChannel, sendValueY);

  mySBUS.Update();
  mySBUS.Send();

  delay(sbusWAIT);
}
