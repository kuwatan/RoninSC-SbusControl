#include "BMC_SBUS.h"
#include <SoftwareSerial.h>

BMC_SBUS mySBUS;

// RS485用 SoftwareSerial (RX,TX)
SoftwareSerial RS485Serial(4, 8);

const int RE_DE = 2;   // MAX485 DE/RE ピン (受信固定ならLOW)
int xVal = 511;
int yVal = 512;

// 実測レンジ（あなたの実測に合わせて調整）
const int RAW_MIN = 263;
const int RAW_MAX = 760;

const int sbusInterval = 7; // ms (SBUSフレーム間隔)
unsigned long lastSbusMillis = 0;
unsigned long lastReceiveTime = 0;

int panChannel = 1;
int tiltChannel = 2;

// 受信用バッファ
char inBuf[32];
uint8_t inIndex = 0;

// 平滑化
int smoothX = 511;
int smoothY = 512;

// デバッグ用カウンタ
unsigned long sbusSendCount = 0;
unsigned long badFrameCount = 0;

int applyDeadZone(int value, int center, int deadZoneSize) {
  if (abs(value - center) < deadZoneSize) return center;
  return value;
}

void setup() {
  pinMode(RE_DE, OUTPUT);
  digitalWrite(RE_DE, LOW); // 受信固定
  Serial.begin(115200);     // デバッグ用シリアル
  RS485Serial.begin(9600);
  mySBUS.begin();

  // 小さな安全表示
  Serial.println("Start stable SBUS bridge");
}

void parseAndApply(const char *s) {
  // 期待形式: "123,456"
  const char *comma = strchr(s, ',');
  if (!comma) {
    badFrameCount++;
    return;
  }
  // 両側に数字のみか簡易チェック
  // 左
  int lenX = comma - s;
  int lenY = strlen(comma + 1);
  if (lenX <= 0 || lenY <= 0 || lenX > 6 || lenY > 6) {
    badFrameCount++;
    return;
  }

  // atoi で変換（atoi は '非数' -> 0 なので範囲チェックする）
  int tmpX = atoi(s);
  int tmpY = atoi(comma + 1);

  // 範囲チェック（受信側で期待するADCレンジに合わせる）
  if (tmpX < 0 || tmpX > 4095 || tmpY < 0 || tmpY > 4095) { // 広めに取る
    badFrameCount++;
    return;
  }

  // 正常値のみ適用
  xVal = tmpX;
  yVal = tmpY;
  lastReceiveTime = millis();
}

void processSerialRx() {
  // 非ブロッキングで1バイトずつ受けて改行でパースする
  while (RS485Serial.available()) {
    char c = RS485Serial.read();
    if (c == '\r') continue; // 無視
    if (c == '\n') {
      inBuf[inIndex] = '\0';
      if (inIndex > 0) {
        parseAndApply(inBuf);
      }
      inIndex = 0;
    } else {
      if (inIndex < (sizeof(inBuf) - 1)) {
        inBuf[inIndex++] = c;
      } else {
        // overflow -> リセット
        inIndex = 0;
        badFrameCount++;
      }
    }
  }
}

void sendSbusIfNeeded() {
  unsigned long now = millis();
  if (now - lastSbusMillis >= (unsigned long)sbusInterval) {
    lastSbusMillis = now;

    // タイムアウト: 受信が一定期間ない場合は中央へ戻す
    if (millis() - lastReceiveTime > 500) {
      // 中立値（あなたの実測中心に合わせる）
      xVal = 511;
      yVal = 512;
    }

    // デッドゾーン（中心付近のノイズを抑制）
    int Xv = applyDeadZone(xVal, 511, 20);
    int Yv = applyDeadZone(yVal, 512, 20);

    // 平滑化（強さは調整可）
    smoothX = (smoothX * 3 + Xv) / 4;
    smoothY = (smoothY * 3 + Yv) / 4;

    // オフセット補正あればここで。今回は実測レンジに合わせる
    int clampedX = constrain(smoothX, RAW_MIN, RAW_MAX);
    int clampedY = constrain(smoothY, RAW_MIN, RAW_MAX);

    // 対称マップ（RAW_MIN..RAW_MAX -> 0..2047）
    int sendValueX = map(clampedX, RAW_MIN, RAW_MAX, 0, 2047);
    int sendValueY = map(clampedY, RAW_MIN, RAW_MAX, 0, 2047);

    // 最低/最高の安全制限
    sendValueX = constrain(sendValueX, 0, 2047);
    sendValueY = constrain(sendValueY, 0, 2047);

    // SBUS送信（ここで詰まっていないか確認）
    mySBUS.Servo(panChannel, sendValueX);
    mySBUS.Servo(tiltChannel, sendValueY);
    mySBUS.Update();
    mySBUS.Send();
    sbusSendCount++;
  }
}

void loop() {
  processSerialRx();
  sendSbusIfNeeded();

  // デバッグ: 定期的に状態を出す（必要なければコメントアウト）
  static unsigned long lastDbg = 0;
  if (millis() - lastDbg > 1000) {
    lastDbg = millis();
    Serial.print("rx:");
    Serial.print(xVal);
    Serial.print(",");
    Serial.print(yVal);
    Serial.print(" sm:");
    Serial.print(smoothX);
    Serial.print(",");
    Serial.print(smoothY);
    Serial.print(" sbusCnt:");
    Serial.print(sbusSendCount);
    Serial.print(" bad:");
    Serial.println(badFrameCount);
  }
}
