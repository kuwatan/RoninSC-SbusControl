// 送信側 (Maker NANO)
const int RE_DE = 3;     // MAX485 DE/RE ピン
const int xPin  = A0;    // ジョイスティック X
const int yPin  = A1;    // ジョイスティック Y

void setup() {
  pinMode(RE_DE, OUTPUT);
  digitalWrite(RE_DE, HIGH); // 送信モード
  Serial.begin(9600);
}

void loop() {
  int xVal = analogRead(xPin);
  int yVal = analogRead(yPin);

  // XとYをカンマ区切りで送信
  Serial.print(xVal);
  Serial.print(",");
  Serial.println(yVal);

  delay(10); // 10Hzくらいで送信
}
