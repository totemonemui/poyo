void lineTrace() {
  isDassen();//脱線の検出
  isCross();//交差点の検出
  Dassen();

  //PD制御
  // 左右のセンサ値の差を状態として扱う.正のとき右寄り,負のとき左寄り.
  x = valRPhotoRef - valLPhotoRef;
  // 状態の時間微分.50 倍 は 0.02(秒) 分の 1 の意味.
  xDiff = (x - xPrev) * 50;
  eInt += x / 50;
  xPrev = x;

  // 前進指令値
  int v = 200;
  // 回転指令値 (左回り正) PD 制御を実装している
  int w = x * LT_KP_NUM / LT_KP_DEN + xInt * LT_KI_NUM / LT_KI_DEN + xDiff * LT_KD_NUM / LT_KD_DEN;

  // モータの PWM パルスを設定
  setMotorPulse(v - w, v + w);

  tProc = millis() - tPrev;
  if (tProc < 20) {
    // 処理時間と合わせて 20ms になるように delay を入れる
    delay(20 - tProc);
  }
  tPrev = millis();
}
