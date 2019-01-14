void frontDistanceControl() {
  valFPSD = analogRead(PSD_F);
  //Serial.println(valF);
  // 目標値との偏差を計算する
  e = refF - valFPSD;
  eInt += e / 50;
  eDiff = (e - ePrev) * 50; ePrev = e;
  // 前進指令値 (PID)
  int v = e * F_KP_NUM / F_KP_DEN + eInt * F_KI_NUM / F_KI_DEN + eDiff * F_KD_NUM / F_KD_DEN; // 回転指令値 (左回り正)
  int w = 0;
  // モータの PWM パルスを設定
  setMotorPulse(v - w, v + w);
  
  tProc = millis() - tPrev;
  if (tProc < 20) {
    // 処理時間と合わせて 20ms になるように delay を入れる
    delay(20 - tProc);
  }
  tPrev = millis();
}
