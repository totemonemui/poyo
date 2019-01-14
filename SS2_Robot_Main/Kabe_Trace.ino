//壁に沿って進むやつ

void kabeTrace(){
  getRPSD();
  getLPSD();
  
  // 左右のセンサ値の差を状態として扱う.正のとき右寄り,負のとき左寄り. 
  kb_x = valRPSD - valLPSD;
  
  // 状態の時間微分.50 倍 は 0.02(秒) 分の 1 の意味.
  kb_xDiff = (kb_x - kb_xPrev) * 50;
  kb_xInf += kb_x/50;
  kb_xPrev = kb_x;
  // 前進指令値
  int v = 200;
  // 回転指令値 (左回り正) PD 制御を実装している
  int w = kb_x * KT_KP_NUM / KT_KP_DEN + kb_xInf * KT_KI_NUM / KT_KI_DEN + kb_xDiff * KT_KD_NUM / KT_KD_DEN;
  // モータの PWM パルスを設定 
  setMotorPulse(v - w, v + w);
  // 処理時間を計算
  unsigned int tProc = millis() - tPrev; 
  if (tProc < 20) {
    // 処理時間と合わせて 20ms になるように delay を入れる
    delay(20 - tProc);
  }
  tPrev = millis();
}

