
//無線機器を通じて相方の機体と通信する
//esp,xbeeどちらでも大丈夫(の予定)

int sendMyState(int state) {
  if (state >= 0 && state <= 0xFF) {
    mySerial.write(state);
    return 0;
  }
  else return -1;
}

int getPartnerState(void) {
  static char data = 0xFF;
  while (mySerial.available())data = mySerial.read(); //最後に送られた文字のみ保存する
  return (int)data;
}
