#include <ServoTimer2.h>

ServoTimer2 servo;

// フォトリフレクタのアナログピン
#define LINE_R A0
#define LINE_M A1
#define LINE_L A2

// PSDセンサのアナログピン
#define PSD_F A3

// KP: Pゲイン, KD: Dゲイン
// **_NUM: 分子, **_DEN: 分母 を指定する
#define LT_KP_NUM 1
#define LT_KP_DEN 20
#define LT_KD_NUM 1
#define LT_KD_DEN 50

// KP: Pゲイン, KI: Iゲイン, KD: Dゲイン
// **_NUM: 分子, **_DEN: 分母 を指定する
#define F_KP_NUM 2
#define F_KP_DEN 1
#define F_KI_NUM 2
#define F_KI_DEN 10
#define F_KD_NUM 1
#define F_KD_DEN 100

// モータ駆動用の PWM ピン
#define MOTOR_L_IN1 5
#define MOTOR_L_IN2 6
#define MOTOR_R_IN1 9
#define MOTOR_R_IN2 10

//ボタン用
#define BUTTON_PIN 12

int buttonState = 0;

int x = 0; // 今の状態
int xPrev = 0; // 前の状態
int xDiff = 0; // 状態の微分値

double ratio_RLspeed = 0.8;
int ikiti = 400; //フォトリフレクタの白黒の閾値
int count_Cross = 0;
int count_Dassen = 0;

int data[4] = {0, 0, 0, 0};

int RSpeed;
int LSpeed;
int State_Cross = 0;
int delaytime;

int valR;
int valM;
int valL;

int valF;

// モータの PWM を設定する関数
void setMotorPulse(int left, int right) {//ここで左右比いじっておきたい
  if (left > 0) {
    analogWrite(MOTOR_L_IN1, min(left, 255)); analogWrite(MOTOR_L_IN2, 0);
  } else {
    analogWrite(MOTOR_L_IN1, 0); analogWrite(MOTOR_L_IN2, min(-left, 255));
  }
  if (right > 0) {
    analogWrite(MOTOR_R_IN1, min(right, 255)); analogWrite(MOTOR_R_IN2, 0);
  } else {
    analogWrite(MOTOR_R_IN1, 0); analogWrite(MOTOR_R_IN2, min(-right, 255));
  }
}

void lineTrace() {
  //PD制御
  // 左右のセンサ値の差を状態として扱う.正のとき右寄り,負のとき左寄り.
  x = valR - valL;
  // 状態の時間微分.50 倍 は 0.02(秒) 分の 1 の意味.
  xDiff = (x - xPrev) * 50;
  xPrev = x;

  // 前進指令値
  int v = 200;
  // 回転指令値 (左回り正) PD 制御を実装している
  int w = x * LT_KP_NUM / LT_KP_DEN + xDiff * LT_KD_NUM / LT_KD_DEN;

  // モータの PWM パルスを設定
  setMotorPulse((v - w)*ratio_RLspeed, v + w);
}

int refF = 650; // 目標値.ここを各自計測した値に設定する.
int e = 0; // 今の偏差
int ePrev = 0; // 前の偏差
int eInt = 0; // 偏差の積分値
int eDiff = 0; // 偏差の微分値

int count_wait_box = 0;
int count_PSD_under;

void frontDistanceControl() {
  int valF = analogRead(PSD_F);
  //Serial.println(valF);
  // 目標値との偏差を計算する
  e = refF - valF;
  eInt += e / 50;
  eDiff = (e - ePrev) * 50; ePrev = e;
  // 前進指令値 (PID)
  int v = e * F_KP_NUM / F_KP_DEN + eInt * F_KI_NUM / F_KI_DEN + eDiff * F_KD_NUM / F_KD_DEN; // 回転指令値 (左回り正)
  int w = 0;
  // モータの PWM パルスを設定
  setMotorPulse((v - w)*ratio_RLspeed, v + w);
}

unsigned long tPrev; // 前の時刻
unsigned int tProc;

int val_Servo;

int state;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(1000000);
  servo.attach(2);
  servo.write(544);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  tPrev = millis();
  state = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  valR = analogRead(LINE_R);
  valM = analogRead(LINE_M);
  valL = analogRead(LINE_L);
  valF = analogRead(PSD_F);

  //Serial.println(valM);

  data[3] = data[2];
  data[2] = data[1];
  data[1] = data[0];
  data[0] = valR - valL;

  if (valR > ikiti && valM > ikiti && valL > ikiti) { //脱線の検出
    //Serial.println("Dassen");
    count_Dassen += 1;
  }
  if (valR < ikiti && valM < ikiti && valL < ikiti) { //交差点の検出
    count_Cross += 1;
  }
  if (count_Cross > 0) { //交差点時の処理
    state = 1;
  }
  switch (state) {
    case 0 :  //待機モード
      buttonState = digitalRead(BUTTON_PIN);
      if (buttonState == HIGH) {

      } else {
        state = 2;
      }
      break;
    case 1 :  //交差点時の処理
      setMotorPulse(0, 0);
      delay(1000);

      //setMotorPulse(255*ratio_RLspeed,255);
      //delay(800);
      switch (State_Cross) {
        case 0 : RSpeed = 255;
          LSpeed = 0;
          delaytime =  700;
          break;
        /*case 1 : RSpeed = -255;
          LSpeed = 255;
          delaytime = 2000;
          break;*/
        case 2 : RSpeed = 255;
          LSpeed = 0;
          delaytime = 700;
          break;
        case 3 : RSpeed = 255;
          LSpeed = 255;
          delaytime = 700;
          break;
        /*case 4 : RSpeed = 255;
          LSpeed = -255;
          delaytime = 2000;
          break;
          case 5 : RSpeed = 0;
          LSpeed = 255;
          delaytime = 700;
          break;*/
        case 4 : RSpeed = 0;
          LSpeed = 0;
          delaytime = 500;
          break;
        case 5 : RSpeed = -255;
          LSpeed = 255;
          delaytime = 2000;
          break;
        case 6 : RSpeed = 255;
          LSpeed = 255;
          delaytime = 700;
          break;
        case 7 : RSpeed = 0;
          LSpeed = 255;
          delaytime =  700;
          break;
      }
      setMotorPulse(LSpeed * ratio_RLspeed, RSpeed);
      delay(delaytime);
      valM = analogRead(LINE_M);
      while (valM > ikiti) {
        if (State_Cross == 6) {
          break;
        }
        valM = analogRead(LINE_M);
        setMotorPulse(LSpeed * ratio_RLspeed, RSpeed);
      }
      count_Cross = 0;
      State_Cross += 1;
      if (State_Cross == 8) {
        State_Cross = 1;
      }
      state = 2;
      break;
    case 2 :  //ライントレース
      if (count_Dassen > 3) { //脱線時の処理
        if (data[3] < 0) {
          while (valM > ikiti) {
            valM = analogRead(LINE_M);
            setMotorPulse(255, 0);
          }
        }
        else if (data[3] > 0) {
          while (valM > ikiti) {
            valM = analogRead(LINE_M);
            setMotorPulse(0, 255);
          }
        }
        count_Dassen = 0;
      }
      lineTrace();
      tProc = millis() - tPrev;
      if (tProc < 20) {
        // 処理時間と合わせて 20ms になるように delay を入れる
        delay(20 - tProc);
      }
      tPrev = millis();
      if (State_Cross == 1) {
        count_wait_box += 1;
        if (count_wait_box > 100) {
          state = 3;
          count_wait_box = 0;
        }
      }
      if (State_Cross == 5) {
        state = 7;
      }
      break;
    case 3 :  //箱下げる
      servo.write(1600);
      val_Servo = 1600;
      delay(500);
      state = 4;
    case 4 :
      valF = analogRead(PSD_F);
      if(valF <= 100 ){
        valF = analogRead(PSD_F);
        setMotorPulse(-200 * ratio_RLspeed, -200);
      }
      else if(valF > 100){
        frontDistanceControl();
        tProc = millis() - ePrev;
        if (tProc < 20) {
          // 処理時間と合わせて 20ms になるように delay を入れる
          delay(20 - tProc);
        }
        tPrev = millis();
        if (e <= 20 && e >= -20) {
          count_wait_box += 1;
        }
        if (e > 20 || e < -20) {
          count_wait_box = 0;
        }
        if (count_wait_box > 2) {
          state = 5;
        }
      }
      break;
    case 5 :
      setMotorPulse(0, 0);
      val_Servo -= 20;
      servo.write(val_Servo);
      delay(100);
      if (val_Servo < 544) {
        state = 6;
      }
      break;
    case 6 ://ここstate1でいけるやろ
      RSpeed = -255;
      LSpeed = 255;
      delaytime = 2000;
      setMotorPulse(LSpeed * ratio_RLspeed, RSpeed);
      delay(delaytime);
      valM = analogRead(LINE_M);
      while (valM > ikiti) {
        valM = analogRead(LINE_M);
        setMotorPulse(LSpeed * ratio_RLspeed, RSpeed);
      }
      State_Cross += 1;//後で削ってCrossのcaseの方を変えて
      state = 2;
      break;
    case 7 :
      setMotorPulse(0, 0);
      val_Servo += 20;
      servo.write(val_Servo);
      delay(100);
      if (val_Servo > 1600) {
        state = 8;
      }
      break;
    case 8 :
      setMotorPulse(-255 * ratio_RLspeed, -255);
      delay(1500);
      state = 1;
      break;
  }
  Serial.print(state);
  Serial.print(" ");
  Serial.println(valF);
}