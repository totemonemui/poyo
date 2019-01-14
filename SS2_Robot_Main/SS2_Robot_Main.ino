#include <ServoTimer2.h>
#include <SoftwareSerial.h>

ServoTimer2 servo;
SoftwareSerial mySerial(2, 3);//RX, TX 無線モジュールとのシリアル通信

// フォトリフレクタのアナログピン
#define LINE_R A0
#define LINE_M A1
#define LINE_L A2
// 前側PSDセンサ(F)のアナログピン
#define PSD_F A3
//側面PSDセンサ(R:右,L:左)のアナログピン
#define PSD_R A5
#define PSD_L A4
// モータ駆動用の PWM ピン
#define MOTOR_L_IN1 5
#define MOTOR_L_IN2 6
#define MOTOR_R_IN1 9
#define MOTOR_R_IN2 10
//ボタン用ピン
#define BUTTON_PIN 12

//ライントレースのパラメータ
// KP: Pゲイン, KD: Dゲイン
// **_NUM: 分子, **_DEN: 分母 を指定する
#define LT_KP_NUM 1
#define LT_KP_DEN 20
#define LT_KI_NUM 2
#define LT_KI_DEN 10
#define LT_KD_NUM 1
#define LT_KD_DEN 50

//前方の壁から一定の距離に止まるやつのパラメータ
// KP: Pゲイン, KI: Iゲイン, KD: Dゲイン
// **_NUM: 分子, **_DEN: 分母 を指定する
#define F_KP_NUM 2
#define F_KP_DEN 1
#define F_KI_NUM 0
#define F_KI_DEN 0
#define F_KD_NUM 1
#define F_KD_DEN 100

//壁沿いに進むためのパラメータ
// KP: Pゲイン, KD: Dゲイン
// **_NUM: 分子, **_DEN: 分母 を指定する
#define KT_KP_NUM 1
#define KT_KP_DEN 2
#define KT_KD_NUM 0
#define KT_KD_DEN 10000
#define KT_KI_NUM 1
#define KT_KI_DEN 100

//定数たち
int IKITI_PHOTO_REF = 400;//フォトリフレクタの白黒の閾値
double RATIO_RLSPEED = 0.8;//左右のモーターの比
int SERVO_UNDER = 1240;//箱が下がった時のサーボの値
int SERVO_UP = 540;//箱が上がった時のサーボの値
int IKITI_PSD = 100;//前のPSDのやつ、これより下の値だとやばい、これより上なら箱がちゃんと降りてる

//変数たち
int state; //大まかなステート
int sub_State; //ステート内での遷移
int count_Cross = 0; //何回フォトリフレクタが何回続けて交差点の状態になったか
//変数たち
int valRPhotoRef;//右のフォトリフレクターの値
int valMPhotoRef;//中央のフォトリフレクターの値
int valLPhotoRef;//左のフォトリフレクターの値
int valFPSD;//箱についてるPSDの値
int valRPSD;//右のPSDの値
int valLPSD;//左のPSDの値
int RSpeed;//右のモータのスピード、交差点で曲がる時に使用
int LSpeed;//左のモータのスピード、交差点で曲がる時に使用
int delaytime;//回り続ける時間、交差点で曲がる時に使用
int count_Dassen = 0;//脱線の状態に何回連続でなったか
int data_photo_ref[4] = {0, 0, 0, 0};//脱線時にどちらに戻るか参照するためのデータ
int buttonState = 0;//ボタンの値
int count_time = 0;//しばらく経ってから次へ移行するときとかに使う
int val_Servo;//サーボの値
int countPSD = 0; //回転の際にPSDの値が何回150を下回ったか確認する用

//ライントレース用
int x = 0; // 今の状態
int xPrev = 0; // 前の状態
int xInt = 0; //状態の積分値
int xDiff = 0; // 状態の微分値
unsigned long tPrev; // 前の時刻
unsigned int tProc;

//前側の壁に近づく用
int refF = 600; // 目標値.ここを各自計測した値に設定する.
int e = 0; // 今の偏差
int ePrev = 0; // 前の偏差
int eInt = 0; // 偏差の積分値
int eDiff = 0; // 偏差の微分値

//壁に沿って進む用  //バグの元 本来kt_の方が良かった
int kb_x = 0; // 今の状態
int kb_xPrev = 0; // 前の状態
int kb_xDiff = 0; // 状態の微分値
int kb_xInf = 0; // 状態の積分値

void setup() {
  // put your setup code here, to run once:
  Serial.begin(1000000);
  mySerial.begin(9600);
  servo.attach(2);
  servo.write(544);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  tPrev = millis();
  state = 0;
  sub_State = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  getRPhotoRef();
  getMPhotoRef();
  getLPhotoRef();
  getFPSD();
  getDassenData();
  switch (state) {
    case 0 : //待機状態
      buttonState = digitalRead(BUTTON_PIN);
      if (buttonState == HIGH) {

      } else {
        state = 1;
      }
      delay(20);
      break;
    case 1 : //一つ目の交差点までライントレース
      lineTrace();
      if (count_Cross > 0) {
        setMotorPulse(0, 0);
        count_Cross = 0;
        sub_State = 0;
        state = 2;
      }
      break;
    case 2 : //球のある通り(線あり)に入り球を回収
      switch (sub_State) {
        case 0 ://交差点を曲がる
          RSpeed = 255;
          LSpeed = 0;
          delaytime =  700;
          Cross();
          sub_State = 9;
          Serial.println("0 to 1");
          break;
        case 9 ://曲がった後のライントレース
          lineTrace();
          count_time += 1;
          if (count_time > 1000) {
            sub_State = 2;
            count_time = 0;
          }
          Serial.println("1 to 2");
          break;
        case 2 : //箱を下げる
          val_Servo += 20;
          servo.write(val_Servo);
          delay(200);
          if (val_Servo > 1240) {
            sub_State = 3;
          }
          break;
        case 3 : //前側の壁へのフィードバック制御
          getFPSD();
          if (valFPSD <= 100 ) { //壁に当たって下がりきらなかった時の処理
            getFPSD();
            setMotorPulse(-200, -200);
            delay(200);
          }
          else if (valFPSD > 100) { //壁側へフィードバック
            frontDistanceControl();
            if (e <= 20 && e >= -20) {
              count_time += 1;
            }
            if (e > 20 || e < -20) {
              count_time = 0;
            }
            if (count_time > 2) {//ちょうど良い位置になったら次へ
              sub_State = 4;
            }
          }
          break;
        case 4 : //箱を上げる処理
          setMotorPulse(0, 0);
          val_Servo -= 20;
          servo.write(val_Servo);
          delay(100);
          if (val_Servo < 540) {
            sub_State = 5;
          }
          break;
        case 5 : //ちょっとだけバック
          setMotorPulse(-200, -200);
          delay(700);
          sub_State = 6;
          break;
        case 6 : //回転
          RSpeed = -255;
          LSpeed = 255;
          delaytime = 2000;
          Cross();
          sub_State = 8;
          break;
        case 8 :
          lineTrace();
          if (count_Cross > 0) { //線を無視してるんだけどもうちょいうまくやりたい
            lineTrace();
            count_Cross = 0;
            count_time += 1;
            if(count_time>4){
              sub_State = 0;
              state = 3;
            }
          }
          break;
      }
    case 3 : //球のある通り(線なし)に入り球を回収
      switch (sub_State) {
        case 0 ://まずはライントレース
          lineTrace();
          if (count_Cross > 0) { //交差点を検知
            setMotorPulse(0, 0);
            count_Cross = 0;
            sub_State = 1;
          }
          break;
        case 1 : //線をまたいで線がないゾーンへ
          setMotorPulse(255, 255);
          delay(1000);
          sub_State = 2;
          break;
        case 2 : //箱を下げる
          servo.write(1240);
          val_Servo = 1240;
          delay(500);
          sub_State = 3;
          break;
        case 3 : //前側の壁へのフィードバック制御
          getFPSD();
          if (valFPSD <= 100 ) { //壁に当たって下がりきらなかった時の処理
            getFPSD();
            setMotorPulse(-200, -200);
          }
          else if (valFPSD > 100) { //壁側へフィードバック
            frontDistanceControl();
            if (e <= 20 && e >= -20) {
              count_time += 1;
            }
            if (e > 20 || e < -20) {
              count_time = 0;
            }
            if (count_time > 2) {//ちょうど良い位置になったら次へ
              sub_State = 4;
            }
          }
          break;
        case 4 : //箱を上げる処理
          setMotorPulse(0, 0);
          val_Servo -= 20;
          servo.write(val_Servo);
          delay(100);
          if (val_Servo < 540) {
            sub_State = 5;
          }
          break;
        case 5 : //ちょっとだけバック
          setMotorPulse(-200, -200);
          delay(700);
          sub_State = 6;
          break;
        case 6 : //回転
          setMotorPulse(255, -255);
          if (valRPSD < 150) {
            countPSD += 1;
          } else {
            countPSD = 0;
          }

          if (countPSD > 3) {
            setMotorPulse(255, -255);
            countPSD = 0;
            sub_State = 7;
            delay(700);
          }
          delay(20);
          break;
        case 7 :
          setMotorPulse(255, -255);
          delay(1200);
          sub_State = 8;
        case 8 :
          kabeTrace();
          isCross();
          if (count_Cross > 0) {
            setMotorPulse(0, 0);
            sub_State = 9;
          }
          break;
        case 9 ://安全な方法でメインの線に戻る
          LSpeed = -255;
          RSpeed = 255;
          delaytime = 1000;
          Cross();
          sub_State = 10;
          count_time = 0;
          break;
        case 10 ://しばらくは交差点を無視してライントレース
          lineTrace();
          count_Cross = 0;
          count_time += 1;
          if (count_time > 700) {
            count_time = 0;
            state = 4;
            sub_State = 0;
          }
          break;
      }
      break;
    case 4 : //ボックスまでライントレース
      lineTrace();
      if (count_Cross > 0) {
        setMotorPulse(0, 0);
        state = 5;
      }
      break;
    case 5 : //ボックスに到達し球を入れる
      switch (sub_State) {
        case 0 :
          val_Servo += 20;
          servo.write(val_Servo);
          delay(200);
          if (val_Servo > 1240) {
            sub_State = 1;
          }
          break;
        case 1 :
          setMotorPulse(-100, -100);
          delay(1500);
          sub_State = 2;
          break;
        case 2 :
          setMotorPulse(0, 0);
          break;
      }
      break;
  }
  Serial.print(state);
  Serial.print(" ");
  Serial.print(sub_State);
  Serial.print(" ");
  Serial.print(valMPhotoRef);
  Serial.print(" ");
  Serial.println(count_Cross);

}
