//CrossよりかはTurnといった感じ

void isCross() {
  if (valRPhotoRef < IKITI_PHOTO_REF && valMPhotoRef < IKITI_PHOTO_REF && valLPhotoRef < IKITI_PHOTO_REF) { //交差点の検出
    count_Cross += 1;
  }
}

void Cross() { //交差点時の処理
  setMotorPulse(0, 0);
  delay(1000);

  //setMotorPulse(255*RATIO_RLSPEED,255);
  //delay(800);
  /*switch (State_Cross) {
    case 0 : RSpeed = 255;
      LSpeed = 0;
      delaytime =  700;
      break;
    case 1 : RSpeed = -255;
      LSpeed = 255;
      delaytime = 2000;
      break;
    case 2 : RSpeed = 255;
      LSpeed = 0;
      delaytime = 700;
      break;
    case 3 : RSpeed = 0;
      LSpeed = 255;
      delaytime = 700;
      break;
    case 4 : RSpeed = 255;
      LSpeed = -255;
      delaytime = 2000;
      break;
    case 5 : RSpeed = 0;
      LSpeed = 255;
      delaytime = 700;
      break;
    case 6 : RSpeed = 0;
      LSpeed = 0;
      delaytime = 10000;
      break;
  }*/
  setMotorPulse(LSpeed, RSpeed);
  delay(delaytime);
  valMPhotoRef = analogRead(LINE_M);
  while (valMPhotoRef > IKITI_PHOTO_REF) {
    valMPhotoRef = analogRead(LINE_M);
    setMotorPulse(LSpeed, RSpeed);
  }
  count_Cross = 0;
  State_Cross += 1;

}

