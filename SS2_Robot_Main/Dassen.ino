void getDassenData(){
  data_photo_ref[3] = data_photo_ref[2];
  data_photo_ref[2] = data_photo_ref[1];
  data_photo_ref[1] = data_photo_ref[0];
  data_photo_ref[0] = valRPhotoRef - valLPhotoRef;
}

void isDassen(){
  if(valRPhotoRef > IKITI_PHOTO_REF && valMPhotoRef > IKITI_PHOTO_REF && valLPhotoRef > IKITI_PHOTO_REF){//脱線の検出
    Serial.println("Dassen");
    count_Dassen += 1;
  }
}

void Dassen(){
  if(count_Dassen > 3){//脱線時の処理
    if(data_photo_ref[3]<0){
      while(valMPhotoRef > IKITI_PHOTO_REF){
        valMPhotoRef = analogRead(LINE_M);
        setMotorPulse(255,0);
      }
    }
    else if(data_photo_ref[3]>0){
      while(valMPhotoRef > IKITI_PHOTO_REF){
        valMPhotoRef = analogRead(LINE_M);
        setMotorPulse(0,255);
      }
    }
    count_Dassen = 0;
  }
}

