
//無線機器を通じて相方の機体と通信する
//esp,xbeeどちらでも大丈夫(の予定)

int sendMyState(int state){
    if(state >= 0 && state <= 0xFF){
        mySerial.write(state);
        return 0;
    }
    else return -1;
}

int getPartnerState(void){
    //return (int)(mySerial.peak());
}
