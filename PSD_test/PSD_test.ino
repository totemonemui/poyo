// PSD センサのアナログピン
#define PSD_F A4
void setup() { 
  Serial.begin(1000000);
}
void loop() {
  int valF = analogRead(PSD_F); 
  Serial.println(valF); 
  delay(100); 
} 
