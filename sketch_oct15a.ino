int a;

void setup() {
  // put your setup code here, to run once:
  pinMode(A1,INPUT);
  Serial.begin(9800);
  Serial.println("poyo");
}

void loop() {
  // put your main code here, to run repeatedly:
  a = analogRead(A1);
  Serial.println(a);
  delay(100);
}
