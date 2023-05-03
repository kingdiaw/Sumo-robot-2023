//Test code: Sensor sumobesi
/*
Line sensor:
Kanan-depan->D11
*/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(10);
  Serial.println("Test Sensor Kanan-Depan");
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100);
  int s_kanan_depan;
  int s_kiri_depan;

  s_kanan_depan = analogRead(A1)<990? 1:0;
  s_kiri_depan = analogRead(A0)<965? 1:0;
  
  Serial.print(s_kiri_depan);
  Serial.print("\t");
  Serial.println(s_kanan_depan);


}
