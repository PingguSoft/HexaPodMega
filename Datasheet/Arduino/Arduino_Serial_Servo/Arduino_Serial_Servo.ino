void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.print("#3P500T2000\r\n");//控制舵机3至500，速度为2秒完成
  delay(2000);//等待2秒，等待舵机执行完成
  Serial.print("#3P1500T2000\r\n");//控制舵机3至1500，速度为2秒完成	
  delay(3000);//等待2秒，等待舵机执行完成。并且执行完成后再等待1秒。
  Serial.print("#3P2500T2000\r\n");	
  delay(2000);
  Serial.print("#3P1500T2000\r\n");
  delay(2000);
}
