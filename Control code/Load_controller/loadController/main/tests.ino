void testBank() {

  digitalWrite(load_pin, HIGH);

  analogWrite(CC_pin, 6000);
  delay(1000);

  analogWrite(CC_pin, 12000);
  delay(1000);
  
  analogWrite(CC_pin, 18000);
  delay(1000);

  analogWrite(CC_pin, 24000);
  delay(1000);

  digitalWrite(load_pin, LOW);
  analogWrite(CC_pin, 0);
  delay(1000);

  digitalWrite(set_alarm_pin, HIGH);
  delay(1000);
  digitalWrite(set_alarm_pin, LOW);

  digitalWrite(clear_pin, LOW);
  delay(100);
  digitalWrite(clear_pin, HIGH);
  delay(100);
  digitalWrite(clear_pin, LOW);
  delay(1000);


}
