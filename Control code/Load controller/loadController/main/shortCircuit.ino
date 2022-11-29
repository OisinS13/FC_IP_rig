// perform short circuit for a set amount of microseconds
void executeShort(int dt) {

  // open IGBT
  toggleIGBT(igbt_selected, HIGH);
  digitalWrite(out_flag_pin, HIGH);

  // delay
  int start_time = micros();

  // dont use delay here
  delayMicroseconds(dt);

  // close IGBT
  toggleIGBT(igbt_selected, LOW);
  digitalWrite(out_flag_pin, LOW);

  // close flag
  short_flag = false;

  if (USB_flag) {
    Serial.println("Short-circuit was performed for (uS):");
    Serial.println(short_period);
    Serial.println("with delay (uS)");
    Serial.println(start_time - time_stamp);
  }
}


// open/close IGBT's
void toggleIGBT(int n, bool ON) {

  // turn on igbt 1
  if (n == 1 && ON == true) {
    digitalWrite(igbt_1_pin, HIGH);
  }

  // turn on igbt 2
  if (n == 2 && ON == true) {
    digitalWrite(igbt_2_pin, HIGH);
  }

  // turn off igbt 1
  if (n == 1 && ON == false) {
    digitalWrite(igbt_1_pin, LOW);
  }

  // turn off igbt 2
  if (n == 2 && ON == false) {
    digitalWrite(igbt_2_pin, LOW);
  }
}
