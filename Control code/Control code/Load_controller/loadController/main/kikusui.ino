// Set current reference
void setCurrent(int I_ref) {
  // Apply PWM on constant current pin
  int out = PWM_range * I_ref / max_current;
  analogWrite(CC_pin, out);
}


// Set voltage reference
void setVoltage(int V_ref) {
  // Apply PWM on constant voltage pin
  int out = PWM_range * V_ref / max_voltage;
  analogWrite(CV_pin, out);
}


// Set power reference
void setPower(int P_ref) {
  // Apply PWM on current/power/resistance pin
  int out = PWM_range * P_ref / max_power;
  analogWrite(CC_pin, out);
}


// Set alarm
void setAlarm(bool ON) {
  // Alarms are set by pulling low
  if (ON) {
  digitalWrite(set_alarm_pin, LOW);
  } else digitalWrite(set_alarm_pin, HIGH);
}


// Clear all existing alarms
void clearAlarms() {
  // Alarms are cleared on a rising edge
  // High/low is flipped in hardware so:
  digitalWrite(clear_pin, HIGH);
  digitalWrite(clear_pin, LOW);
}


// Read alarm 1
void statusAlarm1() {
  if (digitalRead(alarm_1_pin) == HIGH) {
    alarm_1 = true;
  } else alarm_1 = false;
}


// Read alarm 2
void statusAlarm2() {
  if (digitalRead(alarm_2_pin) == HIGH) {
    alarm_2 = true;
  } else alarm_2 = false;
}


// Set load on/off switch
void setLoad(bool flag) {
  if (flag) {
    digitalWrite(load_pin, HIGH);
  } else digitalWrite(load_pin, LOW);
}
