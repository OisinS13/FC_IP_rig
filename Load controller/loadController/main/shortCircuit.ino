// perform short circuit for a set amount of microseconds
void executeShort() {
    
    // code here
    
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
