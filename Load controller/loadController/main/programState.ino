void setRefTime(uint32_t value) {
  ref_time_ms = millis() + value;
  ref_time_us = micros() + value * 1000;
}


// handle external alarm
void handleExternalAlarm(bool flag) {

  if (flag) {
    alarm_ext = true;
    setAlarm(HIGH);
    if (USB_flag) {
      Serial.println("External alarm activated");
    }
  }

  if (!flag) {
    clearAlarms();
    alarm_ext = false;
    if (USB_flag) {
      Serial.println("Alarms cleared");
    }
  }
}



// Function to set reference current in load bank
void setRefCurrent(uint32_t value) {

  I_ref = value;

  // Requested current may not be higher than maximum current
  if (I_ref > max_current) {
    I_ref = max_current;
  }
}


// Function to set reference current in load bank
void setRefVoltage(uint32_t value) {

  V_ref = value;

  // Requested voltage may not be higher than maximum voltage
  if (V_ref > max_voltage) {
    V_ref = max_voltage;
  }
}


// Toggle Load bank ON/OFF
void setLoadFlag(bool flag) {
  if (flag) {
    load_flag = true;
    setLoad(HIGH);
  } else {
    load_flag = false;
    setLoad(LOW);
  }
}


void setOpMode(uint32_t value) {
  if (value == 1) {
    op_mode = 1;
    // reference tracking mode
  }

  // Power tracking
  if (value == 2) {
    op_mode = 2;
    // MPPT mode
  }

  // Efficiency tracking
  if (value == 3) {
    op_mode = 3;
   // MPET mode
  }
}
