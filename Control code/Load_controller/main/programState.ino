// void setRefTime(uint32_t value) {
//   ref_time = millis() + value;
// }


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


// void setShortCircuitFlag(uint32_t value) {
//   if (digitalRead(in_flag_pin)) {
//   // extract short circuit period in [uS]
//   short_period = value;

//   // Requested time may not exceed max short time
//   if (short_period > max_short_time) {
//       short_period = max_short_time;
//     }

//     // request short circuit
//     // time_stamp = micros();
//     short_flag = true;
//   } else {
//     if (USB_flag) {
//       Serial.println("Short circuit requested but in_flag was low.");
//     }
//   }
// }


// Function to set reference current in load bank
void setRefCurrent(uint32_t value) {

  data.I_ref = value;

  // Requested current may not be higher than maximum current
  if (data.I_ref > max_current) {
    data.I_ref = max_current;
  }

  if (USB_flag) {
    Serial.println("Current reference was set to (mA):");
    Serial.println(data.I_ref);
  }
}


// Toggle Load bank ON/OFF
void setLoadFlag(bool flag) {
  if (flag) {
    load_flag = true;
    setLoad(HIGH);

    if (USB_flag) {
      Serial.println("Load is ON");
    }

  } else {
    load_flag = false;
    setLoad(LOW);
    Serial.println("Load is OFF");
  }
}


void setOpMode(uint32_t value) {
  if (value == 1) {
    data.op_mode = 1;
    if (USB_flag) {
      Serial.println("Reference tracking mode was selected.");
    }
  }

  // Power tracking
  if (value == 2) {
    data.op_mode = 2;
    if (USB_flag) {
      Serial.println("MPPT mode was selected.");
    }
  }

  // Efficiency tracking
  if (value == 3) {
    data.op_mode = 3;
    if (USB_flag) {
      Serial.println("MPET mode was selected.");
    }
  }
}
