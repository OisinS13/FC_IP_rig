bool USB_setup(uint32_t Baud_rate, uint16_t Wait_time_mS) {
  Serial.begin(Baud_rate);
  delay(Wait_time_mS);  //Give USB time to connect EDITME to shortest reliable time
  if (Serial) {
    // USB_flag = 1;  //Used so that when connected via USB, verbose status and error messages can be sent, but will still run if not connected
    Serial.println("Load Controller Connected");
    Serial.println("V.0.1 Oisin Shaw Nov 2023");

    return 1;
  } else {
    Serial.end();
    return 0;
  }
}

// read serial
void msgRead() {
  // check messages from interface
  if (CellVoltageMonitor.available()) {
    uint16_t n_byt = 0;
    n_byt = CellVoltageMonitor.rxObj(rx_msg, n_byt);

    // Parse message
    msgParse(rx_msg.cmd, rx_msg.flag, rx_msg.value);
  }
}



void msgWrite(SerialTransfer &stf, char cmd, bool flag, uint32_t value) {
  uint16_t n_byt = 0;

  // Set struct values
  tx_msg.cmd   = cmd;
  tx_msg.flag  = flag;
  tx_msg.value = value;

  // Stuff buffer with struct
  n_byt = stf.txObj(tx_msg, n_byt);

  // Send buffer
  stf.sendData(n_byt);
}



// write data to logger
void dataWrite() {
  uint16_t n_byt = 0;
  n_byt = DataLogger.txObj(data, n_byt);
  DataLogger.sendData(n_byt);
}



void msgParse(char cmd, bool flag, uint32_t value) {
  // Set Load on/off
  if (cmd == 'L') {
    setLoadFlag(flag);
  }

  // Set operating mode
  if (cmd == 'M') {
    setOpMode(value);
  }

  // Set constant current reference
  if (cmd == 'I') {
    setRefCurrent(value);
  }


  // external alarm
  if (cmd == 'A') {
    handleExternalAlarm(flag);
  }

  // synch timers
  if (cmd == 'T') {
    setRefTime(value);
  }

  // send ack
  if (cmd == 'P') {
    msgWrite(CellVoltageMonitor, 'P', true, 0);
  }
}
