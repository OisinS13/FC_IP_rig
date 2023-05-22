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
