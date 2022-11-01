// read serial
void msgRead() {
  // check messages from interface
  if (intf_stf.available()) {
    uint16_t n_byt = 0;
    n_byt = intf_stf.rxObj(pkt, n_byt);

    // Parse message
    msgParse(pkt.cmd, pkt.flag, pkt.value);
  }
  
  // check messages from data logger
  if (data_stf.available()) {
    uint16_t n_byt = 0;
    n_byt = data_stf.rxObj(pkt, n_byt);

    // Parse message
    msgParse(pkt.cmd, pkt.flag, pkt.value);
  }
}

void msgWrite(SerialTransfer &stf, char cmd, bool flag, uint32_t value) {
  uint16_t n_byt = 0;

  // Set struct values
  pkt.cmd   = cmd;
  pkt.flag  = flag;
  pkt.value = value;

  // Stuff buffer with struct
  n_byt = stf.txObj(pkt, n_byt);

  // Send buffer
  stf.sendData(n_byt);
}

// write data to logger
void dataWrite() {
  uint16_t n_byt = 0;
  n_byt = data_stf.txObj(data, n_byt);
  data_stf.sendData(n_byt);
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

  // short-circuit
  if (cmd == 'S') {
    setShortCircuitFlag(value);
  }

  // external alarm
  if (cmd == 'A') {
    handleExternalAlarm(flag);
  }

  // synch timers
  if (cmd == 'T') {
    setRefTime(value);
  }
}
