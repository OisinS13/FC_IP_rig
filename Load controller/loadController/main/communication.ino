// read serial
void msgRead(SerialTransfer &stf) {
  // check messages from interface
  while (stf.available()) {

    // read packet ID
    uint8_t ID = stf.currentPacketID();
    uint16_t n_byt = 0;


    // parse packets with ID 0x67
    if (ID == 0x67) {
      n_byt = stf.rxObj(rx_msg, n_byt);
      msgParse(stf, ID, rx_msg.cmd, rx_msg.flag, rx_msg.value);
    }


    // parse packets with other IDs
    if (ID == 0x68) {
        // code here
    }
  }
}





void msgWrite(SerialTransfer &stf, byte ID, char cmd, bool flag, uint32_t value) {
  uint16_t n_byt = 0;

  // Set struct values
  tx_msg.cmd   = cmd;
  tx_msg.flag  = flag;
  tx_msg.value = value;

  // Stuff buffer with struct
  n_byt = stf.txObj(tx_msg, n_byt);

  // Send buffer
  stf.sendData(n_byt, ID);
}



// write data to logger
void dataWrite() {
  uint16_t n_byt = 0;
  n_byt = DL.txObj(data, n_byt);
  byte ID = 0x68;
  DL.sendData(n_byt, ID);
}



void msgParse(SerialTransfer &stf, uint8_t ID, char cmd, bool flag, uint32_t value) {

  // Handle packets of ID = 0x67
  if (ID == 0x67) {
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
    // Set constant voltage reference
    if (cmd == 'V') {
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
    // send ping
    if (cmd == 'P') {
      msgWrite(stf, 0x67, 'P', true, 0);
    }
  }
}
