// read serial
void msgRead() {
  // check messages from interface
  while (intf_stf.available()) {
    uint16_t n_byt = 0;
    n_byt = intf_stf.rxObj(pkt, n_byt);

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


void msgParse(char cmd, bool flag, uint32_t value) {
  // receive ack
  if (cmd == 'K') {
    if (USB_flag) {
     Serial.println("Ack. received"); 
    }
  }

  
}
