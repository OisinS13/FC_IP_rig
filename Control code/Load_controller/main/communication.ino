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

// // read serial
// void msgRead() {
//   // check messages from interface
//   if (CVM.available()) {
//     uint16_t n_byt = 0;
//     n_byt = CVM.rxObj(rx_msg, n_byt);

//     // Parse message
//     msgParse(rx_msg.cmd, rx_msg.flag, rx_msg.value);
//   }
// }



// void msgWrite(SerialTransfer& stf, char cmd, bool flag, uint32_t value) {
//   uint16_t n_byt = 0;

//   // Set struct values
//   tx_msg.cmd = cmd;
//   tx_msg.flag = flag;
//   tx_msg.value = value;

//   // Stuff buffer with struct
//   n_byt = stf.txObj(tx_msg, n_byt);

//   // Send buffer
//   stf.sendData(n_byt);
// }



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

  // // synch timers
  // if (cmd == 'T') {
  //   setRefTime(value);
  // }

  // // send ack
  // if (cmd == 'P') {
  //   msgWrite(CellVoltageMonitor, 'P', true, 0);
  // }
}


void USB_Set_Current(CommandParameter& Parameters) {
  uint32_t temp = Parameters.NextParameterAsInteger();
  if (temp < 0 || temp > max_current) {
    Serial.println("Requested Current outside allowable range");
  } else {
    data.I_ref = temp;
    setCurrent();
  }
}

void USB_Set_Voltage(CommandParameter& Parameters) {
  uint32_t temp = Parameters.NextParameterAsInteger();
  if (temp < 0 || temp > max_voltage) {
    Serial.println("Requested Voltage outside allowable range");
  } else {
    data.V_ref = temp;
    setVoltage();
  }
}

void USB_Set_Power(CommandParameter& Parameters) {
  uint32_t temp = Parameters.NextParameterAsInteger();
  if (temp < 0 || temp > max_power) {
    Serial.println("Requested Power outside allowable range");
  } else {
    data.P_ref = temp;
    setPower();
  }
}

void USB_Set_Load(CommandParameter& Parameters) {
  bool temp = Parameters.NextParameterAsInteger();
  setLoad(temp);
}

void USB_Set_Mode(CommandParameter& Parameters) {
  uint8_t temp = Parameters.NextParameterAsInteger();
  if (temp < 0 || temp > 3) {
    Serial.println("Requested mode outside allowable range");
  } else {
    data.op_mode = temp;
  }
}

void USB_Clear_Alarms(CommandParameter& Parameters) {
  clearAlarms();
}

void Cmd_Unknown() {
  Serial.println(F("I don't know that command. Try another. "));
}



void USB_Set_Delta(CommandParameter& Parameters) {
  uint32_t temp = Parameters.NextParameterAsInteger();
  if (temp < 0 || temp > max_short_time) {
    Serial.println("Requested duration outside allowable range");
  } else {
   OptimisationParameters.Parameters[0]= temp;
  }
}

void USB_Set_Tau(CommandParameter& Parameters) {
  uint32_t temp = Parameters.NextParameterAsInteger();
   OptimisationParameters.Parameters[1]= temp;
}

void USB_Set_Strategy(CommandParameter& Parameters) {
  uint32_t temp = Parameters.NextParameterAsInteger();
  if (temp < 0 || temp > 4) {
    Serial.println("Requested strategy outside allowable range");
  } else {
   OptimisationParameters.Optimisation_strategy= temp;
  }
}

void USB_Set_Delta_step(CommandParameter& Parameters) {
  uint32_t temp = Parameters.NextParameterAsInteger();
  if (temp < OptimisationParameters.Minimum_step_size[0] || temp > max_short_time) {
    Serial.println("Requested duration step outside allowable range");
  } else {
   OptimisationParameters.Step_size[0]= temp;
  }
}

void USB_Set_Tau_step(CommandParameter& Parameters) {
  uint32_t temp = Parameters.NextParameterAsInteger();
  if (temp < OptimisationParameters.Minimum_step_size[1] || temp > 50000000) {
    Serial.println("Requested period step outside allowable range");
  } else {
   OptimisationParameters.Step_size[1]= temp;
  }
}

void USB_Set_adaptive_window(CommandParameter& Parameters) {
  uint32_t temp = Parameters.NextParameterAsInteger();
  if (temp < -5000 || temp > 5000) {
    Serial.println("Requested adaptive Tau hysteresis outside allowable range");
  } else {
   OptimisationParameters.Adaptive_tau_hysteresis_window= temp;
  }
}