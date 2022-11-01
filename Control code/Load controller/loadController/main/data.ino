// Assemble data struct
void aggData() {
  // reference values
  data.I_ref = I_ref;
  data.V_ref = V_ref;
  data.P_ref = P_ref;

  // Operation mode
  data.op_mode = op_mode;

  // Flags
  data.alarm_1    = alarm_1;
  data.alarm_2    = alarm_2;
  data.alarm_ext  = alarm_ext;  

  // Current
  data.I_monitor = readBankCurrent();
  data.I_clamp   = readSensorCurrent();

  // Voltage
  data.V_fc       = readVoltage();
  data.V_fc_spike = readVoltageSpike();

  // time
  data.time_stamp = millis() - ref_time;
}

// Read current
int readBankCurrent() {
  // ask ADC for the current measurement
  // return dummy variable
  return 100;
}

// Read current
int readSensorCurrent() {
  // ask ADC for the current measurement
  // return dummy variable
  return 100;
}

int readVoltage() {
  // ask ADC for the voltage measurement
  // return dummy variable
  return 20;
}

int readVoltageSpike() {
  // ask ADC for the voltage measurement
  // return dummy variable
  return 20;
}
