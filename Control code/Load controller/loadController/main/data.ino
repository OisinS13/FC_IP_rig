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

  // ADC data
  readADC();


  // time
  data.time_stamp = millis() - ref_time;
}




// Read data from the ADC and store in the data struct
void readADC() {
  digitalWrite(CS_pin, LOW);
  SPI.transfer(0x5);  //Burst read of non moving average data
  uint16_t ch1 = SPI.transfer16(0x00);
  uint16_t ch2 = SPI.transfer16(0x00);
  uint16_t ch3 = SPI.transfer16(0x00);
         //ch4 = SPI.transfer16(0x00); 4th channel not connected
  digitalWrite(CS_pin, HIGH);

  // store
  data.V_fc_spike =   101 * 1800 *  ch1 / 4096;
  data.V_fc       =    16 * 1800 *  ch2 / 4096;
  data.I_monitor  =  2.4 * (((10000 + 1000 + 2200)/2200) * 1800 *  ch3 / 4096) / 10 ;
}
