// perform short circuit for a set amount of microseconds
void executeShort(int dt) {


  // open IGBT
  toggleIGBT(HIGH);

  // delay
  // int start_time = micros();


  Last_short_time = micros();

  if (dt > max_short_time) {
    delayMicroseconds(max_short_time);
  } else {
    delayMicroseconds(dt);
  }
  // close IGBT
  toggleIGBT(LOW);


  // if (USB_flag) {
  //   Serial.println("Short-circuit was performed for (uS):");
  //   Serial.println(short_period);
  //   Serial.println("with delay (uS)");
  //   Serial.println(start_time - time_stamp);
  // }
}


// open/close IGBT's
void toggleIGBT(bool ON) {

  // turn on igbt
  if (ON == true) {
    digitalWrite(igbt_1_pin, HIGH);
    digitalWrite(igbt_2_pin, HIGH);
  }

  // turn off igbt
  if (ON == false) {
    digitalWrite(igbt_1_pin, LOW);
    digitalWrite(igbt_2_pin, LOW);
  }
}


// Parameters[2] //0=duration, 1=period
// Parameter_flag //if 0, optimising duraction, if 1, optimising period
// Mean_V_per_cycle[Cycles_per_step]
// Mean_V_per_step[2]
// Cycle_counter
// Cycles_per_step
// Step_size[2] //(signed)
// Minimum_step_size[2] //0 element is for duration, 1 element is for period
// Step_size_multiplier //After switching parameters, increase step size to start re-optimising parameter

void Perturb_and_observ_calcs_2DOF() {

  //take average of cycles for last step //EDITME just take a rolling average for entire step?
  OptimisationParameters.Mean_V_per_step[0] = OptimisationParameters.Mean_V_per_step[1];  //Move last steps data up one index
  OptimisationParameters.Mean_V_per_step[1] = 0;                                          //Clear this steps data for calculation
  for (int i = 0; i < Cycles_per_step; i++) {
    OptimisationParameters.Mean_V_per_step[1] += OptimisationParameters.Mean_V_per_cycle[i];
  }
  OptimisationParameters.Mean_V_per_cycle[1] /= Cycles_per_step;
  if (OptimisationParameters.Mean_V_per_cycle[1] > OptimisationParameters.Mean_V_per_cycle[0]) {
    OptimisationParameters.Parameters[OptimisationParameters.Parameter_flag] += OptimisationParameters.Step_size[OptimisationParameters.Parameter_flag];
  } else {
    OptimisationParameters.Step_size[OptimisationParameters.Parameter_flag] /= -0.618;                                                                                     // new step = - (old step/phi)
    if (abs(OptimisationParameters.Step_size[OptimisationParameters.Parameter_flag]) < OptimisationParameters.Minimum_step_size[OptimisationParameters.Parameter_flag]) {  //If we have reached the minimum step size for this parameter, change parameter
      OptimisationParameters.Parameter_flag ^= 1;                                                                                                                          //Flip the parameter flag bit
      OptimisationParameters.Step_size[OptimisationParameters.Parameter_flag] *= OptimisationParameters.Step_size_multiplier;                                              //Multiply the old smallest step size in order to speed up convergence
    } else {
      OptimisationParameters.Parameters[OptimisationParameters.Parameter_flag] += OptimisationParameters.Step_size[OptimisationParameters.Parameter_flag];
    }
  }
}

void Perturb_and_observ_calcs_1DOF() {

  //take average of cycles for last step //EDITME just take a rolling average for entire step?
  OptimisationParameters.Mean_V_per_step[0] = OptimisationParameters.Mean_V_per_step[1];  //Move last steps data up one index
  OptimisationParameters.Mean_V_per_step[1] = 0;                                          //Clear this steps data for calculation
  for (int i = 0; i < Cycles_per_step; i++) {
    OptimisationParameters.Mean_V_per_step[1] += OptimisationParameters.Mean_V_per_cycle[i];
  }
  OptimisationParameters.Mean_V_per_cycle[1] /= Cycles_per_step;
  if (OptimisationParameters.Mean_V_per_cycle[1] > OptimisationParameters.Mean_V_per_cycle[0]) {
    OptimisationParameters.Parameters[0] += OptimisationParameters.Step_size[0];
  } else {
    if (abs(OptimisationParameters.Step_size[0]) < OptimisationParameters.Minimum_step_size[0]) {  //If we have reached the minimum step size for this parameter, stay there
      OptimisationParameters.Step_size[0] *= -1;
      OptimisationParameters.Parameters[0] += OptimisationParameters.Step_size[0];
    } else {
      OptimisationParameters.Step_size[0] /= -0.618;  // new step = - (old step/phi)
    }
  }
}

void Mean_V_data() {
  readADC();

  uint32_t Time_step = OptimisationParameters.Mean_V_time;
  OptimisationParameters.Mean_V_time = micros();
  Time_step = OptimisationParameters.Mean_V_time - Time_step;
  OptimisationParameters.Mean_V_time_sum += Time_step;
  OptimisationParameters.Mean_V_sum += (data.V_fc * Time_step);
  OptimisationParameters.Mean_V_per_cycle[OptimisationParameters.Cycle_counter] = OptimisationParameters.Mean_V_sum / OptimisationParameters.Mean_V_time_sum;
}
