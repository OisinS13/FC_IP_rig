// perform short circuit for a set amount of microseconds
void executeShort(int dt) {

  // open IGBT
  toggleIGBT(igbt_selected, HIGH);
  digitalWrite(out_flag_pin, HIGH);

  // delay
  int start_time = micros();

  // dont use delay here
  delayMicroseconds(dt);

  // close IGBT
  toggleIGBT(igbt_selected, LOW);
  digitalWrite(out_flag_pin, LOW);

  // close flag
  short_flag = false;

  if (USB_flag) {
    Serial.println("Short-circuit was performed for (uS):");
    Serial.println(short_period);
    Serial.println("with delay (uS)");
    Serial.println(start_time - time_stamp);
  }
}


// open/close IGBT's
void toggleIGBT(int n, bool ON) {

  // turn on igbt 1
  if (n == 1 && ON == true) {
    digitalWrite(igbt_1_pin, HIGH);
  }

  // turn on igbt 2
  if (n == 2 && ON == true) {
    digitalWrite(igbt_2_pin, HIGH);
  }

  // turn off igbt 1
  if (n == 1 && ON == false) {
    digitalWrite(igbt_1_pin, LOW);
  }

  // turn off igbt 2
  if (n == 2 && ON == false) {
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
  OptimisationParameters.Mean_V_per_step[1] = 0;                   //Clear this steps data for calculation
  for (int i = 0; i < Cycles_per_step; i++) {
    OptimisationParameters.Mean_V_per_step[1] += OptimisationParameters.Mean_V_per_cycle[i];
  }
  OptimisationParameters.Mean_V_per_cycle[1] /= Cycles_per_step;
  if (OptimisationParameters.Mean_V_per_cycle[1] > OptimisationParameters.Mean_V_per_cycle[0]) {
    OptimisationParameters.Parameters[OptimisationParameters.Parameter_flag] += OptimisationParameters.Step_size[OptimisationParameters.Parameter_flag];
  } else {
    OptimisationParameters.Step_size[OptimisationParameters.Parameter_flag] /= -0.618;                                                        // new step = - (old step/phi)
    if (abs(OptimisationParameters.Step_size[OptimisationParameters.Parameter_flag]) < OptimisationParameters.Minimum_step_size[OptimisationParameters.Parameter_flag]) {  //If we have reached the minimum step size for this parameter, change parameter
      OptimisationParameters.Parameter_flag ^= 1;                                                     //Flip the parameter flag bit
      OptimisationParameters.Step_size[OptimisationParameters.Parameter_flag]*=OptimisationParameters.Step_size_multiplier; //Multiply the old smallest step size in order to speed up convergence
    } else {
      OptimisationParameters.Parameters[OptimisationParameters.Parameter_flag] += OptimisationParameters.Step_size[OptimisationParameters.Parameter_flag];
    }
  }
}