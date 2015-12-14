/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static bool fly_init(bool ignore_checks)
{
	// initialize vertical speeds and leash lengths
        pos_control.set_speed_z(-50, 50); //NOTE: check the header file for defined minimums. they seemed high, but maybe for a reason
        pos_control.set_accel_z(100);

        /* initialise velocity controller
         * set roll, pitch lean angle targets to current attitude
         * set target position in xy axis
         * move current vehicle velocity into feed forward velocity
         */
        pos_control.init_vel_controller_xyz();

	return true;
}
static void continue_fly() {
	Vector3f des_vel(30,0,0); //30 cm/s in the y direction
	pos_control.set_desired_velocity(des_vel);
	firedrone_velocity_run();
}
static void fly_run()
{
  if (rangefinder_works) {
    if ( DistanceSensor = DistanceSensorReader->voltage_average()*1023/5< rangefinder_low) {
      set_mode(OBSTACLE);
    }
  }
  if (MLX90614_works) {
    if (MLX_90614_tempData > MLX90614_heat_high) {
      set_mode(FIRE);
    } else {
      continue_fly();
    }
  } else if (gas_sensor_works) {
    if (MLX_90614_tempData > gas_sensor_high) {
      set_mode(FIRE);
    } else {
      continue_fly();
    }
  } else {
    hal.console->printf_P(PSTR("No sensors!"));
    set_mode(LAND);
  }
}

static void firedrone_velocity_run(){
	 float target_yaw_rate = 0;
        float heading = 0; //same as yaw

         pos_control.update_vel_controller_xyz();

        //NOTE: get_roll() and get_pitch() return a roll and pitch set in init_vel_controller_xyz(), which is set from ahrs sensors, can set get_auto_heading to 0
        attitude_control.angle_ef_roll_pitch_yaw(pos_control.get_roll(), pos_control.get_pitch(), heading, true);


}

static void turn(float heading){
	int16_t target_roll = 0;
	int16_t target_pitch = 0; 
	attitude_control.angle_ef_roll_pitch_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), heading,true);
}


