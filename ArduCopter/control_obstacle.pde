/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static bool obstacle_init(bool ignore_checks)
{
  const Vector3f des_vel_run(0,40,10);	
  const Vector3f des_vel_zero(0,0,0);
  vel_control_start();
  return true;
}

static void obstacle_run()
{
  pos_control.set_desired_velocity(des_vel_zero);  // stops when seeing obstacle
  ahrs.update();
  vel_control_run();
  attitude_control.rate_controller_run(); //sets roll pitch and yaw for the motor
  motors.output();
  while(flag_o == 1)
  {
    hal.scheduler->delay(100);
    pos_control.set_desired_velocity(des_vel_run);  // move away
    ahrs.update();
    vel_control_run();
    attitude_control.rate_controller_run(); //sets roll pitch and yaw for the motor
    motors.output();
    Obstacle_Update();
  }
  hal.scheduler->delay(100);
  pos_control.set_desired_velocity(des_vel_zero);   // zero again and exits state
  ahrs.update();
  vel_control_run();
  attitude_control.rate_controller_run(); //sets roll pitch and yaw for the motor
  motors.output();
}