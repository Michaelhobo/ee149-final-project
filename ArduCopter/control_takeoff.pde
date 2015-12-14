/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

uint32_t timer;
static int16_t start = 1; //used to switch from takeoff to land
uint32_t start_time;
uint32_t current_time;

static void vel_control_start();
static void vel_control_run();

void init_firedrone();
uint8_t starting;

uint32_t takeoff_start_time;

static bool takeoff_init(bool ignore_checks)
{
        motors.set_frame_orientation(AP_MOTORS_X_FRAME);
        set_auto_armed(true);
        float target_height = 100.0f; // height in cm
        guided_takeoff_start(target_height); //Make this modular with params?
}

static void takeoff_run()
{
        hal.console->printf_P(PSTR("takeoff_run()"));
        Vector3f curr_pos = inertial_nav.get_position();
        if (!failsafe.radio) {
                hal.console->printf_P(PSTR("radio ok"));
                if( curr_pos.z > 90 && inertial_nav.get_velocity_z() < 5){
                        hal.console->printf_P(PSTR("set mode fly"));
                        set_mode(FLY);
                }		
                else{
                        hal.console->printf_P(PSTR("calling guided_takeoff_run()"));
                        guided_takeoff_run();
                }
        } else {
                hal.console->printf_P(PSTR("mode land"));
                set_mode(LAND);
        }
}


/* Takeoff, move, then land.
 *   A basic test.
 */
void takeoff_move_land(){

        int16_t throttle_radio_in;
        int16_t avg_out;

        int16_t time_passed = timer-start_time/1000000; //in seconds 
        if(start == 1){
                //auto_takeoff_start(100);
                start++;
                //hal.console->println("ran auto_takeoff_start");
        }
        else if(time_passed < 10){ //takeoff for ten seconds
                //ahrs.update();
                //attitude_control.rate_controller_run(); //sets roll pitch and yaw for the motor as a function of _rate_bf_target
                //motors.output();
                auto_takeoff_run();

                throttle_radio_in = g.rc_3.radio_out;
                avg_out = ((hal.rcout->read(0) + hal.rcout->read(1) + hal.rcout->read(2) + hal.rcout->read(3))/4);

                hal.console->printf_P(PSTR("MOT1:%5d \tMOT2:%5d \tMOT3:%5d \tMOT4:%5d \t THR_IN/AVG_OUT: %d/%d\n"),
                                (int)hal.rcout->read(0),
                                (int)hal.rcout->read(1),
                                (int)hal.rcout->read(2),
                                (int)hal.rcout->read(3),
                                (int)throttle_radio_in,
                                (int)avg_out);
                //hal.console->println("ran auto_takeoff_run");
        }
        else if(start == 2){
                vel_control_start();
                start++;	
        }
        else if(time_passed < 15){ //move xy for 5 seconds
                ahrs.update();
                attitude_control.rate_controller_run(); //sets roll pitch and yaw for the motor
                motors.output();

                /// set_desired_velocity_xy - sets desired velocity in cm/s in lat and lon directions
                ///when update_xy_controller is next called the position target is moved based on the desired velocity and
                ///the desired velocities are fed forward into the rate_to_accel step
                //Usage: pos_control.set_desired_velocity_xy(float vel_lat_cms, float vel_lon_cms) {_vel_desired.x = vel_lat_cms; _vel_desired.y = vel_lon_cms; }
                pos_control.set_desired_velocity_xy(0, 40); //TODO: check which way y-axis is in X-frame orientation. outputing 40 cm/s as target velocity
                vel_control_run();
        }
        else if(start == 3){ 
                auto_land_start();
                start++;
                //hal.console->println("ran auto_land_start");
        }
        else{ //landing
                ahrs.update();                
                attitude_control.rate_controller_run(); //sets roll pitch and yaw for the motor
                motors.output();
                auto_land_run();

                throttle_radio_in = g.rc_3.radio_out;
                avg_out = ((hal.rcout->read(0) + hal.rcout->read(1) + hal.rcout->read(2) + hal.rcout->read(3))/4);

                hal.console->printf_P(PSTR("MOT1:%5d \tMOT2:%5d \tMOT3:%5d \tMOT4:%5d \t THR_IN/AVG_OUT: %d/%d\n"),
                                (int)hal.rcout->read(0),
                                (int)hal.rcout->read(1),
                                (int)hal.rcout->read(2),
                                (int)hal.rcout->read(3),
                                (int)throttle_radio_in,
                                (int)avg_out);

                //hal.console->println("ran auto_land_run");

        }
}

static void vel_control_start(){

        // initialize vertical speeds and leash lengths
        //pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
        //pos_control.set_accel_z(g.pilot_accel_z);
        pos_control.set_speed_z(-50, 50); //NOTE: check the header file for defined minimums. they seemed high, but maybe for a reason
        pos_control.set_accel_z(100);

        /* initialise velocity controller
         * set roll, pitch lean angle targets to current attitude
         * set target position in xy axis
         * move current vehicle velocity into feed forward velocity
         */ 
        pos_control.init_vel_controller_xyz();
}

static void vel_control_run(){
        float target_yaw_rate = 0;
        float heading = 0; //same as yaw

        // update_velocity_controller_xyz - run the velocity controller - should be called at 100hz or higher
        // velocity targets should be set using set_desired_velocity_xy() method
        // apply desired velocity request to position target
        // run position controller's position error to desired velocity step
        // run velocity to acceleration step
        // run acceleration to lean angle step
        // update z controller
        pos_control.update_vel_controller_xyz();

        //NOTE: get_roll() and get_pitch() return a roll and pitch set in init_vel_controller_xyz(), which is set from ahrs sensors, can set get_auto_heading to 0     
        attitude_control.angle_ef_roll_pitch_yaw(pos_control.get_roll(), pos_control.get_pitch(), heading, true);
        hal.console->printf_P(PSTR("velocity roll: %d \t velocity pitch: %d \t velocity yaw: %d \t"), 
                        (int)pos_control.get_roll(), (int)pos_control.get_pitch(), (int)heading);

}

/* Initialize firedrone data. */
void init_firedrone(){
        // standard gps running. Note that we need a 256 byte buffer for some
        // GPS types (eg. UBLOX)
        hal.uartB->begin(38400, 256, 16);

        /*
           run the timer a bit slower on APM2 to reduce the interrupt load
           on the CPU
         */
        hal.scheduler->set_timer_speed(500);

        barometer.init();

        // Do GPS init
        //gps.init(&DataFlash); DataFlash associated with Logging. we're not implementing logging yet. may be a source of error

        // initialise attitude and position controllers
        attitude_control.set_dt(MAIN_LOOP_SECONDS);
        pos_control.set_dt(MAIN_LOOP_SECONDS);

        // initialise inertial nav
        inertial_nav.init();

        // read Baro pressure at ground
        barometer.calibrate();

        // initialise ahrs (may push imu calibration into the mpu6000 if using that device).
        ahrs.init();
        ahrs.set_vehicle_class(AHRS_VEHICLE_COPTER);

        // Turn on MPU6050 - quad must be kept still as gyros will calibrate
        ins.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_100HZ);

        // reset ahrs gyro bias
        ahrs.reset_gyro_drift();

        // setup fast AHRS gains to get right attitude
        ahrs.set_fast_gains(true);

        // set landed flag
        set_land_complete(true);

}
