/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
// Libraries
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_Empty.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel.h>     // RC Channel Library
#include <AP_Motors.h>
#include <AP_Curve.h>
#include <AP_Notify.h>
#include <AP_GPS.h>
#include <DataFlash.h>
#include <AP_InertialSensor.h>
#include <AP_ADC.h>
#include <GCS_MAVLink.h>
#include <AP_Baro.h>
#include <Filter.h>
#include <AP_AHRS.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_Airspeed.h>
#include <AP_Vehicle.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_Mission.h>
#include <StorageManager.h>
#include <AP_Terrain.h>
#include <AP_NavEKF.h>


#include <AP_InertialNav.h>     // Inertial Navigation library
#include <AC_WPNav.h>           // Waypoint Navigation library
#include <AC_PosControl.h>      // Position Control library
#include <AC_AttitudeControl.h> // Attitude Control library
#include <AP_GPS_Glitch.h>      // GPS glitch protection library
#include <AP_Baro_Glitch.h>     // Baro glitch protection library
#include <AP_Buffer.h>          // FIFO buffer library
#include <AC_PID.h>            // PID library
#include <AP_Scheduler.h>
#include "i2cmaster.h"

static float MLX_90614_tempFactor = 0.02; // 0.02 degrees per LSB (measurement resolution of the MLX90614)

static float MLX_90614_tempData = 0x0000; // zero out the data


# define LAND_SPEED    -20          // the descent speed for the final stage of landing in cm/s
# define LAND_DETECTOR_TRIGGER 50    // number of 50hz iterations with near zero climb rate and low throttle that triggers landing complete.
# define LAND_DETECTOR_CLIMBRATE_MAX    30  // vehicle climb rate must be between -30 and +30 cm/s
# define LAND_DETECTOR_BARO_CLIMBRATE_MAX   150  // barometer climb rate must be between -150cm/s ~ +150cm/s
# define LAND_DETECTOR_DESIRED_CLIMBRATE_MAX    -20    // vehicle desired climb rate must be below -20cm/s
# define LAND_DETECTOR_ROTATION_MAX    0.50f   // vehicle rotation must be below 0.5 rad/sec (=30deg/sec for) vehicle to consider itself landed
# define MAIN_LOOP_SECONDS 0.01
# define MAIN_LOOP_MICROS  10000 //originally 10000


 # define RATE_ROLL_P                   0.150f
 # define RATE_ROLL_I                   0.100f
 # define RATE_ROLL_D                   0.004f
 # define RATE_ROLL_IMAX                1000

 # define RATE_PITCH_P                  0.150f
 # define RATE_PITCH_I                  0.100f
 # define RATE_PITCH_D                  0.004f
 # define RATE_PITCH_IMAX               1000

 # define RATE_YAW_P                    0.200f
 # define RATE_YAW_I                    0.020f
 # define RATE_YAW_D                    0.000f
 # define RATE_YAW_IMAX                 1000

 # define STABILIZE_ROLL_P		4.5f
 # define STABILIZE_PITCH_P		4.5f
 # define STABILIZE_YAW_P		4.5f

 # define ALT_HOLD_P            	1.0f

 # define THROTTLE_RATE_P       	5.0f

 # define THROTTLE_ACCEL_P      	0.50f
 # define THROTTLE_ACCEL_I      	1.00f
 # define THROTTLE_ACCEL_D      	0.0f
 # define THROTTLE_ACCEL_IMAX   	800

 # define LOITER_POS_P                  1.0f

 # define LOITER_RATE_P                 1.0f
 # define LOITER_RATE_I                 0.5f
 # define LOITER_RATE_D                 0.0f
 # define LOITER_RATE_IMAX	        1000        // maximum acceleration from I term build-up in cm/s/s

AC_P p_stabilize_roll(STABILIZE_ROLL_P);
AC_P p_stabilize_pitch(STABILIZE_PITCH_P);
AC_P p_stabilize_yaw(STABILIZE_YAW_P);
AC_P p_alt_hold(ALT_HOLD_P);
AC_P p_throttle_rate(THROTTLE_RATE_P);
AC_P p_loiter_pos(LOITER_POS_P);

AC_PID PID_RATE_ROLL(RATE_ROLL_P, RATE_ROLL_I, RATE_ROLL_D, RATE_ROLL_IMAX);

AC_PID PID_RATE_PITCH(RATE_PITCH_P, RATE_PITCH_I, RATE_PITCH_D, RATE_PITCH_IMAX);

AC_PID PID_RATE_YAW(RATE_YAW_P, RATE_YAW_I, RATE_YAW_D, RATE_YAW_IMAX);

AC_PID PID_THROTTLE_ACCEL(THROTTLE_ACCEL_P, THROTTLE_ACCEL_I, THROTTLE_ACCEL_D, THROTTLE_ACCEL_IMAX);	

AC_PID PID_LOITER_RATE_LON(LOITER_RATE_P, LOITER_RATE_I, LOITER_RATE_D, LOITER_RATE_IMAX);

AC_PID PID_LOITER_RATE_LAT(LOITER_RATE_P, LOITER_RATE_I, LOITER_RATE_D, LOITER_RATE_IMAX);

RC_Channel rc1(0), rc2(1), rc3(2), rc4(3);

AP_Baro_MS5611 baro(&AP_Baro_MS5611::spi);

uint16_t throttle_cruise = 450; //just as a starting value like in Newton's approximation for finding zeros
uint16_t throttle_min = 130; //originally hardcoded into the file when we got it


// Time in microseconds of main control loop
uint32_t timer;
uint32_t start_time;
uint32_t current_time;

static int16_t start = 1; //used to switch from takeoff to land

// auto_takeoff_start - initialises waypoint controller to implement take-off
static void auto_takeoff_start(float final_alt); //units are cm

static void auto_land_start();

// auto_land_start - initialises controller to implement a landing
static void auto_land_start(const Vector3f& destination);

static void vel_control_start();
static void vel_control_run();

void init_firedrone();
static void read_MLX90614();
void init_MLX90614();
 /*
  scheduler table - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called
  (in 10ms units) and the maximum time they are expected to take (in
  microseconds)
  1    = 100hz
  2    = 50hz
  4    = 25hz
  10   = 10hz
  20   = 5hz
  33   = 3hz
  50   = 2hz
  100  = 1hz
  1000 = 0.1hz
 */
static bool takeoff_init(bool ignore_checks)
{
  
    hal.console->println("AP_Motors library test ver 1.0");

    // motor initialisation
    motors.set_update_rate(490); //RC_FAST_SPEED
    motors.set_frame_orientation(AP_MOTORS_X_FRAME);
    //motors.set_frame_orientation(AP_MOTORS_PLUS_FRAME);
    motors.set_min_throttle(throttle_min); //throttle_min = 130
    motors.set_mid_throttle(500);
    motors.Init();      // initialise motors
    // setup radio
    if (rc3.radio_min == 0) {
	    // cope with AP_Param not being loaded
	    rc3.radio_min = 1000;
    }
    if (rc3.radio_max == 0) {
	    // cope with AP_Param not being loaded
	    rc3.radio_max = 2000;
    }
    // set rc channel ranges
    rc1.set_angle(4500);
    rc2.set_angle(4500);
    rc3.set_range(130, 1000);
    rc4.set_angle(4500);

    init_firedrone();
    init_MLX90614();
    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));


    motors.enable();
    motors.output_min();

    hal.scheduler->delay(1000);
  return true;
}

static void takeoff_run()
{
	start_time = hal.scheduler->micros();
        current_time = hal.scheduler->micros();
        while((current_time - start_time)/1000000 < 25){ //loop for 25 seconds

		// wait for an INS sample
       		ins.wait_for_sample();
		//hal.console->println("got ins sample");

        	timer = hal.scheduler->micros();	
	
		// used by PI Loops
        	G_Dt                    = (float)(timer - fast_loopTimer) / 1000000.f;
        	fast_loopTimer          = timer;
    
		//hal.console->println("about to run takeoff_move_land");
		takeoff_move_land();
		
		// tell the scheduler one tick has passed
        	scheduler.tick();
	
		// run all the tasks that are due to run. Note that we only
    		// have to call this once per loop, as the tasks are scheduled
    		// in multiples of the main loop tick. So if they don't run on
    		// the first call to the scheduler they won't run on a later
    		// call until scheduler.tick() is called again
    		uint32_t time_available = (timer + MAIN_LOOP_MICROS) - hal.scheduler->micros();
    		scheduler.run(time_available);
		
		//hal.console->println("ran scheduler");
		current_time = hal.scheduler->micros();
  }
	motors.output_min();
}

void init_MLX90614(){
	i2c_init(); //Initialise the i2c bus
	PORTC = (1 << PORTC4) | (1 << PORTC5);//enable pullups
}

void init_firedrone(){
    // standard gps running. Note that we need a 256 byte buffer for some
    // GPS types (eg. UBLOX)
    hal.uartB->begin(38400, 256, 16);

    /*
      run the timer a bit slower on APM2 to reduce the interrupt load
      on the CPU
    */
    hal.scheduler->set_timer_speed(500);

    baro.init();

    // Do GPS init
    //gps.init(&DataFlash); DataFlash associated with Logging. we're not implementing logging yet. may be a source of error

    // initialise attitude and position controllers
    attitude_control.set_dt(MAIN_LOOP_SECONDS);
    pos_control.set_dt(MAIN_LOOP_SECONDS);

    // initialise inertial nav
    inertial_nav.init();

    // read Baro pressure at ground
    baro.calibrate();

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


void takeoff_move_land(){
   
    int16_t throttle_radio_in;
    int16_t avg_out;

    int16_t time_passed = timer-start_time/1000000; //in seconds 
    if(start == 1){
        auto_takeoff_start(100);
	start++;
	//hal.console->println("ran auto_takeoff_start");
    }
    else if(time_passed < 10){ //takeoff for ten seconds
	ahrs.update();
        attitude_control.rate_controller_run(); //sets roll pitch and yaw for the motor as a function of _rate_bf_target
        motors.output();
	auto_takeoff_run();

	throttle_radio_in = rc3.radio_out;
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
      
	throttle_radio_in = rc3.radio_out;
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


static void read_MLX90614() {

    static int32_t dev = 0x5A<<1;
    static int32_t data_low = 0;
    static int32_t data_high = 0;
    static int32_t pec = 0;

    i2c_start_wait(dev+I2C_WRITE);
    i2c_write(0x07);

    // read

    i2c_rep_start(dev+I2C_READ);
    data_low = i2c_readAck(); //Read 1 byte and then send ack
    data_high = i2c_readAck(); //Read 1 byte and then send ack
    pec = i2c_readNak();
    i2c_stop();

    MLX_90614_tempData = (double)(((data_high & 0x007F) << 8) + data_low);
    MLX_90614_tempData = (MLX_90614_tempData * MLX_90614_tempFactor)-0.01;

    hal.console->printf_P(PSTR("%d Kelvins \n"), (int16_t)MLX_90614_tempData); // no need to print but for val checking

}
