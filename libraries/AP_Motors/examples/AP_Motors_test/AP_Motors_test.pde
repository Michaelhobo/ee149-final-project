/*
 *  Example of AP_Motors library.
 *  Code by Randy Mackay. DIYDrones.com
 */

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

// CHANGE Roberto
static AP_HAL::AnalogSource *DistanceSensorReader;
float DistanceSensor;
int flag_o = 0;


float G_Dt = 0.02; //seconds

static float baro_climbrate;

//Documentation of GLobals:
static union {
    struct {
//        uint8_t home_is_set         : 1; // 0
//        uint8_t simple_mode         : 2; // 1,2 // This is the state of simple mode : 0 = disabled ; 1 = SIMPLE ; 2 = SUPERSIMPLE
//        uint8_t pre_arm_rc_check    : 1; // 3   // true if rc input pre-arm checks have been completed successfully
//        uint8_t pre_arm_check       : 1; // 4   // true if all pre-arm checks (rc, accel calibration, gps lock) have been performed
        uint8_t auto_armed          : 1; // 5   // stops auto missions from beginning until throttle is raised
//        uint8_t logging_started     : 1; // 6   // true if dataflash logging has started
        uint8_t land_complete       : 1; // 7   // true if we have detected a landing
//        uint8_t new_radio_frame     : 1; // 8       // Set true if we have new PWM data to act on from the Radio
//        uint8_t CH7_flag            : 2; // 9,10   // ch7 aux switch : 0 is low or false, 1 is center or true, 2 is high
//        uint8_t CH8_flag            : 2; // 11,12   // ch8 aux switch : 0 is low or false, 1 is center or true, 2 is high
//        uint8_t usb_connected       : 1; // 13      // true if APM is powered from USB connection
//        uint8_t rc_receiver_present : 1; // 14  // true if we have an rc receiver present (i.e. if we've ever received an update
//        uint8_t compass_mot         : 1; // 15  // true if we are currently performing compassmot calibration
//        uint8_t motor_test          : 1; // 16  // true if we are currently performing the motors test
//        uint8_t initialised         : 1; // 17  // true once the init_ardupilot function has completed.  Extended status to GCS is not sent until this completes
        uint8_t land_complete_maybe : 1; // 18  // true if we may have landed (less strict version of land_complete)
        uint8_t throttle_zero       : 1; // 19  // true if the throttle stick is at zero, debounced
    };
    uint32_t value;
} ap;


const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;


// main loop scheduler
static AP_Scheduler scheduler;

RC_Channel rc1(0), rc2(1), rc3(2), rc4(3);
AP_MotorsQuad   motors(rc1, rc2, rc3, rc4);

AP_InertialSensor ins;

AP_Baro_MS5611 baro(&AP_Baro_MS5611::spi);
AP_GPS gps;
GPS_Glitch gps_glitch(gps);
Baro_Glitch baro_glitch(baro);

AP_Compass_HMC5843 compass;

AP_AHRS_DCM ahrs(ins, baro, gps);

// key aircraft parameters passed to multiple libraries
static AP_Vehicle::MultiCopter aparm;


// Inertial Nav declaration
AP_InertialNav inertial_nav(ahrs, baro, gps_glitch, baro_glitch);



AC_AttitudeControl attitude_control(ahrs, aparm, motors, p_stabilize_roll, p_stabilize_pitch, p_stabilize_yaw,
                        PID_RATE_ROLL, PID_RATE_PITCH, PID_RATE_YAW);


AC_PosControl pos_control(ahrs, inertial_nav, motors, attitude_control,
                        p_alt_hold, p_throttle_rate, PID_THROTTLE_ACCEL,
                        p_loiter_pos, PID_LOITER_RATE_LAT, PID_LOITER_RATE_LON);

static AC_WPNav wp_nav(inertial_nav, ahrs, pos_control);

uint16_t throttle_avg = 0; //initialization
uint16_t throttle_cruise = 450; //just as a starting value like in Newton's approximation for finding zeros
uint16_t throttle_min = 130; //originally hardcoded into the file when we got it


// Current location of the copter
// current_loc uses the baro/gps soloution for altitude rather than gps only.
static struct   Location current_loc; //AP_Common.h

// The cm/s we are moving up or down based on filtered data - Positive = UP
static float climb_rate;
static float baro_alt;            // barometer altitude in cm above home

// Time in microseconds of main control loop
static uint32_t fast_loopTimer;
uint32_t timer;
uint32_t start_time;
uint32_t current_time;

static int16_t start = 1; //used to switch from takeoff to land

// auto_takeoff_start - initialises waypoint controller to implement take-off
static void auto_takeoff_start(float final_alt); //units are cm

// auto_takeoff_run - takeoff in auto mode
//      called by auto_run at 100hz or more
static void auto_takeoff_run();

static void auto_land_start();

// auto_land_start - initialises controller to implement a landing
static void auto_land_start(const Vector3f& destination);

// auto_land_run - lands in auto mode
//      called by auto_run at 100hz or more
static void auto_land_run();

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
static const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
    { throttle_loop,         2,     450 },
//  { update_GPS,            2,     900 },
    { read_barometer,       10,    1000 },
    { run_nav_updates,       4,     800 },
    { update_thr_cruise,     1,      50 }, //sets throttle_hover which is needed by update_z_controller
//  { compass_accumulate,    2,     420 },
    { barometer_accumulate,  2,     250 },
//  { update_notify,         2,     100 },
    { crash_check,          10,      20 },
//  { gcs_check_input,       2,     550 },
//  { gcs_send_heartbeat,  100,     150 },
//  { gcs_send_deferred,     2,     720 },
//  { gcs_data_stream_send,  2,     950 },
//  { ten_hz_logging_loop,  10,     300 },
//  { fifty_hz_logging_loop, 2,     220 },
//  { read_receiver_rssi,   10,      50 },
    { read_MLX90614,	     2,     500 },
};

// CHANGE Roberto
void Obstacle_Update() {
  DistanceSensor = DistanceSensorReader->voltage_average()*1023/5;
  if(DistanceSensor < 100)
    flag_o = 1;
  else
    flag_o = 0;
}

// setup
void setup()
{
    hal.console->println("AP_Motors library test ver 1.0");
    
    // CHANGE Roberto
    DistanceSensorReader = hal.analogin->channel(0);
    
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
}

// loop
void loop()
{
    int16_t value;
    
    Obstacle_Update();
    hal.console->printf_P(PSTR("Flag Obstacle = %d \n"),(int)flag_o);

    // display help
    hal.console->println("Press 'm' to run motor orders test, 's' to run stability patch test, 't' to run takeoff_move_land test, 'o' to update Obstacle Flag.  Be careful the motors will spin!");

    // wait for user to enter something
    while( !hal.console->available() ) {
        hal.scheduler->delay(20);
    }

    // get character from user
    value = hal.console->read();

    // test motors
    if (value == 'm') {
        motor_order_test();
    }
    if (value == 's') {
	hal.scheduler->delay(7000);
        stability_test();
    }
    if (value == 't') {
  	hal.scheduler->delay(7000);

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
    // CHANGE Roberto
        if (value == 'o')
        {
          Obstacle_Update();
//          hal.console->printf_P(PSTR("Flag Obstacle = %d \n"),(int)flag_o);
//          hal.console->printf_P(PSTR("Distance = %f \n"),(float)DistanceSensor);
        }
	motors.output_min();
    }
}

// motor_test
void motor_order_test()
{
    hal.console->println("testing motor order");
    motors.armed(true);
    for (int8_t i=1; i <= AP_MOTORS_MAX_NUM_MOTORS; i++) {
        hal.console->printf_P(PSTR("Motor %d\n"),(int)i);
        motors.output_test(i, 1300); //previously 1150
        hal.scheduler->delay(300);
        motors.output_test(i, 1000);
        hal.scheduler->delay(2000);
    }
    motors.armed(false);
    hal.console->println("finished test.");

}

// stability_test
void stability_test()
{
    int16_t value, roll_in, pitch_in, yaw_in, throttle_in, throttle_radio_in, avg_out;

    int16_t testing_array[][4] = {
        //  roll,   pitch,  yaw,    throttle
        {   0,      0,      0,      0},
        {   0,      0,      0,      100},
        {   0,      0,      0,      200},
        {   0,      0,      0,      200},
        {   0,      0,      0,      200},
      //  {   0,      0,      0,      300},
      //  {   4500,   0,      0,      300},
      //  {   -4500,  0,      0,      300},
      //  {   0,   4500,      0,      300},
      //  {   0,  -4500,      0,      300},
      //  {   0,      0,   4500,      300},
      //  {   0,      0,  -4500,      300},
      //  {   0,      0,      0,      400},
      //  {   0,      0,      0,      500},
      //  {   0,      0,      0,      600},
      //  {   0,      0,      0,      700},
      //  {   0,      0,      0,      800},
      //  {   0,      0,      0,      900},
      //  {   0,      0,      0,      1000},
      //  {   4500,   0,      0,      1000},
      //  {   -4500,  0,      0,      1000},
      //  {   0,   4500,      0,      1000},
      //  {   0,  -4500,      0,      1000},
      //  {   0,      0,   4500,      1000},
      //  {   0,      0,  -4500,      1000},
      //  {5000,   1000,      0,      1000},
      //  {5000,   2000,      0,      1000},
      //  {5000,   3000,      0,      1000},
      //  {5000,   4000,      0,      1000},
      //  {5000,   5000,      0,      1000},
      //  {5000,      0,   1000,      1000},
      //  {5000,      0,   2000,      1000},
      //  {5000,      0,   3000,      1000},
      //  {5000,      0,   4500,      1000}
    };
    uint32_t testing_array_rows = 5;
 
    hal.console->printf_P(PSTR("\nTesting stability patch\nThrottle Min:%d Max:%d\n"),(int)rc3.radio_min,(int)rc3.radio_max);

    // arm motors
    motors.armed(true);
    
    // run stability test
    for (int16_t i=0; i < testing_array_rows; i++) {
        roll_in = testing_array[i][0];
        pitch_in = testing_array[i][1];
        yaw_in = testing_array[i][2];
        throttle_in = testing_array[i][3];
        motors.set_pitch(roll_in);
        motors.set_roll(pitch_in);
        motors.set_yaw(yaw_in);
        motors.set_throttle(throttle_in);
        motors.output();

	start_time = hal.scheduler->micros();
        current_time = hal.scheduler->micros();
	while((current_time - start_time)/1000000 < 3){ //loop for three seconds
        	
		ahrs.update(); 
		//attitude_control.angle_ef_roll_pitch_yaw(roll_in, pitch_in, yaw_in, true); //calculates error
		attitude_control.rate_controller_run(); //sets roll pitch and yaw for the motor
		motors.output();
			
		// calc average output
        	throttle_radio_in = rc3.radio_out;
        	avg_out = ((hal.rcout->read(0) + hal.rcout->read(1) + hal.rcout->read(2) + hal.rcout->read(3))/4);

        	// display input and output
        	hal.console->printf_P(PSTR("R:%5d \tP:%5d \tY:%5d \tT:%5d\tMOT1:%5d \tMOT2:%5d \tMOT3:%5d \tMOT4:%5d \t ThrIn/AvgOut:%5d/%5d\n"),
                	(int)roll_in,
                	(int)pitch_in,
               		(int)yaw_in,
                	(int)throttle_in,
                	(int)hal.rcout->read(0),
                	(int)hal.rcout->read(1),
                	(int)hal.rcout->read(2),
                	(int)hal.rcout->read(3),
                	(int)throttle_radio_in,
                	(int)avg_out);
		
		current_time = hal.scheduler->micros();
	}
    }
    // set all inputs to motor library to zero and disarm motors
    motors.set_pitch(0);
    motors.set_roll(0);
    motors.set_yaw(0);
    motors.set_throttle(0);
    motors.output();
    motors.armed(false); //this doesn't work

    hal.console->println("finished test.");
    hal.scheduler->delay(3000);
    motors.output_min(); //use this to "turn off" motors
}

// throttle_loop - should be run at 50 hz
// ---------------------------
static void throttle_loop()
{
    //passing in the barometer measured altitude to the inertal navigation since read_inertial_altitude gets its 
    //current altitude from inertial_nav.get_altitude, which only asks for an altitude that is set by the set_altitude function
    inertial_nav.set_altitude(baro_alt);
    inertial_nav.set_velocity_z(baro_climbrate);
    // get altitude and climb rate from inertial lib
    read_inertial_altitude();

    // check if we've landed
    update_land_detector();

    // check auto_armed status
    update_auto_armed();

}

// return barometric altitude in centimeters
static void read_barometer(void)
{
    baro.read();
    //if (should_log(MASK_LOG_IMU)) {
    //    Log_Write_Baro();
    //}
    baro_alt = baro.get_altitude() * 100.0f;
    baro_climbrate = baro.get_climb_rate() * 100.0f;

    hal.console->printf_P(PSTR("baro_alt = %d cm above home \t baro_climbrate = %d cm/s"), (int)baro_alt, (int)baro_climbrate); 
    // run glitch protection and update AP_Notify if home has been initialised
    baro_glitch.check_alt();
    bool report_baro_glitch = (baro_glitch.glitching());//&& !ap.usb_connected && hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED);
    //if (AP_Notify::flags.baro_glitching != report_baro_glitch) {
    //    if (baro_glitch.glitching()) {
    //        Log_Write_Error(ERROR_SUBSYSTEM_BARO, ERROR_CODE_BARO_GLITCH);
    //    } else {
    //        Log_Write_Error(ERROR_SUBSYSTEM_BARO, ERROR_CODE_ERROR_RESOLVED);
    //    }
    //    AP_Notify::flags.baro_glitching = report_baro_glitch;
    //}
}

/*
  try to accumulate a baro reading
 */
static void barometer_accumulate(void)
{
    baro.accumulate();
    //hal.console->println("ran barometer_accumulate");
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

// auto_takeoff_start - initialises waypoint controller to implement take-off
static void auto_takeoff_start(float final_alt) //units are cm
{
    //auto_mode = Auto_TakeOff;

    // initialise wpnav destination
    Vector3f target_pos = inertial_nav.get_position();
    target_pos.z = final_alt;
    hal.console->printf_P(PSTR("target_pos.x = %f \t target_pos.y = %f \t target_pos.z = %f \n"), (float)target_pos.x, (float)target_pos.y, (float)target_pos.z);
    wp_nav.set_wp_destination(target_pos); //units are cm

    // initialise yaw
    //defined in control_auto.pde, takes in param auto_yaw_mode defined in ArduCopter.pde. will need to implement later
    //set_auto_yaw_mode(AUTO_YAW_HOLD); // pilot controls the heading

    motors.armed(true);
    // tell motors to do a slow start
    motors.slow_start(true);
}

// auto_takeoff_run - takeoff in auto mode
//      called by auto_run at 100hz or more
static void auto_takeoff_run()
{
   // if not auto armed set throttle to zero and exit immediately
   if(!ap.auto_armed) {   //ap is a struct in ArduCopter.pde which uses update_auto_armed() (system.pde) to update status of auto_armed flag 

       // initialise wpnav targets
       wp_nav.shift_wp_origin_to_current_pos();
       // reset attitude control targets
       attitude_control.relax_bf_rate_controller();
       attitude_control.set_yaw_target_to_current_heading();
       attitude_control.set_throttle_out(0, false);
       // tell motors to do a slow start
       motors.slow_start(true);
       return;
   }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    //if (!failsafe.radio) {   //failsafe is a struct in ArduCopter.pde
    //    // get pilot's desired yaw rate
    //    target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
    //}

    // run waypoint controller
    wp_nav.update_wpnav(); //calls pos_control.update_xy_controller

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.update_z_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
}

// auto_land_start - initialises controller to implement a landing
static void auto_land_start()
{
    // set target to stopping point
    Vector3f stopping_point;
    wp_nav.get_loiter_stopping_point_xy(stopping_point); //param is address. add & ?

    // call location specific land start function
    auto_land_start(stopping_point);
}

// auto_land_start - initialises controller to implement a landing
static void auto_land_start(const Vector3f& destination)
{
    //auto_mode = Auto_Land;

    // initialise loiter target destination
    wp_nav.init_loiter_target(destination);

    // initialise altitude target to stopping point
    pos_control.set_target_to_stopping_point_z();

    // initialise yaw
    //set_auto_yaw_mode(AUTO_YAW_HOLD);
}

// auto_land_run - lands in auto mode
//      called by auto_run at 100hz or more
static void auto_land_run()
{
    int16_t roll_control = 0, pitch_control = 0;
    float target_yaw_rate = 0;

   // if not auto armed set throttle to zero and exit immediately
   if(!ap.auto_armed || ap.land_complete){ //in land_detector.pde
       attitude_control.relax_bf_rate_controller();
       attitude_control.set_yaw_target_to_current_heading();
       attitude_control.set_throttle_out(0, false);
       // set target to current position
       wp_nav.init_loiter_target();
       return;
    }

   // // relax loiter targets if we might be landed
   // if (land_complete_maybe()) { //TODO. defined in land_detector.pde
   //     wp_nav.loiter_soften_for_landing();
   // }

    //// process pilot's input
    //if (!failsafe.radio) { //TODO. if still receiving input
    //    if (g.land_repositioning) {
    //        // apply SIMPLE mode transform to pilot inputs
    //        update_simple_mode(); //TODO

    //        // process pilot's roll and pitch input
    //        roll_control = g.rc_1.control_in; //TODO
    //        pitch_control = g.rc_2.control_in; //TODO
    //    }

    //    // get pilot's desired yaw rate
    //    target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in); //TODO
    //}

    //added by henry
    roll_control = 0;
    pitch_control = 0;
    target_yaw_rate = 0;

    // process roll, pitch inputs
    wp_nav.set_pilot_desired_acceleration(roll_control, pitch_control);

    // run loiter controller
    wp_nav.update_loiter();

    // call z-axis position controller
    //sets new height target every G_Dt seconds since G_Dt is expected time between calls. if in 100Hz loop, shouldn't G_Dt=.01?
    //pos_control.set_alt_target_from_climb_rate(get_throttle_land(), G_Dt, true); //TODO: get_throttle_land(), G_Dt
    //NOTE:get_throttle_land determines what landing speed. if less than 10 m high, sets as -50 cm/s
    pos_control.set_alt_target_from_climb_rate(LAND_SPEED, G_Dt, true);

    pos_control.update_z_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
}

static void update_thr_cruise()
{
    // ensure throttle_avg has been initialised
    if( throttle_avg == 0 ) {
        throttle_avg = throttle_cruise;
        // update position controller
        pos_control.set_throttle_hover(throttle_avg);
    }

    // if not armed or landed exit
    if (!motors.armed() || ap.land_complete) {
        return;
    }

    //TODO: does this need to be fixed? rc3.radio_out?
    // get throttle output
    int16_t throttle = rc3.servo_out;

    // calc average throttle if we are in a level hover
    if (throttle > throttle_min && abs(climb_rate) < 60 && labs(ahrs.roll_sensor) < 500 && labs(ahrs.pitch_sensor) < 500) {
        throttle_avg = throttle_avg * 0.99f + (float)throttle * 0.01f;
        throttle_cruise = throttle_avg;
        // update position controller
        pos_control.set_throttle_hover(throttle_avg);
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
AP_HAL_MAIN();