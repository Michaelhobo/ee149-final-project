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

#define LAND_SPEED    -20          // the descent speed for the final stage of landing in cm/s
#define LAND_DETECTOR_TRIGGER 50    // number of 50hz iterations with near zero climb rate and low throttle that triggers landing complete.
#define LAND_DETECTOR_CLIMBRATE_MAX    30  // vehicle climb rate must be between -30 and +30 cm/s
#define LAND_DETECTOR_BARO_CLIMBRATE_MAX   150  // barometer climb rate must be between -150cm/s ~ +150cm/s
#define LAND_DETECTOR_DESIRED_CLIMBRATE_MAX    -20    // vehicle desired climb rate must be below -20cm/s
#define LAND_DETECTOR_ROTATION_MAX    0.50f   // vehicle rotation must be below 0.5 rad/sec (=30deg/sec for) vehicle to consider itself landed

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

// Global parameters are all contained within the 'g' class.
static Parameters g;


RC_Channel rc1(0), rc2(1), rc3(2), rc4(3);
AP_MotorsQuad   motors(rc1, rc2, rc3, rc4);

TODO: ins needs to take in a param. check how i fixed this in henry_1.1
AP_InertialSensor_MPU6000 ins;

AP_Baro_MS5611 baro(&AP_Baro_MS5611::spi);
AP_GPS gps;
GPS_Glitch gps_glitch(gps);
Baro_Glitch baro_glitch(baro);

AP_Compass_HMC5843 compass;

AP_AHRS_DCM ahrs(ins, baro, gps);

// key aircraft parameters passed to multiple libraries
static AP_Vehicle::MultiCopter aparm;


// Inertial Nav declaration
AP_InertialNav inertialnav(ahrs, baro, gps_glitch, baro_glitch);

//TODO:replace all the g.p_* with PID library stuff

AC_AttitudeControl attitude_control(ahrs, aparm, motors, g.p_stabilize_roll, g.p_stabilize_pitch, g.p_stabilize_yaw,
                        g.pid_rate_roll, g.pid_rate_pitch, g.pid_rate_yaw);


AC_PosControl pos_control(ahrs, inertial_nav, motors, attitude_control,
                        g.p_alt_hold, g.p_throttle_rate, g.pid_throttle_accel,
                        g.p_loiter_pos, g.pid_loiter_rate_lat, g.pid_loiter_rate_lon);

static AC_WPNav wp_nav(inertial_nav, ahrs, pos_control);

// Current location of the copter
// current_loc uses the baro/gps soloution for altitude rather than gps only.
static struct   Location current_loc; //AP_Common.h

// The cm/s we are moving up or down based on filtered data - Positive = UP
static int16_t climb_rate;

// auto_takeoff_start - initialises waypoint controller to implement take-off
static void auto_takeoff_start(float final_alt); //units are cm

// auto_takeoff_run - takeoff in auto mode
//      called by auto_run at 100hz or more
static void auto_takeoff_run();

// auto_land_start - initialises controller to implement a landing
static void auto_land_start(const Vector3f& destination);

// auto_land_run - lands in auto mode
//      called by auto_run at 100hz or more
static void auto_land_run();


// setup
void setup()
{
    hal.console->println("AP_Motors library test ver 1.0");

    // motor initialisation
    motors.set_update_rate(490); //RC_FAST_SPEED
    // motors.set_frame_orientation(AP_MOTORS_X_FRAME);
    motors.set_frame_orientation(AP_MOTORS_PLUS_FRAME);
    motors.set_min_throttle(130);
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

    motors.enable();
    motors.output_min();

    hal.scheduler->delay(1000);
}

// loop
void loop()
{
    int16_t value;

    // display help
    hal.console->println("Press 'm' to run motor orders test, 's' to run stability patch test, 't' to run takeoff_and_land test.  Be careful the motors will spin!");

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
	takeoff_and_land();
    }
}

// stability_test
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
        {   0,      0,      0,      200},
	{   0,      0,      0,      400},
        {   0,      0,      0,      500},
        {   0,      0,      0,      500},
        {   0,      0,      0,      500},
        {   0,      0,      0,      500},
	{   0,      0,      0,      400},
	{   0,      0,      0,      300},
	{   0,      0,      0,      200},
	{   0,	    0, 	    0, 	    100},
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
    //uint32_t testing_array_rows = 32;
    uint32_t testing_array_rows = 7;
 
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
    hal.scheduler->delay(2000);
    }
    // set all inputs to motor library to zero and disarm motors
    motors.set_pitch(0);
    motors.set_roll(0);
    motors.set_yaw(0);
    motors.set_throttle(0);
    motors.output();
    motors.armed(false);

    hal.console->println("finished test.");
}


void takeoff_and_land(){
    auto_takeoff_start(100);
    
    //TODO:implement 100 Hz loop, put time limit (10 seconds = 1000 loops)
    auto_takeoff_run();
    //exit loop
    
    auto_land_start();
    //TODO: implement 100 Hz loop, keep looping while above start_location
    auto_land_run();
    //exit loop
}

// auto_takeoff_start - initialises waypoint controller to implement take-off
static void auto_takeoff_start(float final_alt) //units are cm
{
    //auto_mode = Auto_TakeOff;

    // initialise wpnav destination
    Vector3f target_pos = inertial_nav.get_position();
    target_pos.z = final_alt;
    wp_nav.set_wp_destination(target_pos); //units are cm

    // initialise yaw
    //defined in control_auto.pde, takes in param auto_yaw_mode defined in ArduCopter.pde. will need to implement later
    //set_auto_yaw_mode(AUTO_YAW_HOLD); // pilot controls the heading


    // tell motors to do a slow start
    motors.slow_start(true);
}

// auto_takeoff_run - takeoff in auto mode
//      called by auto_run at 100hz or more
static void auto_takeoff_run()
{
   // if not auto armed set throttle to zero and exit immediately
   //TODO: need flag
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
    wp_nav.update_wpnav();

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


AP_HAL_MAIN();
