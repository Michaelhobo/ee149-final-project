/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static bool takeoff_init(bool ignore_checks)
{

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
        return true;
}

static void takeoff_run()
{
}
