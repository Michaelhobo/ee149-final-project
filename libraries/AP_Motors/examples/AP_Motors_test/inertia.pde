/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// read_inertia - read inertia in from accelerometers
static void read_inertia()
{
    // inertial altitude estimates
    inertial_nav.update(G_Dt);
}

// read_inertial_altitude - pull altitude and climb rate from inertial nav library
void read_inertial_altitude()
{
    // with inertial nav we can update the altitude and climb rate at 50hz
    current_loc.alt = inertial_nav.get_altitude(); //returns  _position.z, which is set in set_altitude, called in init_arm_motors, called in arm_motors_check
    current_loc.flags.relative_alt = true;
    climb_rate = inertial_nav.get_velocity_z(); //returns _velocity.z
   // hal.console->printf_P(PSTR("inertia_alt = %d \t inertia_climb_rate = %d"), (int)current_loc.alt, (int)climb_rate);
}
