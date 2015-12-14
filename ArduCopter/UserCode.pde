/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
// CHANGE Roberto
static AP_HAL::AnalogSource *DistanceSensorReader;
float DistanceSensor;
int flag_o = 0;

#ifdef USERHOOK_INIT
void userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    // CHANGE Roberto
    DistanceSensorReader = hal.analogin->channel(1);  // analog input A1
}
#endif

#ifdef USERHOOK_FASTLOOP
void userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

// CHANGE Roberto
void Obstacle_Update() {
  DistanceSensor = DistanceSensorReader->voltage_average()*1023/5;
  if(DistanceSensor < 100)
    flag_o = 1;
  else
    flag_o = 0;
}
