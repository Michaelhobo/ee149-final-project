/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#define FIRE_HOLD 0
#define FIRE_FIND 1
int fire_mode;
static bool fire_init(bool ignore_checks)
{
  guided_pos_control_start();
  fire_mode = FIRE_HOLD;
  return true;
}
static void fire_run()
{
  if (rangefinder_works) {
    if (DistanceSensorReader->voltage_average()*1023/5 < rangefinder_low) {
      set_mode(OBSTACLE);
    }
  }
  
  if (fire_mode == FIRE_HOLD) {
    // TODO if no fire or fire too long, move back. else, stay
    guided_pos_control_run();
  } else {
    //move back. If no fire, move forward until fire again. 
  }
}
