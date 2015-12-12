/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static bool is_fire;
static int fire_count;
static bool fire_init(bool ignore_checks)
{
  is_fire = false;
  fire_count = 1000;
  gcs_send_text_P(SEVERITY_LOW, PSTR("No fire yet"));
  motors.output_min();
  return true;
}
static void fire_run()
{
  gcs_send_text_P(SEVERITY_HIGH, PSTR("FIRE!!!!"));
  if(is_fire) {
    gcs_send_text_P(SEVERITY_HIGH, PSTR("FIRE!!!!"));
    fire_count = 1000;
    is_fire = false;
  } else {
    fire_count -= 1;
    if (fire_count <= 0) is_fire = true;
  }
}
