/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static bool kill_init(bool ignore_checks)
{
  motors.output_min();
	return true;
}

static void kill_run()
{
}
