# UC Berkeley EECS149 Fire Detection Drone Project

The UC Berkeley EECS149 Fire Detection Drone Project is a Clone of the Ardupilot Project. If you are looking for the original ArduPilot project, this is probably not what you are looking for! 

Thank you to the ArduPilot development community for creating an easy to use base platform for this class project. Our project builds on the original ArduPilot program with additional code to help it detect and find fires.

# Notes

This project is only developed for APM2.6. The Master branch has been switched to ArduCopter 3.2

If using this code with an RC controller, we could simply upload ArduCopter.pde. However, we want to automate our quadcopter. The problem is that outputs to the motors are through the RC_Channel library.
To start writing our own code, upload AP_Motors_test.pde. This is a good demonstration of how to write to the motors through the RC_Channel library without actually using an RC Controller. You may need to increase the value of the motors.output_test to actually actuate the motors.



# Building the Project

Go to ardupilot/ArduCopter
```
make
```

# Uploading to APM2.6
Go to ardupilot/ArduCopter
```
make upload
```

# Testing using MAVLink and the SITL simulator
We included a modified version of MAVProxy that can send the states we want using the mode module. This can be found in the `mavlink/` directory, with a bulk of the modifications in `mavlink/pymavlink/mavutil.py`. 

# Step 1 - Install modified version of MAVProxy
To install the modified version of MAVProxy, you need to install it to your current system
```
cd mavlink/pymavlink
sudo python setup.py install
```
If this breaks your MAVProxy implementation, you can reinstall the latest unmodified version using `pip`:
```
sudo pip install --upgrade pymavlink MAVProxy
```

# Step 2 - Run the simulator
*You ALWAYS need to run the simulator from the same directory: ArduCopter*
```
cd ArduCopter
make clean
make

# console displays the text console. map displays the interactive map.
# Both are optional. sim_vehicle.sh will drop you in a MAVProxy console.
sim_vehicle.sh --console --map
```

# Step 3 - Changing Modes
You will now be in the MAVProxy console. As of Dec 7, 2015 the implementation is not yet complete. However, MAVProxy is set up to now include different modes. We use the `mode` module to set different modes manually via MAVLink. Currently, we've added FIRE, TAKEOFF, FLY, and OBSTACLE.
To see the list of modes:
```
mode
```

To set the mode:
```
mode <MODE_NAME>
```

Note that the new modes will put you in the wrong state. This is because the mapping from MAVProxy --> ArduCopter is correct, but the mapping from ArduCopter --> MAVProxy is incorrect. I haven't found the offending line of code yet, and I may not bother because it works.
You can verify that the correct states are being called by switching to different modes. This is because different modes have different responses in their init funcitons.
Your mode should change when you switch to fly, fire, or takeoff. These are "correct"
```
STABILIZE> mode fly
LOITER>
```

If you switch to obstacle, there should be no change, because the init function currently returns false.
```
STABILIZE> mode obstacle
STABILIZE>
```
I have added a debug message in the init function for fire mode.
```
STABILIZE> mode fire
APM: No fire yet
ALT_HOLD>
```

# Adding debugging messages in ArduCopter
Debugging messages can be added almost anywhere in ArduCopter's PDE files. The include functions are done for you automatically in their main structure. To send a message, you can use the function `gcs_send_text_P`. A good reference is currently in `ArduCopter/control_fire.pde`.

# Adding control logic to different modes
Modes are defined in files called `control_<modename>.pde`, and have 2 basic functions (init and run). These modes are changed by `flight_mode.pde`. I have redefined the states in `ArduCopter/defines.h` so that an incoming MAVLink `set_mode`message will call the correct state in the `flight_mode.pde` file's `set_mode` function.
From here, they will run the `<modename>_init()` function in the corresponding `control_<modename>.pde` file. Add logic for different states into these control files.

# Snippet from the original Ardupilot team:
#
# ArduPilot Development Team

The ArduPilot project is open source and maintained by a team of volunteers.

To contribute, you can send a pull request on Github. You can also
join the [development discussion on Google
Groups](https://groups.google.com/forum/?fromgroups#!forum/drones-discuss). Note
that the Google Groups mailing lists are NOT for user tech support,
and are moderated for new users to prevent off-topic discussion.
