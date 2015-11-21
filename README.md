# UC Berkeley EECS149 Fire Detection Drone Project

The UC Berkeley EECS149 Fire Detection Drone Project is a Clone of the Ardupilot Project. If you are looking for the original ArduPilot project, this is probably not what you are looking for! 

Thank you to the ArduPilot development community for creating an easy to use base platform for this class project. Our project builds on the original ArduPilot program with additional code to help it detect and find fires.

# Notes

This project is only developed for APM2.6. The Master branch has been switched to ArduCopter 3.2

If using this code with an RC controller, we could simply upload ArduCopter.pde. However, we want to automate our quadcopter. The problem is that outputs to the motors are through the RC_Channel library.
To start writing our own code, upload AP_Motors_test.pde. This is a good demonstration of how to write to the motors through the RC_Channel library without actually using an RC Controller. You may need to increase the value of the motors.output_test to actually actuate the motors.



# Building the Project

Go to ardupilot/ArduCopter
Run make

# Uploading to APM2.6
Go to ardupilot/ArduCopter
Run make upload

# Snippet from the original Ardupilot team:
#
# ArduPilot Development Team

The ArduPilot project is open source and maintained by a team of volunteers.

To contribute, you can send a pull request on Github. You can also
join the [development discussion on Google
Groups](https://groups.google.com/forum/?fromgroups#!forum/drones-discuss). Note
that the Google Groups mailing lists are NOT for user tech support,
and are moderated for new users to prevent off-topic discussion.
