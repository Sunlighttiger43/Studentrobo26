# sr2026
Student robotics group for 2025/26

I am going to list the functions of OlavesRobot here, their Inputs and Outputs and any sidenotes you should be aware of (so as not to overwhelm)
Sidenote for robot.py - any year specific code (such as pick acid box), goes there, or alternatively create a new python file and call it within robot.py

OLAVESROBOT

Class defining:
    Note the marker ids must be edited every year so that we are able to recognise and differentiate between wall markers and box markers
    Pid Controls should remain as many of the functions require it, but at some point, this needs to be understood and dealt with as the ratios are currently off
    Most of the stuff is pretty self explanatory
    Only enable servos if engineers say is in use

def _load_pid_tunings_from_file()

    Outputs dictionary
    Tunings exists as a json file in the usb stick
    Edit those to figure out which pid tuning values makes the robot move straight

def _initialise_PID_controllers()
    Outputs nothing
    does what it says it does

def get_comp_time()
    Outputs float, which is time in seconds
    A way to help strategisation

def get_motor_ticks()
    # Not a function to be used in robot.py

> [!IMPORTANT]  
> ![spin-record](https://github.com/user-attachments/assets/7d535723-68db-4271-adf9-a4d496c18155)
