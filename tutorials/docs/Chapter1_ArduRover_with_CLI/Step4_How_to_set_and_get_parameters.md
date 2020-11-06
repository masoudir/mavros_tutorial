# How to Get/Set parameters of Ardupilot-SITL Rover

*Author:* Masoud Iranmehr

*Github Page:* [github.com/masoudir/mavros_tutorial](https://github.com/masoudir/mavros_tutorial)


# Initialize

Please follow steps mentioned in this [link](../index.md)


# Get/Set MAVROS Parameters

## Get/Set MAVROS Parameters using MAVProxy or Ardupilot-SITL

If you want to set a parameter, you can just use this command:

* `param set PARAMETER VALUE` - Sets the new value for the parameter

And if you want to set a parameter, you can just use this command:

* `param fetch PARAMETER` - Gets the value of the parameter

Or

* `param show PARAMETER` - Gets the value of the parameter


For example, if you want to consider "CRUISE_SPEED" as your parameter, you can use these commands to get or set it:

* `param set CRUISE_SPEED 20` - Sets the new value for the parameter

* `param fetch CRUISE_SPEED` - Gets the value of the parameter

Or

* `param show CRUISE_SPEED` - Gets the value of the parameter

## Get/Set MAVROS Parameters using ROS rqt:

At first you need to add two plugin from rqt:

    Plugins -> Services -> Service Caller
    
If you want to set a parameter, then in "Service Caller" side, you have to select "/mavros/param/set" and then in the 
field of "param_id", you can enter your name of parameter and then in the field of "integer" or "real" you can set a new
value for that parameter. Then click on "call" button to send this message. The example result for parameter 
"CRUISE_SPEED" should be as follow:

![Screenshot](../img/ch1_rqt_set_param.jpg)

As you can see the output constitutes a "success" field which is in bool type (False/True) and indicates that this 
process was successfully done or not.


## Change vehicle mode using ROS command lines

You need to publish the destination location via this command:

* `rostopic pub /mavros/setpoint_raw/global mavros_msgs/GlobalPositionTarget -1 "{'longitude': 14.51218,'latitude': 10.15785}"` - Move Rover to the specified destination

Or if you want to specify only latitude (longitude=0):

* `rostopic pub /mavros/setpoint_raw/global mavros_msgs/GlobalPositionTarget -1 "'longitude': 14.51218"` - Move Rover to the specified destination

It is noted that "-1" determines that this command will be only published once. If you want to publish this message 
periodically, you have to remove "-1" argument and also you can use the argument of "-r RATE" to specify the publish rate for this message:

* `rostopic pub /mavros/setpoint_raw/global mavros_msgs/GlobalPositionTarget -r 1"{'longitude': 14.51218,'latitude': 10.15785}"` - Move Rover to the specified destination with 1 Hz rate

