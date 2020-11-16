# How to move Ardupilot-SITL Rover

*Author:* Masoud Iranmehr

*Github Page:* [github.com/masoudir/mavros_tutorial](https://github.com/masoudir/mavros_tutorial)

# Initialize

Please follow steps mentioned in this [link](../index.md). Then you need to change the mode of your vehicle to "GUIDED"
using this [link](Step1_How_to_change_mode.md) then you have to ARM your robot using one of the mentioned methods on this
[link]. Then it is time to move your vehicle which is described here. 

# Move your vehicle

## Move your vehicle using MAVProxy or Ardupilot-SITL

We assume that your vehicle is in "GUIDED" mode, if it's not please follow the links mentioned in "Initialize" sector on
this page. Then move your vehicle using this command in MAVproxy or SITL:

* `GUIDED lat lon alt` - Move vehicle to specified destination

Parameters of (lat, lon, alt) are the location parameters to determive the destination of the vehicle.

Then if you want to stop vehicle from moving, you need this command:

* `mode HOLD` - Stops the vehicle

Also there are some other modes to make vehicle move such as "RTL", "MANUAL" and etc . Please refer to this 
[link](https://ardupilot.org/dev/docs/rover-sitlmavproxy-tutorial.html) for further information.

## Move your vehicle using ROS rqt:++

We assume that your vehicle is in "GUIDED" mode, if it's not please follow the links mentioned in "Initialize" sector on
this page. Then you need to add this plugin from rqt:

    Plugins -> Topics -> Message Publisher
    
Then you have to add a topic with the address of "/mavros/setpoint_raw/global" and click "+" to add. After that tou need 
to change the value of position ("latitude", "longitude", "altitude") in order to make a destination for the vehicle.
Then tick this topic to be published every 1 second (this duration can be changed) or even right click on the topic and 
then choose "Publish Selected Once" to publish only once. Then you can see that your vehicle moves to the specified 
destination.

## Change vehicle mode using ROS command lines

You need to publish the destination location via this command:

* `rostopic pub /mavros/setpoint_raw/global mavros_msgs/GlobalPositionTarget -1 "{'longitude': 14.51218,'latitude': 10.15785}"` - Move Rover to the specified destination

Or if you want to specify only latitude (longitude=0):

* `rostopic pub /mavros/setpoint_raw/global mavros_msgs/GlobalPositionTarget -1 "'longitude': 14.51218"` - Move Rover to the specified destination

It is noted that "-1" determines that this command will be only published once. If you want to publish this message 
periodically, you have to remove "-1" argument and also you can use the argument of "-r RATE" to specify the publish rate for this message:

* `rostopic pub /mavros/setpoint_raw/global mavros_msgs/GlobalPositionTarget -r 1"{'longitude': 14.51218,'latitude': 10.15785}"` - Move Rover to the specified destination with 1 Hz rate

