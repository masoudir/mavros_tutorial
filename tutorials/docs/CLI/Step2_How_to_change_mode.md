# How to Arm and Disarm Ardupilot-SITL Rover in MAVROS 

*Author:* Masoud Iranmehr

*Github Page:* [github.com/masoud-iranmehr/Ardupilot_MAVROS_Examples](https://github.com/masoud-iranmehr/Ardupilot_MAVROS_Examples)

# Initialize

Please follow steps mentioned in this [link](../index.md)

# Change vehicle mode

## Change vehicle mode using MAVProxy or Ardupilot-SITL

### Get mode

If you want to see the last mode of vehicle, you can just looking for the name before ">" character in these terminals.

### Set mode

Also if you want to change the mode, you can send your commands directly from your terminal provided by MAVProxy or SITL. For example if you want to change vehicle mode to *"GUIDED"*, you can type this command:

* `GUIDED` - Change mode to GUIDED

Also you can view all of available modes for your vehicle via this command:

* `mode` - Shows all available modes

## Change vehicle mode using ROS command lines

### Get mode

If you want to see the last mode of the vehicle, you can just type this command:

* `rostopic echo /mavros/state` - Returns all contents of /mavros/state topic periodically

The results should be as similar as this:

    ---                                                                                                                                                                    
    header:                                                                                                                                                                
      stamp: 09                                                                                                                                                            
        secs: 1595198641                                                                                                                                                   
                                                                                                                                                                           
        nsecs: 651358095                                                                                                                                                   
      frame_id: ''                                                                                                                                                         
    connected: True                                                                                                                                                        
    guided: False                                                                                                                                                          
    manual_input: True                                                                                                                                                     
                                                                                                                                                                           
    mode: "MANUAL"                                                                                                                                                         
    system_status: 4                                                                                                                                                       
    --- 
    
You can find the mode in the field of "mode" of the result shown.

### Set mode

You need to call a ros service in order to do that. Just follow these commands:

* `rosservice call /mavros/cmd/arming True` - Send Arm command to robot

* `rosservice call /mavros/cmd/arming False` - Send Disarm command to robot

Another method is to use "rosrun" command:

* `rosrun mavros mavsafety arm` - Send Arm command to robot

* `rosrun mavros mavsafety disarm` - Send Disarm command to robot
