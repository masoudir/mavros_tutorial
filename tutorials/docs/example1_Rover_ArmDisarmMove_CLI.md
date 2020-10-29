# How to Arm and Disarm Ardupilot-SITL Rover in MAVROS 

*Author:* Masoud Iranmehr

*Github Page:* [github.com/masoud-iranmehr/Ardupilot_MAVROS_Examples](https://github.com/masoud-iranmehr/Ardupilot_MAVROS_Examples)

## Initialize
You just need to follow these steps (has been mentioned in [Quick Start](index.md))

### Bringing up Ardupilot-SITL for Rover

* `cd <Your_Ardupilot_Instalation_Folder>/ardupilot/Tools/autotest/` - Jump to ardupilot folder

* `python3 sim_vehicle.py -v Rover` - Start Rover vehicle

Note that if you want to show other SITL modules such as map or console, you can use these commands:

    python3 sim_vehicle.py -v Rover --map --console
    
Or alternatively you can mention them inside SITL terminal:

    module load map
    
    module load console

Note: Please be patient when ardupilot is compiling robots at first time. This takes 2-3 minutes to complete.

You can see that ardupilot-sitl created some outputs such as "127.0.0.1:14550" "127.0.0.1:14551". These are auxiliary UDP ports for communicating this vehicle to another MAVProxy console. We use these ports in examples.    

### Connect Rover to MAVROS

* `source <ROS_INSTALL_PATH>/devel/setup.bash` - Defines the installed folder of ROS (You can insert this command at the bottom of ~/.bashrc file to automatically run this command while opening a new shell. For this, you can use this command: `sudo nano ~/.bashrc`)

* `source <MAVROS_INSTALL_PATH>/devel/setup.bash` - Defines the installed folder of MAVROS (You can insert this command at the bottom of ~/.bashrc file to automatically run this command while opening a new shell. For this, you can use this command: `sudo nano ~/.bashrc`)

* `roscore` - Brings up ros core for accessing its functions and built packages

* `roslaunch mavros apm.launch fcu_url:=udp://:14550@` - Connects vehicle from UDP:14550 port to MAVROS 

Note: In order to make this document more useful, I have assumed that you put sourcing commands described above on the "`/.bashrc" file to be executed automatically. So that I have eliminated sourcing commands from the next steps.


## Arm and Disarm using MAVProxy or Ardupilot-SITL

You can send your commands directly from your terminal provided by MAVProxy or SITL. For this just type this command:

* `arm throttle` - Arms the robot

* `disarm` - Disarms the robot

### Change vehicle mode
You can send your commands directly from your terminal provided by MAVProxy or SITL. For example if you want to change vehicle mode to *"GUIDED"*, you can type this command:

* `GUIDED` - Change mode to GUIDED

Also you can view all of available modes for your vehicle via this command:

* `mode` - Shows all available modes


### Execute GOTO in vehicle

You can use this command for forcing vehicle to move:

* `GUIDED LAT LON ALTITUDE` - Changes the mode to GUIDED and force vehicle to move to the point of (LAT, LON, ALT)

### Stopping vehicle from moving

You can use this command for forcing vehicle to move:

* `hold` - Stops vehicle from moving

Or

* `mode hold` - Stops vehicle from moving

For reverse, you have to change mode to *"GUIDED"*:

* `mode guided` - Stops vehicle from moving

## Arm and Disarm using ROS commands lines

### Monitoring Rover general status (Arm status and Mode name)
If you want to view the general status of your ardupilot robot in MAVROS, you have to know all the topics created by MAVROS:

* `rostopic list` - Returns all topics available

Then you can see that every part of your robot has been mapped to a specific topic. In order to show robot arming status, you must use "/mavros/state" topic:

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

The command "rostopic echo <TOPIC_NAME>" will return all the contents of the topic every one second. In this case, "connected" field shows you that your MAVROS is still connected to the robot. Also "mode" shows you the name of vehicle mode at that time.

### Send Arm/Disarm command to robot

TODO.......

If your robot wants to move and use its motors, you have to Arm you robot. For sending this command, we have to use ROS services provided by MAVROS:

* `rosservice call /mavros/cmd/arming True` - Send Arm command to robot

* `rosservice call /mavros/cmd/arming False` - Send Disarm command to robot

Another method is to use "rosrun" command:

* `rosrun mavros mavsafety arm` - Send Arm command to robot

* `rosrun mavros mavsafety disarm` - Send Disarm command to robot


  
