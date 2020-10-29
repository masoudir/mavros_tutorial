# How to Arm and Disarm Ardupilot-SITL Rover in MAVROS 

*Author:* Masoud Iranmehr

*Github Page:* [github.com/masoud-iranmehr/Ardupilot_MAVROS_Examples](https://github.com/masoud-iranmehr/Ardupilot_MAVROS_Examples)



## Ignite your robot


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
