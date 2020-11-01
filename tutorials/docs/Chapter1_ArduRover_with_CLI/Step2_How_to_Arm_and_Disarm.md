# How to Arm and Disarm Ardupilot-SITL Rover

*Author:* Masoud Iranmehr

*Github Page:* [github.com/masoudir/Ardupilot_MAVROS_Examples](https://github.com/masoudir/Ardupilot_MAVROS_Examples)

# Initialize

Please follow steps mentioned in this [link](../index.md)

## Arm and Disarm

## Arm and Disarm using MAVProxy or Ardupilot-SITL

You can send your commands directly from your terminal provided by MAVProxy or SITL. For this just type this command:

* `arm throttle` - Arms the robot

* `disarm` - Disarms the robot

## Arm and Disarm using ROS rqt

At first you need to Add two plugin from rqt:

    Plugins -> Topics -> Topic Monitor
    Plugins -> Services -> Service Caller
    
Then in "Topic Monitor" side, you have to tick the topic of "/mavros/state" to view its contents, so that you can get 
the vehicle mode and also some other parameters such as "arm status" and "Guided status".

Then in "Service Caller" side, you have to select "/mavros/cmd/arming" and then in the field of "value", you can set
your arm status as a bool type variable (False/ True).  Then click on "call" button to send this message. The result should be as follow:

![Screenshot](../img/ch1_rqt_arm.jpg)

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

The command "rostopic echo <TOPIC_NAME>" will return all the contents of the topic every one second. In this case, 
"connected" field shows you that your MAVROS is still connected to the robot. Also "mode" shows you the name of vehicle 
mode at that time.

### Send Arm/Disarm command to robot

Just follow these commands:

* `rosservice call /mavros/cmd/arming True` - Send Arm command to robot

* `rosservice call /mavros/cmd/arming False` - Send Disarm command to robot

Another method is to use "rosrun" command:

* `rosrun mavros mavsafety arm` - Send Arm command to robot

* `rosrun mavros mavsafety disarm` - Send Disarm command to robot


  
