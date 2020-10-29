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




## Arm and Disarm using Python

### Getting Arm and Disarm status

In the first step, we want to read the Arming status of our robot. For this, we have to use "/mavros/state" topic. To create connection to this topic, we have to get the type of this topic. For this, you can use this command:

* `rostopic type /mavros/state` - Shows the type of /mavros/state topic

The result should be as follows:

    mavros_msgs/State

In python we have access to "mavros_msgs/State" library from package "mavros_msgs.msg". After that we have to write a python Subscriber which runs as an *event_handler*. The final code for reading Arming status would be:

    import rospy
    import mavros_msgs.msg
      
    def callback(data):
        print("all data is:", data)
        print("Robot Arm Status:", data.armed)
    
    
    def listener():
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/mavros/state", mavros_msgs.msg.State, callback)
        rospy.spin()
    
    
    if __name__ == "__main__":
        listener()

This code creates a ros listener node for "/mavros/state" topic which it's type is "". The results should be similar to the following:

    all data is: header: 
      seq: 299
      stamp: 
        secs: 1595581934
        nsecs: 491440912
      frame_id: ''
    connected: True
    armed: False
    guided: False
    manual_input: True
    mode: "MANUAL"
    system_status: 4
    Robot Arm Status: False

Which we need "armed" field, so that we have to use "data.armed" which is mention in above.

### Arming robot

To accomplish arming your ardupilot robot, we have to use a ros service. To get all lists of available ros services, we have to use this command:

* `rosservice list` - Shows all available ros services

We have to use "/mavros/cmd/arming" service. For getting the type of this service, we have to use this command:

* `rosservice type /mavros/cmd/arming` - Shows the type of ros service

The result should be as follows:

    mavros_msgs/CommandBool

So we have to use "CommandBool" package inside python ros service. Finally the python code for write arm status would be as follows:

    import rospy
    from mavros_msgs.srv import CommandBool
    
    
    def caller():
        rospy.init_node('caller', anonymous=True)
        rate = rospy.Rate(10)
        rospy.loginfo("waiting for ROS services")
        service_timeout = 30
        try:
            rospy.wait_for_service('/mavros/cmd/arming', service_timeout)
            rospy.loginfo("ROS service is Up")
        except rospy.ROSException:
            print("Failed")
    
        set_arming_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        set_arming_srv(True)
        rospy.spin()
    
    if __name__ == "__main__":
        try:
            caller()
        except rospy.ROSInternalException:
            pass


Finally if we want to both read the Arming status and set Arming value to robot simultaneously, we could use this python code:
    
    import rospy
    from mavros_msgs.srv import CommandBool
    import mavros_msgs.msg
    
    arm_status = False
    
    
    def callback(data):
        print("Robot Arm Status:", data.armed)
        global arm_status
        arm_status = data.armed
    
    
    def listener_caller():
        rospy.init_node('listener_caller', anonymous=True)
        rospy.Subscriber("/mavros/state", mavros_msgs.msg.State, callback)
        rate = rospy.Rate(10)
        rospy.loginfo("waiting for ROS services")
        service_timeout = 30
        try:
            rospy.wait_for_service('/mavros/cmd/arming', service_timeout)
            rospy.loginfo("ROS service is Up")
        except rospy.ROSException:
            print("Failed")
    
        set_arming_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        global arm_status
        while not rospy.is_shutdown():
            if not arm_status:
                set_arming_srv(True)
            rate.sleep()
        rospy.spin()
    
    
    if __name__ == "__main__":
        try:
            listener_caller()
        except rospy.ROSInternalException:
            pass

This code will Arm the robot whenever the robot is disarmed. To access to these code directly visit [github.com/masoud-iranmehr/Ardupilot_MAVROS_Examples](https://github.com/masoud-iranmehr/Ardupilot_MAVROS_Examples)










