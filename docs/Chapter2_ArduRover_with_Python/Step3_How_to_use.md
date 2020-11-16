# How to Drive Ardupilot-SITL Rover

*Author:* Masoud Iranmehr

*Github Page:* [github.com/masoudir/mavros_tutorial](https://github.com/masoudir/mavros_tutorial)

# Initialize

Please follow steps mentioned in this [link](../index.md)

# Requirements

    import threading
    import time
    from mavHandler.roverHandler import *

    
    class MyRoverHandler(RoverHandler):
        def __init__(self):
            super().__init__()
    
            self.user_thread = threading.Timer(0, self.user)
            self.user_thread.daemon = True
            self.user_thread.start()
    
        def user(self):
            while True:
                time.sleep(1)
                print("arm:", self.armed, "mode:", self.mode)
                print("set param:", self.set_param("CRUISE_SPEED", 2, 0))
                if self.connected:
                    print("get param:", self.get_param("CRUISE_SPEED"))
                    print("set param:", self.set_param("CRUISE_SPEED", 10, 0))
                    self.change_mode(MODE_GUIDED)
                    self.arm(True)
                    self.move(50.15189, 10.484885, 0)
    
    
    
    if __name__ == "__main__":
        v = MyRoverHandler()
        v.enable_topics_for_read()
        v.connect("node1", rate=10)
        

In order to run your code please follow the instructions mentioned at this [link](../index.md) to introduce ROS 
functions to this code (Becareful to source the path of ROS and MAVROS on the current terminal in order to use the code).
Then if you download this project with git, you can run your code.

This will create a node with the name of "node1" and then it will connect to the ArduRover vehicle you brought up via the former 
step, then it will set the value of "CRUISE_SPEED" parameter to 2 and then it will change the vehicle
mode to "GUIDED". Then it will ARM the vehicle and then force the vehicle to move to the destination of 
{"lat":50.15189, "lon":10.484885}. 
        