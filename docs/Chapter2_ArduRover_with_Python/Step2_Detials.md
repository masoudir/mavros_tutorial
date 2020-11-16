# Details of "mavros_python_examples"

*Author:* Masoud Iranmehr

*Github Page:* [github.com/masoudir/mavros_tutorial](https://github.com/masoudir/mavros_tutorial)

# Introduction

This package constitutes of four different python classes to make an easy way to work with Ardupilot vehicles. These 
classes are described here.

# Explaining details

## Defining a class for TOPICS and SERVICES

At first, we have created a class named "TopicService" as below to define TOPICS and SERVICES more easily: 

    class TopicService:
    def __init__(self, name: str, classType):
        self.__name = name
        self.__classType = classType
        self.__data = None

    def set_data(self, data):
        self.__data = data

    def get_data(self):
        return self.__data

    def get_type(self):
        return self.__classType

    def get_name(self):
        return self.__name
        
As you see, in this class three parameters has been defined: 

* "name" is pointing to the name of TOPIC or SERVICE, 

* "classType" is pointing at the type of class used

* "data" is pointing to the contents of this TOPIC or SERVICE

## Defining a class for handling ROS topics and services

Then we have defined the second class named "RosHandler" for handling TOPICS and SERVICES more easily in ROS environment.
The content of this class is as below:

    class RosHandler:
    def __init__(self):
        self.rate = 1
        self.connected = False

    def connect(self, node: str, rate: int):
        rospy.init_node(node, anonymous=True)
        self.rate = rospy.Rate(rate)
        self.connected = True
        rospy.loginfo("Rospy is up ...")
        rospy.spin()

    def disconnect(self):
        if self.connected:
            rospy.loginfo("shutting down rospy ...")
            rospy.signal_shutdown("disconnect")
            self.connected = False

    @staticmethod
    def topic_publisher(topic: TopicService):
        pub = rospy.Publisher(topic.get_name(), topic.get_type(), queue_size=10)
        pub.publish(topic.get_data())
        print("edfgedge")

    @staticmethod
    def topic_subscriber(topic: TopicService):
        rospy.Subscriber(topic.get_name(), topic.get_type(), topic.set_data)

    @staticmethod
    def service_caller(service: TopicService, timeout=30):
        try:
            srv = service.get_name()
            typ = service.get_type()
            data = service.get_data()

            rospy.loginfo("waiting for ROS service:" + srv)
            rospy.wait_for_service(srv, timeout=timeout)
            rospy.loginfo("ROS service is up:" + srv)
            call_srv = rospy.ServiceProxy(srv, typ)
            return call_srv(data)
        except rospy.ROSException as e:
            print("ROS ERROR:", e)
        except rospy.ROSInternalException as e:
            print("ROS ERROR:", e)
        except KeyError as e:
            print("ERROR:", e)
        return None

Becareful to execute "topic_subscriber()" function before connecting to the vehicle in MAVROS. This function creates and 
event handler for receiving data from ros.

Then you can connect to your vehicle using "connect()" function. This takes two arguments "name" and "rate", which are 
the name of the node in ROS established by this code and the speed of receiving data from TOPICS in ROS respectively.

It is noted that the actual rate of receiving data from each topic is directly related to the rater defined in that topic.
This means that if a specific topic has been established on the rate of 40 Hz, you can update its data in the rate of 40 Hz
despite the rate you have been used on the function of "connect()".

You can publish your topics via "topic_publisher()" function and also you can send you service commands via "service_caller"
function.

## Defining a class for using Rover functionality

For using Rover vehicle, we need to define its TOPICS and SERVICES and define the rules on how to use them. These are 
accomplished on this class as below:

    class RoverHandler(RosHandler):
    def __init__(self):
        super().__init__()
        self.armed = False
        self.mode = ""

        self.TOPIC_STATE = TopicService("/mavros/state", mavros_msgs.msg.State)
        self.SERVICE_ARM = TopicService("/mavros/cmd/arming", mavros_msgs.srv.CommandBool)
        self.SERVICE_SET_MODE = TopicService("/mavros/set_mode", mavros_msgs.srv.SetMode)
        self.SERVICE_SET_PARAM = TopicService("/mavros/param/set", mavros_msgs.srv.ParamSet)
        self.SERVICE_GET_PARAM = TopicService("/mavros/param/get", mavros_msgs.srv.ParamGet)
        self.TOPIC_SET_POSE_GLOBAL = TopicService('/mavros/setpoint_raw/global', mavros_msgs.msg.GlobalPositionTarget)

        self.thread_param_updater = threading.Timer(0, self.update_parameters_from_topic)
        self.thread_param_updater.daemon = True
        self.thread_param_updater.start()

    def enable_topics_for_read(self):
        self.topic_subscriber(self.TOPIC_STATE)

    def arm(self, status: bool):
        data = mavros_msgs.srv.CommandBoolRequest()
        data.value = status
        self.SERVICE_ARM.set_data(data)
        result = self.service_caller(self.SERVICE_ARM, timeout=30)
        return result.success, result.result

    def change_mode(self, mode: str):
        data = mavros_msgs.srv.SetModeRequest()
        data.custom_mode = mode
        self.SERVICE_SET_MODE.set_data(data)
        result = self.service_caller(self.SERVICE_SET_MODE, timeout=30)
        return result.mode_sent

    def move(self, lat: float, lon: float, alt: float):
        data = mavros_msgs.msg.GlobalPositionTarget()
        data.latitude = lat
        data.longitude = lon
        data.altitude = alt
        self.TOPIC_SET_POSE_GLOBAL.set_data(data)
        self.topic_publisher(topic=self.TOPIC_SET_POSE_GLOBAL)

    def get_param(self, param: str):
        data = mavros_msgs.srv.ParamGetRequest()
        data.param_id = param
        self.SERVICE_GET_PARAM.set_data(data)
        result = self.service_caller(self.SERVICE_GET_PARAM, timeout=30)
        return result.success, result.value.integer, result.value.real

    def set_param(self, param: str, value_integer: int, value_real: float):
        data = mavros_msgs.srv.ParamSetRequest()
        data.param_id = param
        data.value.integer = value_integer
        data.value.real = value_real
        self.SERVICE_SET_PARAM.set_data(data)
        result = self.service_caller(self.SERVICE_SET_PARAM, timeout=30)
        return result.success, result.value.integer, result.value.real

    def update_parameters_from_topic(self):
        while True:
            if self.connected:
                data = self.TOPIC_STATE.get_data()
                self.armed = data.armed
                self.mode = data.mode 
                
As you see, there are some TOPICS and SERVICES defined using the class of "TopicService":

* Topic "/mavros/state" for reading the total status of the vehicle which its class type is "mavros_msgs.msg.State" in python

* Topic "/mavros/setpoint_raw/global" for publishing the destination location of vehicle to move and its class type is "mavros_msgs.msg.GlobalPositionTarget" in python

* Service "/mavros/cmd/arming" for arming the vehicle which its class type is "mavros_msgs.srv.CommandBool" in python

* Service "/mavros/set_mode" for changing the vehicle mode which its class type is "mavros_msgs.srv.SetMode" in python

* Service "/mavros/param/set" for setting the parameters of the vehicle which its class type is "mavros_msgs.srv.ParamSet" in python

* Service "/mavros/param/get" for getting the parameters of the vehicle which its class type is "mavros_msgs.srv.ParamGet" in python





               