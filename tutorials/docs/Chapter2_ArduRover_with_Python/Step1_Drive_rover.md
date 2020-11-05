# How to Drive Ardupilot-SITL Rover

*Author:* Masoud Iranmehr

*Github Page:* [github.com/masoudir/mavros_tutorial](https://github.com/masoudir/mavros_tutorial)

# Initialize

Please follow steps mentioned in this [link](../index.md)

# Requirements

You just need to clone [mavros_python_examples](https://www.github.com/masoudir/mavros_python_examples) using this 
command:

    git clone https://www.github.com/masoudir/mavros_python_examples
    cd mavros_python_examples
    
Or you can easily install this package via pip:

    pip3 install mavros_python_examples
    
# How to use:

"mavros_python_examples" includes a test file showing you the easiest way to drive rover using this package. To access 
this file please refer to "<MAVROS_PYTHON_EXAMPLES_PATH>/test/rover.py" or you if you have not cloned this project, 
simply you can download this code via this command:

    wget https://github.com/masoudir/mavros_tutorial/test/rover.py
    
The contents of this files is shown below:

    ****

In order to drive your Rover and make it move, you have to follow the instructions [here](../index.md), then just run 
this command:

* `python3 test/rover.py` - If you have cloned this package

* `python3 rover.py` - If you have installed this package via pip3

You can see that the mode of your vehicle changes to "GUIDED" and then this robot will be ARMED and then moves to a 
specific point.

For learning more about the details of this package, refer to this [link](./Step2_Detials.md)
