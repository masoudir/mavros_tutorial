# How to Arm and Disarm Ardupilot-SITL Rover in MAVROS 

*Author:* Masoud Iranmehr

*Github Page:* [github.com/masoud-iranmehr/Ardupilot_MAVROS_Examples](https://github.com/masoud-iranmehr/Ardupilot_MAVROS_Examples)

## Initialize

Please follow steps mentioned in this [link](../index.md)

Then follow the steps in this [link](Step1_How_to_Arm_and_Disarm.md) to Arm your vehicle.


## Execute GOTO in vehicle

You can use this command for forcing vehicle to move:

* `GUIDED LAT LON ALTITUDE` - Changes the mode to GUIDED and force vehicle to move to the point of (LAT, LON, ALT)

### Stopping vehicle from moving

You can use this command for forcing vehicle to move:

* `hold` - Stops vehicle from moving

Or

* `mode hold` - Stops vehicle from moving

For reverse, you have to change mode to *"GUIDED"*:

* `mode guided` - Stops vehicle from moving

