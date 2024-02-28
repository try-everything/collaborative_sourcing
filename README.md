# files description
## collaborative_sourcing
Collaborative search for pollution sources by multiple drones. (Not finished)

## concentration_server
Calculate concentration at point(x,y,z).(Not used)

## multi_uav_demo
a state machine demo for testing that one node controls 3 UAVs.

## multi_uav_waypoint
3 UAVs following specific watpoints.

## launch
### multi_uav_mavros_real
For real run.

### multi_uav_mavros_real_single
For real run.(Single uav)

### multi_uav_mavros_sitl
For gazebo simuzation.

# switch to OFFBOARD and ARMED (Terminal)
rosrun mavros mavsys -n uav0/mavros mode -c OFFBOARD
rosrun mavros mavsafety -n uav0/mavros arm
rosrun mavros mavsys -n uav1/mavros mode -c OFFBOARD
rosrun mavros mavsafety -n uav1/mavros arm
rosrun mavros mavsys -n uav2/mavros mode -c OFFBOARD
rosrun mavros mavsafety -n uav2/mavros arm



# notes
TEST
Proportion
UnitTime


face East
