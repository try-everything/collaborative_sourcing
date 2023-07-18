# collaborative_sourcing
Collaborative search for pollution sources by multiple drones.

# switch to OFFBOARD and ARMED
rosrun mavros mavsys -n uav0/mavros mode -c OFFBOARD
rosrun mavros mavsafety -n uav0/mavros arm
rosrun mavros mavsys -n uav1/mavros mode -c OFFBOARD
rosrun mavros mavsafety -n uav1/mavros arm
rosrun mavros mavsys -n uav2/mavros mode -c OFFBOARD
rosrun mavros mavsafety -n uav2/mavros arm


TEST
Proportion
UnitTime
