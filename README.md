# CrazyFlie-ROS2 Interface for Multi-UAV Setup

**Maintained By:** Sanket Salunkhe


## Installation:

```
cd ros2_ws/src
git clone --recurse-submodules https://github.com/sanketsalunkhe12/crazyflie.git
cd ../
colcon build --symlink-install
```


## How to run:

1. Modify the `crazyflie.yaml` as per the configuration of CF Swarm in the Simulator or Real World.

2. Don't run `cfclient` when using ROS2 interface. It create disturbance in communication with ROS2 CF interface.

3. Use the following launch file to run CF interface.
```
ros2 launch crazyflie_interface cf_launch.py backend:=cflib
```


### Simulator SITL:

### Hardware CF:

