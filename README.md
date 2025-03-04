# Relative phase transfer based turtle imitation motion ros2 node
**The related paper has been under the review, depending on that result, the public state may change**

This script for quick testing imitating motion on eeUVsim with pre-trained model. It is implemented with ros2.


# Installation
### Tested environment
| Ubuntu version                  | ROS2 Version | Comment                                                  |
| -------------------------- | ------- | ------------------------------------------------------------ |
| 22.04                  | Humble   |  Recommnded                                                            |
| 20.04                  | Galactic   |                                                              |

### Depended libraries
As simulating environment, this node require eeUVsim_Gazebo. You can install from here.

https://github.com/Centre-for-Biorobotics/eeUVsim_Gazebo/

### Quick start

**Launch simulator environment with U-CAT robot**

`ros2 launch eeuv_sim spawn_UCAT.launch.py`

**Launch imitatin node**

with sliding mode controlled flapping

`ros2 launch turtle_motion_rft with_flapping.launch.py`

only imitated stroking

`ros2 launch turtle_motion_rft with_flapping.launch.py`
