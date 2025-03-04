# Relative phase transfer based turtle imitation motion ros2 node

![overview](https://github.com/user-attachments/assets/c7be8045-2285-44e5-a50c-6c1399069f63)


**The related paper has been under the review, depending on that result, the public state may change**

This script for quick testing imitating motion on eeUVsim with pre-trained model. It is implemented with ros2. This implementation is based on the machine learning with tensorflow, but it should work on tiny CPU environment without GPU.

# Installation
### Tested environment
| Ubuntu version                  | ROS2 Version | Comment                                                  |
| -------------------------- | ------- | ------------------------------------------------------------ |
| 22.04                  | Humble   |  Recommnded                                                            |
| 20.04                  | Galactic   |                                                              |

### Depended libraries
As simulating environment, this node require eeUVsim_Gazebo. You can install from here.

https://github.com/Centre-for-Biorobotics/eeUVsim_Gazebo/

| Package                                                      | Version      | Comment                                                      |
| ------------------------------------------------------------ | ------------ | ------------------------------------------------------------ |
| eeUVsim                                                      | 1.00         | For dynamics and kinematics simulation on Gazebo classic     |
| tensorflow                                                   | 2.13.1       | For predicting imitated motion with pre-trained weights      |


### Quick start

**Launch simulator environment with U-CAT robot**

`ros2 launch eeuv_sim spawn_UCAT.launch.py`

**Launch imitatin node**

with sliding mode controlled flapping

`ros2 launch turtle_motion_rft with_flapping.launch.py`

only imitated stroking

`ros2 launch turtle_motion_rft only_stroke.launch.py`
