# Gripper3F_interface
**Gripper3F Robot interface for controlling the robotic gripper with ROS**
## Description 
This repo allows you to control easily the Gripper3F from your computer with ROS, sending commands (open, close) and changing gripping modes.
Also, is possible to set up the force, speed, and the precise position to go. 

It's designed to work also with the [UR10 robot interface](https://github.com/igor-lirussi/UR10_robot_interface)

| **Method**                | **Description**                                                                 |
|----------------------------|---------------------------------------------------------------------------------|
| **`activate_gripper()`**   | Activates the gripper. Returns `True` if successful.                            |
| **`reset()`**              | Resets the gripper to its default state. Returns `True` if successful.          |
| **`close_gripper()`**      | Closes the gripper hand. Returns `True` if successful.                          |
| **`open_gripper()`**       | Opens the gripper hand. Returns `True` if successful.                           |
| **`set_mode(mode)`**       | Sets the operating mode (`basic`, `pinch`, `wide`, `scissor`). Returns `True` if successful. |
| **`get_mode()`**           | Retrieves the current operating mode as a string.                              |
| **`get_mode_success()`**   | Checks if the last mode retrieval was successful. Returns `True` if successful. |
| **`set_position(position)`**| Sets the gripper position (clamped between `0` and `255`). Returns `True` if successful. |
| **`get_position()`**       | Retrieves the target position of the gripper as an integer.                     |
| **`get_position_fingers()`**| Retrieves positions of individual gripper fingers as a list `[A, B, C]`.       |
| **`get_position_success()`**| Checks if the last position retrieval was successful. Returns `True` if successful. |
| **`set_speed(speed)`**     | Sets the movement speed of the gripper. Returns `True` if successful.           |
| **`get_speed()`**          | Retrieves the current speed setting as an integer.                              |
| **`get_speed_success()`**  | Checks if the last speed retrieval was successful. Returns `True` if successful.|
| **`set_torque(torque)`**   | Sets the torque level of the gripper. Returns `True` if successful.             |
| **`get_torque()`**         | Retrieves the current torque setting as an integer.                             |
| **`get_torque_success()`** | Checks if the last torque retrieval was successful. Returns `True` if successful.|


### Topics:
- Robotics
- Gripper

## Result
![Result](./img/result.jpg)

![Result](./img/image.gif)

## Requirements & Dependencies
- ROS 
- rospy and numpy
- https://github.com/ros-industrial/robotiq build in your system with catkin
- https://github.com/Nishida-Lab/robotiq_3f_ros_pkg build in your system with catkin


## Install 
*   install the Requirements & Dependencies
*   clone this repo in your project

## Run
*   remember to source the setup.bash of catkin
*   run the gripper node
*   use the interface as in the "Code" section

Example:
```bash
# on the external pc

# FOR THE GRIPPER
# source the knowledge for ROS
source /home/ur-colors/UR10_dockerized/catkin_ws/devel/setup.bash 
# activate your env to have your pkgs installed
conda activate ur10_py3.9
# launch the node for the gripper, it moves to calibrate
roslaunch robotiq_3f_driver listener.launch ip_address:=192.168.0.11
```

## Code
```python
from gripper3f_interface import Gripper
import rospy

rospy.init_node("gripper")
rospy.sleep(1.0)
gripper = Gripper()
gripper.activate_gripper()

gripper.close_gripper()
```

## Useful Resources & Extra
- https://github.com/ros-industrial/robotiq
- https://github.com/Nishida-Lab/robotiq_3f_ros_pkg

## Authors
* **Igor Lirussi** @ BOUN Boğaziçi University - CoLoRs (Cognitive Learning and Robotics) Lab

## Acknowledgments
*   All the people that contributed with suggestions and tips.

## License
This project is licensed - see the [LICENSE](LICENSE) file for details.
