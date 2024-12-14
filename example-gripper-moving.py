"""
Description: Example of usage of Gripper3F interface
Author: Igor Lirussi (https://igor-lirussi.github.io)
"""
from gripper3f_interface import Gripper
import rospy

rospy.init_node("gripper")
rospy.sleep(2.0)
gripper = Gripper()
rospy.sleep(2.0)

gripper.activate_gripper()
print('gripper activated')

modes=["basic","wide","scissor","pinch"]
for mode in modes:
    rospy.sleep(2.0)
    print('set mode: ', mode)
    gripper.set_mode(mode)
    rospy.sleep(2.0)

gripper.set_speed(50)

print('closing')
gripper.close_gripper()
print('done')

position=gripper.get_position()
print('position ',position)

print('opening')
gripper.open_gripper()
print('done')