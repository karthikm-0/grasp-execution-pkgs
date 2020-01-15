#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelState
from moveit_msgs.msg import Joint

def modify_cube(joint_state_msg):
    print("LOL")

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("jaco/joint_state", JointState, modify_cube) 

    rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    listener()
