#!/usr/bin/env python
import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

def move_box(data):
    state_msg = ModelState()
    state_msg = data

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state(state_msg)

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e    

def main():
    rospy.init_node('set_pose')
    rospy.Subscriber("move_box", ModelState, move_box)
    rospy.spin()

    '''state_msg = ModelState()
    state_msg.model_name = 'cube1'
    state_msg.pose.position.x = 1
    state_msg.pose.position.y = 0
    state_msg.pose.position.z = 0.76
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 0
    state_msg.pose.orientation.w = 0'''

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass