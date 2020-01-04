#!/usr/bin/env python

from grasp_execution.srv import GetModelStates, GetModelStatesRequest
from gazebo_msgs.msg import ModelStates
import rospy


def retrieve_model_states(req):
    model_states = ModelStates()
    rospy.wait_for_service('get_model_states')
    get_model_states = rospy.ServiceProxy('get_model_states', GetModelStates)
    try:
        model_states = get_model_states(req)
        print(model_states)
        return model_states
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def main():
    retrieve_model_states("G")


if __name__ == "__main__":
    main()
