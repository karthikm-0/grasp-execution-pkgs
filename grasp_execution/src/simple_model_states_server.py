#!/usr/bin/env python

from grasp_execution.srv import GetModelStates, GetModelStatesResponse
from gazebo_msgs.msg import ModelStates
import rospy


def states_callback(data):
    global model_states_msg
    model_states_msg = ModelStates()
    model_states_msg = data
    print(model_states_msg)
    sub.unregister()


def return_model_states(req):
    #print ("Model states retrieved.")
    # print(model_states_msg)
    return GetModelStatesResponse(model_states_msg)


def main():
    rospy.init_node('get_model_states_server')
    rospy.Service('get_model_states', GetModelStates, return_model_states)
    print("Get model states server initialized.")

    global sub
    sub = rospy.Subscriber("/gazebo/model_states",
                           ModelStates, states_callback)
    print("Subscribed to /gazebo/model_states.")

    rospy.spin()


if __name__ == "__main__":
    main()
