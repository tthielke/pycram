
from __future__ import print_function

from iai_apartment_kitchen_msgs.srv import Authenticateuser
import rospy


def call_kitchen_service(command, argument):
    rospy.loginfo('Waiting for kitchen service')
    rospy.wait_for_service(' ') # #TODO: add service name

    try:
        kitchen_service = rospy.ServiceProxy(' ', Authenticateuser)
        resp1 = kitchen_service(command, argument)
        return resp1.resp
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

