
from __future__ import print_function

import sys
from typing import Callable

import rosnode
import rospy
try:
    from iai_apartment_kitchen_msgs.srv import Authenticateuser
except ModuleNotFoundError as e:
    rospy.logwarn(f"Could not import Apartment kitchen messages, Apartment kitchen interface could not be initialized")


def init_kitchen_interface(func: Callable) -> Callable:
    """
    Tries to import the RoboKudo messages and with that initialize the RoboKudo interface.
    """
    def wrapper(*args, **kwargs):

        topics = list(map(lambda x: x[0], rospy.get_published_topics()))
        if "iai_apartment_kitchen_msgs" not in sys.modules:
            rospy.logwarn("Could not initialize the Apartment kitchen interface since the iai_apartment_kitchen_msgs are not imported")
            return

        if "/blum_control_service_server" in rosnode.get_node_names():
            rospy.loginfo("Successfully initialized Apartment kitchen interface")
        else:
            rospy.logwarn("Apartment kitchen is not running, could not initialize Apartment kitchen interface")
            return
        authenticate_user()

        func(*args, **kwargs)
    return wrapper


def authenticate_user():
    rospy.loginfo('Waiting for authentication')
    rospy.wait_for_service('/blum_kitchen_server')

    try:
        kitchen_service = rospy.ServiceProxy('/blum_kitchen_server', Authenticateuser)
        resp1 = kitchen_service('authenticate_chk', 'pycram')
        if resp1.resp == 'FALSE':
            rospy.loginfo('Please press the cyan button on the kitchen gateway')
            resp1 = kitchen_service('authenticate', 'pycram')
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

@init_kitchen_interface
def call_kitchen_service(command, argument):
    rospy.loginfo('Waiting for kitchen service')
    rospy.wait_for_service('/blum_kitchen_server')

    try:
        kitchen_service = rospy.ServiceProxy('/blum_kitchen_server', Authenticateuser)
        resp1 = kitchen_service(command, argument)
        print(resp1)
        return resp1.resp
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")


@init_kitchen_interface
def open(kitchen_element):
    call_kitchen_service('open', kitchen_element)


@init_kitchen_interface
def close(kitchen_element):
    call_kitchen_service('close', kitchen_element)

@init_kitchen_interface
def modules_show():
    return call_kitchen_service('modules_show', ' ')

