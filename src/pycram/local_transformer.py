import sys
import logging

if 'bullet_world' in sys.modules:
    logging.warning("(publisher) Make sure that you are not loading this module from pycram.bullet_world.")
import rospkg
import rospy
import atexit

from threading import Thread, currentThread
from tf import TransformerROS, transformations
from rospy import Duration, logerr, Rate, is_shutdown
from urdf_parser_py.urdf import URDF

from geometry_msgs.msg import TransformStamped
from .pose import Pose, Transform
from .robot_descriptions import robot_description
from typing import List, Optional, Tuple, Union, Callable


class LocalTransformer(TransformerROS):
    """
    This class allows to use the TF class TransformerROS without using the ROS
    network system or the topic /tf, where transforms are usually published to.
    Instead, a local transformer is saved and allows to publish local transforms,
    as well the use of TFs convenient lookup functions (see functions below).

    This class uses the robots (currently only one! supported) URDF file to
    initialize the tfs for the robot. Moreover, the function update_local_transformer_from_btr
    updates these tfs by copying the tfs state from the pybullet world.

    This class extends the TransformerRos, you can find documentation for TransformerROS here:
    `TFDoc <http://wiki.ros.org/tf/TfUsingPython>`_
    """

    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(LocalTransformer, cls).__new__(cls, *args, **kwargs)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized: return
        super().__init__(interpolate=True, cache_time=Duration(10))

        # TF tf_stampeds and static_tf_stampeds of the Robot in the BulletWorld:
        # These are initialized with the function init_transforms_from_urdf and are
        # used to update the local transformer with the function update_local_transformer_from_btr
        self.tf_stampeds: List[TransformStamped] = []
        self.static_tf_stampeds: List[TransformStamped] = []

        # Since this file can't import bullet_world.py this holds the reference to the current_bullet_world
        self.bullet_world = None

        # If the singelton was already initialized
        self._initialized = True

    def update_objects_for_current_world(self) -> None:
        """
        Updates transformations for all objects that are currently in :py:attr:`~pycram.bullet_world.BulletWorld.current_bullet_world`
        """
        curr_time = rospy.Time.now()
        for obj in list(self.bullet_world.current_bullet_world.objects):
            self.update_transforms_for_object(obj, curr_time)

    def transform_pose(self, pose: Pose, target_frame: str) -> Union[Pose, None]:
        """
        Transforms a given pose to the target frame.

        :param pose: Pose that should be transformed
        :param target_frame: Name of the TF frame into which the Pose should be transformed
        :return: A transformed pose in the target frame
        """
        self.update_objects_for_current_world()
        copy_pose = pose.copy()
        copy_pose.header.stamp = rospy.Time(0)
        if not self.canTransform(target_frame, pose.frame, rospy.Time(0)):
            rospy.logerr(
                f"Can not transform pose: \n {pose}\n to frame: {target_frame}.\n Maybe try calling 'update_transforms_for_object'")
            return
        new_pose = super().transformPose(target_frame, copy_pose)

        copy_pose.pose = new_pose.pose
        copy_pose.header.frame_id = new_pose.header.frame_id
        copy_pose.header.stamp = rospy.Time.now()

        return Pose(*copy_pose.to_list(), frame=new_pose.header.frame_id)

    def transform_to_object_frame(self, pose: Pose,
                                  bullet_object: 'bullet_world.Object', link_name: str = None) -> Union[Pose, None]:
        """
        Transforms the given pose to the coordinate frame of the given BulletWorld object. If no link name is given the
        base frame of the Object is used, otherwise the link frame is used as target for the transformation.

        :param pose: Pose that should be transformed
        :param bullet_object: BulletWorld Object in which frame the pose should be transformed
        :param link_name: A link of the BulletWorld Object which will be used as target coordinate frame instead
        :return: The new pose the in coordinate frame of the object
        """
        if link_name:
            target_frame = bullet_object.get_link_tf_frame(link_name)
        else:
            target_frame = bullet_object.tf_frame
        return self.transform_pose(pose, target_frame)

    def tf_transform(self, source_frame: str, target_frame: str,
                     time: Optional[rospy.rostime.Time] = None) -> Transform:
        """
        Returns the latest known transform between the 'source_frame' and 'target_frame'. If no time is given the last
        common time between the two frames is used.

        :param source_frame: Source frame of the transform
        :param target_frame: Target frame of the transform
        :param time: Time at which the transform should be
        :return:
        """
        self.update_objects_for_current_world()
        tf_time = time if time else self.getLatestCommonTime(source_frame, target_frame)
        translation, rotation = self.lookupTransform(source_frame, target_frame, tf_time)
        return Transform(translation, rotation, source_frame, target_frame)

    def update_transforms_for_object(self, bullet_object: 'bullet_world.Object', time: rospy.Time = None) -> None:
        """
        Updates local transforms for a Bullet Object, this includes the base as well as all links

        :param bullet_object: Object for which the Transforms should be updated
        :param time: a specific time that should be used
        """
        time = time if time else rospy.Time.now()
        for transform in bullet_object._current_link_transforms.values():
            transform.header.stamp = time
            self.setTransform(transform)

    def get_all_frames(self) -> List[str]:
        """
        Returns all know coordinate frames as a list with human-readable entries.

        :return: A list of all know coordinate frames.
        """
        frames = self.allFramesAsString().split("\n")
        frames.remove("")
        return frames

    def transformPose(self, target_frame, ps) -> Pose:
        """
        Alias for :func:`~LocalTransformer.transform_pose` to avoid confusion since a similar method exists in the
        super class.

        :param target_frame: Frame into which the pose should be transformer
        :param ps: Pose that should be transformed
        :return: Input pose in the target_frame
        """
        return self.transform_pose(ps, target_frame)

