"""Implementation of helper functions and classes for internal usage only.

Functions:
_block -- wrap multiple statements into a single block.

Classes:
GeneratorList -- implementation of generator list wrappers.
"""
from inspect import isgeneratorfunction
from typing import List
from typing import Tuple, Callable

import numpy as np
import pybullet as p
from pytransform3d.rotations import quaternion_wxyz_from_xyzw, quaternion_xyzw_from_wxyz
from pytransform3d.transformations import transform_from_pq, transform_from, pq_from_transform

from macropy.core.quotes import ast_literal, q
from .bullet_world import Object as BulletWorldObject
from .local_transformer import LocalTransformer
from .pose import Transform
from .robot_descriptions import robot_description
import os


class bcolors:
    """
    Color codes which can be used to highlight Text in the Terminal. For example,
    for warnings.
    Usage:
    Firstly import the class into the file.
    print(f'{bcolors.WARNING} Some Text {bcolors.ENDC}')
    """
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def _apply_ik(robot: BulletWorldObject, joint_poses: List[float], joints: List[str]) -> None:
    """
    Apllies a list of joint poses calculated by an inverse kinematics solver to a robot

    :param robot: The robot the joint poses should be applied on
    :param joint_poses: The joint poses to be applied
    :param gripper: specifies the gripper for which the ik solution should be applied
    :return: None
    """
    # arm ="left" if gripper == robot_description.get_tool_frame("left") else "right"
    # ik_joints = [robot_description.torso_joint] + robot_description._safely_access_chains(arm).joints
    # ik_joints = robot_description._safely_access_chains(arm).joints
    robot.set_joint_states(dict(zip(joints, joint_poses)))
    # for i in range(0, len(joints)):
    #     robot.set_joint_state(joints[i], joint_poses[i])


def _transform_to_torso(pose_and_rotation: Tuple[List[float], List[float]], robot: BulletWorldObject) -> Tuple[
    List[float], List[float]]:
    # map_T_torso = robot.get_link_position_and_orientation("base_footprint")
    # map_T_torso = robot.get_position_and_orientation()
    map_T_torso = robot.get_link_pose(robot_description.torso_link).to_list()
    torso_T_map = p.invertTransform(map_T_torso[0], map_T_torso[1])
    map_T_target = pose_and_rotation
    torso_T_target = p.multiplyTransforms(torso_T_map[0], torso_T_map[1], map_T_target[0], map_T_target[1])
    return torso_T_target


def calculate_wrist_tool_offset(wrist_frame: str, tool_frame: str, robot: BulletWorldObject) -> Transform:
    local_transformer = LocalTransformer()
    tool_pose = robot.get_link_pose(tool_frame)
    wrist_to_tool = local_transformer.transform_pose(tool_pose, robot.get_link_tf_frame(wrist_frame))
    return wrist_to_tool.to_transform(robot.get_link_tf_frame(tool_frame))


def inverseTimes(transform1: Tuple[List[float], List[float]], transform2: Tuple[List[float], List[float]]) -> Tuple[
    List[float], List[float]]:
    """
    Like a Minus for Transforms, this subtracts the second transform from the first.
    """
    inv = p.invertTransform(transform2[0], transform2[1])
    return p.multiplyTransforms(transform1[0], transform1[1], inv[0], inv[1])


def transform(pose: List[float],
              transformation: List[float],
              local_coords=False):  # TODO: if pose is a list of position and orientation calculate new pose w/ orientation too
    input_has_rotation = len(pose) == 7
    transformation_has_rotation = len(transformation) == 7
    pose_tf = transform_from_pq(
        np.concatenate((pose[:3], quaternion_wxyz_from_xyzw(pose[3:])))) if input_has_rotation else transform_from(
        np.eye(3), pose)
    transformation_tf = transform_from_pq(np.concatenate((transformation[:3], quaternion_wxyz_from_xyzw(
        transformation[3:])))) if transformation_has_rotation else transform_from(np.eye(3), transformation)
    if local_coords:
        res = pose_tf @ transformation_tf
    else:
        res = transformation_tf @ pose_tf
    res = pq_from_transform(res)
    res[3:] = quaternion_xyzw_from_wxyz(res[3:])
    if not input_has_rotation:
        return res[:3].tolist()
    return res.tolist()


def _block(tree):
    """Wrap multiple statements into a single block and return it.

    If macros themselves are not a single statement, they can't always be nested (for example inside the par macro which executes each statement in an own thread).

    Arguments:
    tree -- the tree containing the statements.
    """
    with q as new_tree:
        # Wrapping the tree into an if block which itself is a statement that contains one or more statements.
        # The condition is just True and therefor makes sure that the wrapped statements get executed.
        if True:
            ast_literal[tree]

    return new_tree


class GeneratorList:
    """Implementation of generator list wrappers.

    Generator lists store the elements of a generator, so these can be fetched multiple times.

    Methods:
    get -- get the element at a specific index.
    has -- check if an element at a specific index exists.
    """

    def __init__(self, generator: Callable):
        """Create a new generator list.

        Arguments:
        generator -- the generator to use.
        """
        if isgeneratorfunction(generator):
            self._generator = generator()
        else:
            self._generator = generator

        self._generated = []

    def get(self, index: int = 0):
        """Get the element at a specific index or raise StopIteration if it doesn't exist.

        Arguments:
        index -- the index to get the element of.
        """
        while len(self._generated) <= index:
            self._generated.append(next(self._generator))

        return self._generated[index]

    def has(self, index: int) -> bool:
        """Check if an element at a specific index exists and return True or False.

        Arguments:
        index -- the index to check for.
        """
        try:
            self.get(index)
            return True
        except StopIteration:
            return False
