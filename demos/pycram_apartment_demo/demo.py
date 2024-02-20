from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.pose import Pose
from pycram.bullet_world import BulletWorld, Object
from pycram.process_module import simulated_robot, with_simulated_robot, real_robot
from pycram.enums import ObjectType
import giskard_msgs
from giskardpy.python_interface import GiskardWrapper
from pycram.ros.robot_state_updater import RobotStateUpdater

world = BulletWorld()
robot = Object("pr2", ObjectType.ROBOT, "pr2.urdf", pose=Pose([1, 2, 0]))
apartment = Object("apartment", ObjectType.ENVIRONMENT, "apartment.urdf")

cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl", pose=Pose([0.3, 3.3, 1.5], [0, 0, 1, 0]), color=[0, 1, 0, 1])
spoon = Object("spoon", ObjectType.SPOON, "spoon.stl", pose=Pose([2.4, 2.2, 0.85]), color=[0, 0, 1, 1])
bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=Pose([2.5, 2.2, 1.02]), color=[1, 1, 0, 1])
apartment.attach(spoon, 'cabinet10_drawer_top')

pick_pose = Pose([2.7, 2.15, 1])

robot_desig = BelieveObject(names=["pr2"])
apartment_desig = BelieveObject(names=["apartment"])
r = RobotStateUpdater("/tf", "/joint_states")



with real_robot:
    ParkArmsAction([Arms.BOTH]).resolve().perform()

    # MoveTorsoAction([0.33]).resolve().perform()
    #
    # #Finding and navigating to the drawer holding the spoon
    # handle_desig = ObjectPart(names=["handle_cab10_t"], part_of=apartment_desig.resolve())
    # drawer_open_loc = AccessingLocation(handle_desig=handle_desig.resolve(), robot_desig=robot_desig.resolve()).resolve()
    #
    # NavigateAction([drawer_open_loc.pose]).resolve().perform()
    #
    # print(apartment.joints, sep="\n")
    # apartment.set_joint_state("cabinet10_drawer_top_joint", 0.4)
    #
    # spoon.detach(apartment)
    #
    # # Detect and pickup the spoon
    # LookAtAction([apartment.get_link_pose("handle_cab10_t")]).resolve().perform()
    #
    # spoon_desig = DetectAction(BelieveObject(types=[ObjectType.SPOON])).resolve().perform()
    #
    # pickup_arm = "left" if drawer_open_loc.arms[0] == "right" else "right"
    # PickUpAction(spoon_desig, [pickup_arm], ["top"]).resolve().perform()
    #
    # ParkArmsAction([Arms.BOTH]).resolve().perform()
    #
    # CloseAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()
    #
    # ParkArmsAction([Arms.BOTH]).resolve().perform()
    #
    # #Find a pose to place the spoon, move and then place it
    # spoon_target_pose = Pose([2.36, 2, 1.02])
    # placing_loc = CostmapLocation(target=spoon_target_pose, reachable_for=robot_desig.resolve(), reachable_arm=pickup_arm).resolve()
    #
    # NavigateAction([placing_loc.pose]).resolve().perform()
    #
    # PlaceAction(spoon_desig, [spoon_target_pose], [pickup_arm]).resolve().perform()
    #
    # ParkArmsAction([Arms.BOTH]).resolve().perform()
    #
    # apartment.set_joint_state("cabinet4_door_top_top_joint", 2)
    #
    # NavigateAction([Pose([1.3, 3.3, 0], [0, 0, 1, 0])]).resolve().perform()
    #
    # # Detect and pickup the cereal
    # LookAtAction([apartment.get_link_pose("cabinet4")]).resolve().perform()
    #
    # cereal_desig = DetectAction(BelieveObject(types=[ObjectType.BREAKFAST_CEREAL])).resolve().perform()
    #
    # NavigateAction([Pose([1, 3.3, 0], [0, 0, 1, 0])]).resolve().perform()
    #
    # PickUpAction(cereal_desig, ["left"], ["front"]).resolve().perform()
    #
    # ParkArmsAction([Arms.BOTH]).resolve().perform()
    #
    # cereal_target_pose = Pose([2.36, 2.4, 1.02])
    #
    # placing_loc = CostmapLocation(target=cereal_target_pose, reachable_for=robot_desig.resolve(),
    #                               reachable_arm="left").resolve()
    #
    # NavigateAction([placing_loc.pose]).resolve().perform()
    #
    # PlaceAction(cereal_desig, [cereal_target_pose], ["left"]).resolve().perform()
    #
    # ParkArmsAction([Arms.BOTH]).resolve().perform()


