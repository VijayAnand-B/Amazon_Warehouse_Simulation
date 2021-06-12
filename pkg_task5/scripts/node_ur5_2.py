#! /usr/bin/env python


import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

import sys

import math
from node_iot_ros_bridge_action_client import RosIotBridgeActionClient


from std_srvs.srv import Empty
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.msg import Model
from pkg_vb_sim.msg import LogicalCameraImage
from pkg_vb_sim.srv import conveyorBeltPowerMsg
import rospkg
import datetime as dt
from threading import Thread


class Ur5Moveit(RosIotBridgeActionClient):

    # Constructor
    def __init__(self, arg_robot_name):

        rospy.init_node("node_UR5_2", anonymous=True)

        self._robot_ns = "/" + arg_robot_name
        self._planning_group = "manipulator"

        self._commander = moveit_commander.roscpp_initialize(sys.argv)

        self._robot = moveit_commander.RobotCommander(
            robot_description=self._robot_ns + "/robot_description",
            ns=self._robot_ns,
        )
        self._scene = moveit_commander.PlanningSceneInterface(
            ns=self._robot_ns
        )
        self._group = moveit_commander.MoveGroupCommander(
            self._planning_group,
            robot_description=self._robot_ns + "/robot_description",
            ns=self._robot_ns,
        )
        self._display_trajectory_publisher = rospy.Publisher(
            self._robot_ns + "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=1,
        )
        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            self._robot_ns + "/execute_trajectory",
            moveit_msgs.msg.ExecuteTrajectoryAction,
        )
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        self._action_client = RosIotBridgeActionClient()

        self._box_name = ""
        rospy.set_param(
            "/move_group/trajectory_execution/allowed_start_tolerance", 0.0
        )
        # Attribute to store computed trajectory by the planner
        self._computed_plan = ""

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        self._subscriber = rospy.Subscriber(
            "/eyrc/vb/logical_camera_2",
            LogicalCameraImage,
            self.func_callback_topic_my_topic,
            queue_size=1,
        )

        rospy.loginfo(
            "\033[94m"
            + "Planning Group: {}".format(self._planning_frame)
            + "\033[0m"
        )
        rospy.loginfo(
            "\033[94m"
            + "End Effector Link: {}".format(self._eef_link)
            + "\033[0m"
        )
        rospy.loginfo(
            "\033[94m"
            + "Group Names: {}".format(self._group_names)
            + "\033[0m"
        )

        self._pkg_shipped = []

        self._pkg_to_be_shipped = []

        self._thread_1 = Thread(target=self.ship_package)

        self._thread_1.start()

        self._home_pose_joint_angles = [
            math.radians(172.155279696),
            math.radians(-40.0705518742),
            math.radians(58.3085807251),
            math.radians(-108.231141355),
            math.radians(-90.057998765),
            math.radians(-7.89215231153),
        ]

        self.hard_set_joint_angles(self._home_pose_joint_angles, 5)

    def set_joint_angles(self, arg_list_joint_angles):

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        if flag_plan == True:
            pass
            # rospy.loginfo(
            #     '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            pass
            # rospy.logerr(
            #     '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):

        number_attempts = 0
        flag_success = False

        while (number_attempts <= arg_max_attempts) and (
            flag_success is False
        ):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts))

    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo("\033[94m" + ">>> Current Pose:" + "\033[0m")
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo("\033[94m" + ">>> Final Pose:" + "\033[0m")
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo("\033[94m" + ">>> Final Joint Values:" + "\033[0m")
        rospy.loginfo(list_joint_values)

        if flag_plan == True:
            rospy.loginfo("\033[94m" + ">>> go_to_pose() Success" + "\033[0m")
        else:
            rospy.logerr(
                "\033[94m"
                + ">>> go_to_pose() Failed. Solution for Pose not Found."
                + "\033[0m"
            )

        return flag_plan

    def ship_package(self):

        while 1:
            if len(self._pkg_to_be_shipped) > 0:
                pkg, box_x, box_y, box_z = (
                    self._pkg_to_be_shipped[0][0],
                    self._pkg_to_be_shipped[0][1],
                    self._pkg_to_be_shipped[0][2],
                    self._pkg_to_be_shipped[0][3],
                )

                # Moving ur5 to Red-Box
                ur5_box_pose = geometry_msgs.msg.Pose()
                ur5_box_pose.position.x = box_x
                ur5_box_pose.position.y = box_y
                ur5_box_pose.position.z = box_z
                ur5_box_pose.orientation.x = -0.5
                ur5_box_pose.orientation.y = -0.5
                ur5_box_pose.orientation.z = 0.5
                ur5_box_pose.orientation.w = 0.5
                self.go_to_pose(ur5_box_pose)
                self.vacuum(
                    True
                )  # Activating vacuum_gripper by sending "True"

                # # Joint angles to place boxes in respective bin
                yellow_bin_pose = [
                    -0.1771709027193411,
                    -0.9921280123753032,
                    1.0548623752397819,
                    -1.70019819009636,
                    -1.252214644178183,
                    0.0000000000000000,
                ]
                #
                red_bin_pose = [
                    1.2565644968956726,
                    -1.239141380724108,
                    1.5358854305658147,
                    -1.867432925778798,
                    -1.5708627880994253,
                    0.0000000000000000,
                ]
                #
                green_bin_pose = [
                    1.7243444681980915,
                    -2.196759909867357,
                    -1.2637365950432176,
                    -1.27554311119922,
                    1.5115639324751566,
                    -0.13684565887823025,
                ]

                pkg_lift_joint_angle = [
                    math.radians(172.152833189),
                    math.radians(-42.4931496255),
                    math.radians(53.8270741059),
                    math.radians(-101.306465154),
                    math.radians(-89.9618821249),
                    math.radians(-7.84856422844),
                ]
                # Lift the package above conveyor
                self.hard_set_joint_angles(pkg_lift_joint_angle, 5)

                # Get the value from parameter server file (packages.yaml)
                pkg_details_dic = rospy.get_param("packages/{}".format(pkg))
                print(pkg_details_dic)
                color = pkg_details_dic["color"]
                print(color)

                # If respective color are detected
                if (
                    pkg_details_dic["dispatch_status"] == "YES"
                    and pkg_details_dic["shipped_status"] == "NO"
                ):

                    self.conveyor(100)
                    self.hard_set_joint_angles(
                        locals()["{}_bin_pose".format(color)], 5
                    )

                    self.vacuum(False)

                    time_now = dt.datetime.now()

                    shipping_time = "{}".format(time_now)[:19]

                    delivery_time = (
                        lambda time_now, color: time_now + dt.timedelta(days=1)
                        if color == "red"
                        else (
                            time_now + dt.timedelta(days=3)
                            if color == "yellow"
                            else time_now + dt.timedelta(days=5)
                        )
                    )

                    pkg_details_dic["shipped_status"] = "YES"

                    rospy.set_param("packages/{}".format(pkg), pkg_details_dic)

                    parameters = {
                        "id": "OrdersShipped",
                        "Team Id": "VB#1516",
                        "Unique Id": "aYzqLq",
                        "Order Id": pkg_details_dic["order_id"],
                        "City": pkg_details_dic["city"],
                        "Item": pkg_details_dic["item"],
                        "Priority": pkg_details_dic["priority"],
                        "Cost": pkg_details_dic["cost"],
                        "Shipped Quantity": "1",
                        "Shipped Status": "YES",
                        "Shipped Date and Time": shipping_time,
                        "Order Date and Time": pkg_details_dic["order_time"],
                        "Estimated Time of Delivery": "{}".format(
                            delivery_time(time_now, color)
                        )[:11],
                    }

                    self._action_client.send_goal(parameters)

                    self.hard_set_joint_angles(self._home_pose_joint_angles, 5)

                    self._pkg_to_be_shipped.pop(0)

    def func_callback_topic_my_topic(self, msg):
        # print(rospy.get_param('packages'))
        try:

            model = msg.models[-1]  # last pkg in list is assigned to 'model'
            pkg = str(model.type)  # type of the detected pkg

            # Extracting pose values detected from logical_camera_2
            x = model.pose.position.x
            y = model.pose.position.y
            z = model.pose.position.z

            box_length = 0.15  # Length of the Package
            vacuum_grip_width = 0.115  # Vacuum Gripper

            # Transforming the subscribed pose values for ur5 to pick the detected box
            box_x = z - 0.800003692
            box_y = y
            box_z = (1.999999799 - x) + vacuum_grip_width + (box_length / 2)

            if (pkg != "ur5") and (
                pkg not in self._pkg_shipped
            ):  # If "ur5" arm is not detected
                # If camera_y value of detected pkg is in this range

                if (y <= 0.166258826852) and (y <= 0.000000001):
                    self.conveyor(0)

                    # Update the detected package values in the lists
                    self._pkg_to_be_shipped.append([pkg, box_x, box_y, box_z])

                    self._pkg_shipped.append(pkg)

        except IndexError:
            pass

    def conveyor(self, pwr):
        rospy.wait_for_service("/eyrc/vb/conveyor/set_power")
        conveyor_power = rospy.ServiceProxy(
            "/eyrc/vb/conveyor/set_power", conveyorBeltPowerMsg
        )
        conveyor_power(pwr)

    def vacuum(self, Boolean):
        if rospy.get_param("vacuum_gripper_plugin_in_usage") == True:
            rospy.sleep(0.05)
        rospy.wait_for_service("/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2")
        vacuum_bool = rospy.ServiceProxy(
            "/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2", vacuumGripper
        )
        vacuum_bool(Boolean)

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            "\033[94m" + "Object of class Ur5Moveit Deleted." + "\033[0m"
        )


def main():
    rospy.sleep(5)
    ur5 = Ur5Moveit("ur5_2")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
