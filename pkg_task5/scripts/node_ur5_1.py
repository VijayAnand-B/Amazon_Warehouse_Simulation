#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import String

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg

import yaml
import os
import math
import time

from ast import literal_eval
from threading import Thread
import datetime as dt


from std_srvs.srv import Empty
from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_ros_iot_bridge.msg import msgMqttSub
from node_iot_ros_bridge_action_client import RosIotBridgeActionClient


class Ur5Moveit(RosIotBridgeActionClient):

    # Constructor
    def __init__(self, arg_robot_name):

        rospy.init_node("node_UR5_1", anonymous=True)

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

        self._action_client = RosIotBridgeActionClient()

        rospy.Subscriber(
            "/ros_iot_bridge/mqtt/sub", msgMqttSub, self.msg_callback
        )

        self._orders = []

        rospy.set_param(
            "/move_group/trajectory_execution/allowed_start_tolerance", 0.0
        )
        self._group.set_planning_time(20)

        self._thread_1 = Thread(target=self.dispatch)

        self._thread_1.start()

        self.conveyor(100)

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ""

        # Attribute to store computed trajectory by the planner
        self._computed_plan = ""

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

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

        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path("pkg_task5")
        self._file_path = self._pkg_path + "/config/saved_trajectories/"
        rospy.loginfo("Package Path: {}".format(self._file_path))

        rospy.loginfo("\033[94m" + " >>> Ur5Moveit init done." + "\033[0m")

        # Initial pose
        self.moveit_hard_play_planned_path_from_file(
            self._file_path, "zero_to_straightup.yaml", 5
        )

    def clear_octomap(self):
        clear_octomap_service_proxy = rospy.ServiceProxy(
            self._robot_ns + "/clear_octomap", Empty
        )
        return clear_octomap_service_proxy()

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._computed_plan = self._group.plan()
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
            # self.clear_octomap()

    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        file_path = arg_file_path + arg_file_name

        with open(file_path, "r") as file_open:
            loaded_plan = yaml.load(file_open)

        ret = self._group.execute(loaded_plan)
        # rospy.logerr(ret)
        return ret

    def moveit_hard_play_planned_path_from_file(
        self, arg_file_path, arg_file_name, arg_max_attempts
    ):
        number_attempts = 0
        flag_success = False

        while (number_attempts <= arg_max_attempts) and (
            flag_success is False
        ):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(
                arg_file_path, arg_file_name
            )
            rospy.logwarn("attempts: {}".format(number_attempts))
            # # self.clear_octomap()

        return True

    def msg_callback(self, mymsg):
        msg = mymsg.message.decode("utf-8")
        msg = literal_eval(msg)
        self._orders.append(msg)
        self._orders = sorted(
            self._orders, key=lambda k: k["item"], reverse=True
        )

        print(self._orders)

    def dispatch(self):
        StraightUp_joint_angles = [
            math.radians(0.0129602093866),
            math.radians(-89.9556396933),
            math.radians(-0.0048708470431),
            math.radians(-0.00448360702218),
            math.radians(0.000637115345487),
            math.radians(-0.00330698420447),
        ]

        packagen20_joint_angles_2 = [
            math.radians(-36.0740318544),
            math.radians(-94.3196562099),
            math.radians(118.328690863),
            math.radians(113.009134505),
            math.radians(-121.806937046),
            math.radians(-4.0045469314),
        ]
        packagen20_joint_angles_3 = [
            math.radians(-0.146887449887),
            math.radians(-164.00768557),
            math.radians(-7.79620993688),
            math.radians(78.4250396066),
            math.radians(-91.3470449193),
            math.radians(-4.16340775465),
        ]
        packagen20_joint_angles_0 = [
            math.radians(-48.9749811074),
            math.radians(-89.2383593813),
            math.radians(112.564691368),
            math.radians(152.213905404),
            math.radians(-132.468114191),
            math.radians(-2.27296792402),
        ]
        packagen20_joint_angles_1 = [
            math.radians(-36.0683789146),
            math.radians(-94.3137067874),
            math.radians(118.332223161),
            math.radians(150.188987622),
            math.radians(-145.328631928),
            math.radians(-4.00259128518),
        ]

        packagen21_joint_angles_0 = [
            math.radians(-118.236159367),
            math.radians(-117.972196819),
            math.radians(104.204424737),
            math.radians(16.3954931942),
            math.radians(63.0968328604),
            math.radians(11.1839953252),
        ]
        packagen21_joint_angles_2 = [
            math.radians(-158.566437817),
            math.radians(-136.896457095),
            math.radians(109.286615733),
            math.radians(33.6677431425),
            math.radians(112.32821072),
            math.radians(6.79546753313),
        ]
        packagen21_joint_angles_3 = [
            math.radians(-179.934707173),
            math.radians(-23.7185619687),
            math.radians(22.8052423995),
            math.radians(87.5348769927),
            math.radians(89.3276460529),
            math.radians(6.85712965621),
        ]
        packagen21_joint_angles_1 = [
            math.radians(-158.573918803),
            math.radians(-136.895106217),
            math.radians(109.291233649),
            math.radians(33.6665209912),
            math.radians(22.8501533788),
            math.radians(6.7921248999),
        ]

        # pkg 00
        packagen00_pick = [
            math.radians(-58.562321565),
            math.radians(-70.1389440809),
            math.radians(6.36033962574),
            math.radians(-114.36701873),
            math.radians(-121.801318427),
            math.radians(-0.0646365608858),
        ]
        # pkg 01
        packagen01_pick = [
            math.radians(116.797725009),
            math.radians(-98.0220396421),
            math.radians(-10.1943285575),
            math.radians(-72.5157068668),
            math.radians(57.5211307714),
            math.radians(0.0409565866476),
        ]
        # pkg 02
        packagen02_pick = [
            math.radians(56.322894151),
            math.radians(-115.63999967),
            math.radians(5.4171743229),
            math.radians(-68.1621520318),
            math.radians(123.15449806),
            math.radians(0.533188835605),
        ]
        # pkg 10
        packagen10_pick = [
            math.radians(-57.5293649471),
            math.radians(-83.5966356046),
            math.radians(39.1148334825),
            math.radians(43.9901612335),
            math.radians(122.487508503),
            math.radians(0.00825628561226),
        ]
        # pkg 11
        packagen11_pick = [
            math.radians(121.804586801),
            math.radians(-63.6719306564),
            math.radians(-97.0756036068),
            math.radians(-19.2309264756),
            math.radians(58.2041686289),
            math.radians(-6.09323736028),
        ]
        # pkg 12
        packagen12_pick = [
            math.radians(56.2749085877),
            math.radians(-82.5240659092),
            math.radians(-85.8732663038),
            math.radians(-12.7630133871),
            math.radians(121.761027007),
            math.radians(-1.46786582814),
        ]

        # pkg 22
        packagen22_pick = [
            math.radians(60.1674829453),
            math.radians(-81.858087871),
            math.radians(-119.891453952),
            math.radians(20.4983646484),
            math.radians(120.476711734),
            math.radians(0.0423775957511),
        ]

        pkg_name = [
            "packagen00",
            "packagen01",
            "packagen02",
            "packagen10",
            "packagen11",
            "packagen12",
            "packagen20",
            "packagen21",
            "packagen22",
        ]

        while True:
            if len(self._orders) > 0:

                order_dic = self._orders[0]
                print(order_dic)

                for i in range(len(pkg_name)):
                    pkg_details_dic = rospy.get_param(
                        "packages/{}".format(pkg_name[i])
                    )
                    if (
                        pkg_details_dic["item"] == order_dic["item"]
                        and pkg_details_dic["dispatch_status"] == "NO"
                    ):
                        if (
                            pkg_name[i] == "packagen20"
                            or pkg_name[i] == "packagen21"
                        ):
                            self.hard_set_joint_angles(
                                locals()[
                                    "{}_joint_angles_0".format(pkg_name[i])
                                ],
                                50,
                            )
                            self.vacuum(True)
                            self.hard_set_joint_angles(
                                locals()[
                                    "{}_joint_angles_1".format(pkg_name[i])
                                ],
                                50,
                            )
                            self.hard_set_joint_angles(
                                locals()[
                                    "{}_joint_angles_2".format(pkg_name[i])
                                ],
                                50,
                            )
                            self.hard_set_joint_angles(
                                locals()[
                                    "{}_joint_angles_3".format(pkg_name[i])
                                ],
                                50,
                            )
                            self.vacuum(False)

                        else:
                            # if pkg color is not 'None',then pick it

                            self.hard_set_joint_angles(
                                locals()["{}_pick".format(pkg_name[i])], 50
                            )

                            self.vacuum(True)

                            self.moveit_hard_play_planned_path_from_file(
                                self._file_path,
                                "{}_place.yaml".format(pkg_name[i]),
                                5,
                            )
                            self.vacuum(False)
                            time_now = dt.datetime.now()

                        pkg_details_dic["dispatch_status"] = "YES"
                        pkg_details_dic["order_id"] = order_dic["order_id"]
                        pkg_details_dic["city"] = order_dic["city"]
                        pkg_details_dic["order_time"] = order_dic["order_time"]

                        rospy.set_param(
                            "packages/{}".format(pkg_name[i]), pkg_details_dic
                        )

                        parameters = {
                            "id": "OrdersDispatched",
                            "Team Id": "VB#1516",
                            "Unique Id": "aYzqLq",
                            "Order Id": order_dic["order_id"],
                            "City": order_dic["city"],
                            "Item": pkg_details_dic["item"],
                            "Priority": pkg_details_dic["priority"],
                            "Cost": pkg_details_dic["cost"],
                            "Dispatch Quantity": "1",
                            "Dispatch Status": "YES",
                            "Dispatch Date and Time": "{}".format(time_now)[
                                :19
                            ],
                        }

                        self._action_client.send_goal(parameters)

                        pkg_name.pop(i)

                        for i in self._orders:
                            if i["order_id"] == order_dic["order_id"]:
                                self._orders.remove(i)
                                break

                        self.hard_set_joint_angles(StraightUp_joint_angles, 50)
                        break

    def vacuum(self, Boolean):
        rospy.wait_for_service("/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1")
        vacuum_bool = rospy.ServiceProxy(
            "/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1", vacuumGripper
        )
        vacuum_bool(Boolean)

    def conveyor(self, pwr):
        rospy.wait_for_service("/eyrc/vb/conveyor/set_power")
        conveyor_power = rospy.ServiceProxy(
            "/eyrc/vb/conveyor/set_power", conveyorBeltPowerMsg
        )
        conveyor_power(pwr)

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            "\033[94m" + "Object of class Ur5Moveit Deleted." + "\033[0m"
        )


def main():
    rospy.sleep(5)
    ur5 = Ur5Moveit("ur5_1")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")


if __name__ == "__main__":
    main()
