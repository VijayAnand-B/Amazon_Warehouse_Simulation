#!/usr/bin/env python

# Import modules
import rospy
import cv2
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from pyzbar.pyzbar import decode
from node_iot_ros_bridge_action_client import RosIotBridgeActionClient
import datetime


class Camera1(RosIotBridgeActionClient):
    # This Class used for Identifying the color of the packages with the help of 2D camera's ROS Image message.

    def __init__(self):
        self._action_client = RosIotBridgeActionClient()
        self.bridge = CvBridge()
        # Subscribe to the topic /eyrc/vb/camera_1/image_raw to receive Raw image message of 2D camera
        self.image_sub = rospy.Subscriber(
            "/eyrc/vb/camera_1/image_raw", Image, self.callback
        )

        self._process_fineshed = False

    # This function stores the identified color of the respective package in parameter server and sends to Action sever.

    def get_qr_data(self, arg_image, pkg):

        # Points for respective box to focus image for particular box in shelf
        packages = {
            "packagen00": [(124, 311), (238, 415)],
            "packagen01": [(308, 305), (412, 420)],
            "packagen02": [(470, 303), (603, 420)],
            "packagen10": [(117, 483), (243, 595)],
            "packagen11": [(307, 483), (413, 593)],
            "packagen12": [(474, 484), (602, 594)],
            "packagen20": [(116, 631), (244, 741)],
            "packagen21": [(304, 631), (413, 738)],
            "packagen22": [(475, 633), (598, 738)],
        }

        if pkg in packages:
            [(x1, y1), (x2, y2)] = packages["{}".format(pkg)]
            # Crop the image of the package and call decode to extract the QR data
            img = arg_image[y1:y2, x1:x2]

            qr_result = decode(img)
            # Asssign the Inventory sheet parameter and send to Action server for update the Inventory sheet
            if len(qr_result) > 0:
                color = qr_result[0].data
                # Assign only if the stored color value of that package is 'None'
                if rospy.get_param("packages/{}".format(pkg)) == "None":

                    def color_code(color):
                        return (
                            "R"
                            if color == "red"
                            else ("Y" if color == "yellow" else "G")
                        )

                    def item_type(color):
                        return (
                            "Medicine"
                            if color == "red"
                            else ("Food" if color == "yellow" else "Clothes")
                        )

                    def priority(color):
                        return (
                            "HP"
                            if color == "red"
                            else ("MP" if color == "yellow" else "LP")
                        )

                    storage_no = "R{} C{}".format(pkg[8], pkg[9])

                    def cost(color):
                        return (
                            "1000"
                            if color == "red"
                            else ("500" if color == "yellow" else "100")
                        )

                    pkg_details = {
                        "color": color,
                        "item": item_type(color),
                        "priority": priority(color),
                        "cost": cost(color),
                        "dispatch_status": "NO",
                        "shipped_status": "NO",
                    }
                    rospy.set_param("packages/{}".format(pkg), pkg_details)
                    date_object = datetime.date.today()
                    date = "{}".format(date_object)
                    year = date.split("-")[0][2:]
                    month = date.split("-")[1]
                    sku = "{}{}{}{}{}".format(
                        color_code(color), pkg[8], pkg[9], month, year
                    )
                    parameters = str(
                        {
                            "id": "Inventory",
                            "Team Id": "VB#1516",
                            "Unique Id": "aYzqLq",
                            "SKU": sku,
                            "Item": item_type(color),
                            "Priority": priority(color),
                            "Storage Number": storage_no,
                            "Cost": cost(color),
                            "Quantity": "1",
                        }
                    )

                    self._action_client.send_goal(parameters)
                    rospy.sleep(2)

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        # Defining package names
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

        for i in range(len(pkg_name)):
            self.get_qr_data(image, pkg_name[i])


def main():
    rospy.sleep(5)
    # Initialize the node
    rospy.init_node("node_camera", anonymous=True)

    ic = Camera1()

    # Run the node until a interrupt
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")


if __name__ == "__main__":
    main()
