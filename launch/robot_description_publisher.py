#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import re
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String

def read_xml_or_file(s: str) -> str:
    """Return XML from a direct string or from a file path if it exists."""
    if os.path.exists(s) and os.path.isfile(s):
        with open(s, "r", encoding="utf-8") as f:
            return f.read()
    return s

def strip_xml_comments(xml: str) -> str:
    """Remove XML comments so CLI/param parsers don't see '--' from '<!--'."""
    return re.sub(r'<!--.*?-->', '', xml, flags=re.DOTALL)

class RobotDescriptionPublisher(Node):
    def __init__(self, argv):
        super().__init__('robot_description_publisher')

        parser = argparse.ArgumentParser(
            description='Publish URDF/Xacro XML to a latched topic '
                        '(defaults to /robot_description). '
                        'Provide either a raw XML string or a path to a URDF/Xacro file.'
        )
        parser.add_argument(
            '-xml_string',
            required=True,
            type=str,
            metavar='XML_OR_FILE',
            help='Raw XML string OR path to a URDF/Xacro file.'
        )
        parser.add_argument(
            '-robot_description_topic',
            type=str,
            default='/robot_description',
            metavar='TOPIC',
            help='Topic to publish the XML on (default: /robot_description)'
        )

        self.args = parser.parse_args(argv[1:])

        # Latched publisher (transient local) so late subscribers get the XML.
        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.pub = self.create_publisher(
            String, self.args.robot_description_topic, latched_qos
        )

        # Read, sanitize, and publish once
        xml = read_xml_or_file(self.args.xml_string)
        if not xml.strip().startswith('<'):
            self.get_logger().warn(
                "Provided -xml_string doesn't look like XML. "
                "If you meant to supply a file, double-check the path."
            )

        xml_clean = strip_xml_comments(xml)
        msg = String()
        msg.data = xml_clean
        self.pub.publish(msg)

        self.get_logger().info(
            f"Published robot_description to {self.args.robot_description_topic} "
            f"(length={len(xml_clean)})."
        )

def main(args=None):
    rclpy.init(args=args)
    # Remove ROS remapping args before handing to argparse
    argv = rclpy.utilities.remove_ros_args(args)
    node = RobotDescriptionPublisher(argv)
    # Spin so the latched message remains available
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# #!/usr/bin/python3
# # -*- coding: utf-8 -*-
# import rclpy
# import argparse
# # import the ROS2 python libraries
# from rclpy.node import Node
# from rclpy.qos import QoSProfile
# from rclpy.qos import QoSDurabilityPolicy
# from std_msgs.msg import String


# class RobotDescPub(Node):

#     def __init__(self, args):

#         super().__init__('robot_desciption_pub')
#         parser = argparse.ArgumentParser(
#             description='Publish into the given topic ( robot_description by default ) the XML string describing you robot')
#         parser.add_argument('-xml_string', required=True, type=str, metavar='XML_URDF_XACRO',
#                             help='Stringified xml data from urdf or xacro describing the robot')
#         parser.add_argument('-robot_description_topic', type=str, default='/robot_description', metavar='ROBOT_DESCRIPTION_TOPIC',
#                             help='Name of topic where the robot rescription from xml data will be published')

#         self.args = parser.parse_args(args[1:])

#         rclpy.logging.set_logger_level(
#             'robot_desciption_pub', rclpy.logging.LoggingSeverity.ERROR)

#         # Data recieved
#         # self.get_logger().info("XML ROBOT ==>"+self.args.xml_string)
#         # self.get_logger().info("Topic to pubish ==>"+self.args.robot_description_topic)

#         # create the publisher object
#         latched_qos = QoSProfile(
#             depth=1,
#             durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
#         self.publisher_ = self.create_publisher(
#             String, '/robot_description', latched_qos)

#         # Send Data
#         self.send(self.args.xml_string)

#         # self.get_logger().info("FINISHED Robot DESCRIPTION PUBLISH")

#     def send(self, xml_data):
#         # self.get_logger().info("Publishing XML DATA....")
#         self.cmd = String()
#         self.cmd.data = xml_data
#         self.publisher_.publish(self.cmd)
#         # self.get_logger().info("Publishing XML DATA.......DONE")


# def main(args=None):
#     # initialize the ROS communication
#     rclpy.init(args=args)
#     args_without_ros = rclpy.utilities.remove_ros_args(args)
#     # declare the node constructor
#     robot_desciption_pub = RobotDescPub(args_without_ros)

#     rclpy.spin(robot_desciption_pub)

#     robot_desciption_pub.destroy_node()

#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()