"""Simplified from talos_apriltag_localizer/bench/shopcam_pub.py.

Removed: command-line options, the re-encode fallback for cameras without MJPG, logging and
frame-rate statistics. Illustrative; the full implementation is not public.

The localizer never opens a camera; it subscribes to an image topic. On the robot the Odin
camera's own driver publishes that topic. This script is the "any USB camera" alternative: it puts
a plain V4L2 webcam onto the same kind of topic, because the ROS distribution used here ships no
USB camera package.
"""

import cv2
import rclpy
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


def publish_usb_camera(device="/dev/video0", width=1280, height=720, fps=60,
                       topic="/shopcam/image/compressed"):
    rclpy.init()
    node = rclpy.create_node("shopcam_pub")

    # Ask the camera for MJPG and tell OpenCV NOT to decode it: each read() then returns the
    # camera's own JPEG bytes, which go onto the topic unchanged (no decode, no re-encode).
    camera = cv2.VideoCapture(device, cv2.CAP_V4L2)
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    camera.set(cv2.CAP_PROP_FPS, fps)
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # always hand over the freshest frame
    camera.set(cv2.CAP_PROP_CONVERT_RGB, 0)

    # Depth 1: a pose solver only ever wants the newest image, never a backlog.
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST)
    publisher = node.create_publisher(CompressedImage, topic, qos)

    message = CompressedImage()
    message.format = "jpeg"
    message.header.frame_id = "shopcam"
    while rclpy.ok():
        ok, jpeg = camera.read()
        # Stamp the moment read() returns (capture plus USB transfer), and never re-stamp it,
        # so any delay measured downstream against this stamp is honest.
        stamp = node.get_clock().now().to_msg()
        if not ok:
            continue
        message.header.stamp = stamp
        message.data = jpeg.reshape(-1).tobytes()
        publisher.publish(message)
