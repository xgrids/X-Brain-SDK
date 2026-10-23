#!/usr/bin/env python3
"""Publish sensor_msgs/Image alongside the camera CompressedImage topics.

Why this exists instead of image_transport republish: republish needs the
compressed_image_transport plugin, and on this robot only image_transport/raw_sub
is registered. Installing ros-humble-compressed-image-transport is currently
blocked because apt is in a broken state (an unrelated nginx dependency on
libssl1.1), so the plugin route is unavailable without touching system packages.

cv_bridge and cv2 are already present, so decoding here avoids that dependency
entirely.

The compressed topics stay untouched; this only adds an Image topic per camera
for RViz and other tools that do not accept CompressedImage.

Frame ids are copied from the source message rather than reassigned, so the
image keeps whatever frame the bridge stamped it with.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image


class JpegDecoder(Node):
    def __init__(self):
        super().__init__("lixel_jpeg_decoder")

        self.declare_parameter("cameras", ["left", "center", "right"])
        self.declare_parameter("in_pattern", "/lixel/camera/{}/compressed")
        self.declare_parameter("out_pattern", "/lixel/camera/{}/image_raw")

        cameras = self.get_parameter("cameras").value
        in_pat = self.get_parameter("in_pattern").value
        out_pat = self.get_parameter("out_pattern").value

        self.bridge = CvBridge()

        # The two ends need different reliability, so they get separate profiles.
        #
        # Input: the bridge publishes sensor data BEST_EFFORT. A RELIABLE
        # subscriber is incompatible with a BEST_EFFORT publisher, so this side
        # must be BEST_EFFORT or it would silently receive nothing.
        sub_qos = QoSProfile(depth=2,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)

        # Output: RViz's Image display subscribes RELIABLE by default, so a
        # BEST_EFFORT publisher here would be unreachable from RViz. Publishing
        # RELIABLE still serves BEST_EFFORT subscribers, since that direction of
        # mismatch is compatible.
        pub_qos = QoSProfile(depth=2,
                             reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)

        self.subs = []
        self.pubs = {}
        self.counts = {}
        for cam in cameras:
            out_topic = out_pat.format(cam)
            self.pubs[cam] = self.create_publisher(Image, out_topic, pub_qos)
            self.counts[cam] = 0
            self.subs.append(self.create_subscription(
                CompressedImage, in_pat.format(cam),
                self._make_cb(cam), sub_qos))
            self.get_logger().info(
                "decoding %s -> %s" % (in_pat.format(cam), out_topic))

        self.create_timer(10.0, self._report)

    def _make_cb(self, cam):
        def cb(msg):
            try:
                buf = np.frombuffer(msg.data, dtype=np.uint8)
                img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                if img is None:
                    # A truncated or non-JPEG payload; log rarely, do not spam.
                    self.get_logger().warn(
                        "%s: decode returned None (%d bytes)" % (cam, len(msg.data)),
                        throttle_duration_sec=5.0)
                    return
                out = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
                # Preserve the original stamp and frame so downstream TF lookups
                # and time sync behave the same as on the compressed topic.
                out.header = msg.header
                self.pubs[cam].publish(out)
                self.counts[cam] += 1
            except Exception as exc:
                self.get_logger().error(
                    "%s: %s" % (cam, exc), throttle_duration_sec=5.0)
        return cb

    def _report(self):
        self.get_logger().info(
            "decoded frames: " + ", ".join(
                "%s=%d" % (k, v) for k, v in self.counts.items()))


def main():
    rclpy.init()
    node = JpegDecoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
