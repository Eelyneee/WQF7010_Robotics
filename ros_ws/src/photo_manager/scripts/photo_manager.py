#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import os
import datetime

class PhotoManager:
    def _init_(self):
        self.bridge = CvBridge()
        self.latest_image = None
        self.photo_dir = rospy.get_param("~photo_dir", "/tmp/smile_photos")

        # Create photo directory if it doesn't exist
        os.makedirs(self.photo_dir, exist_ok=True)

        # Subscribers
        rospy.Subscriber("/save_photo", Image, self.save_photo_callback)
        rospy.Subscriber("/view_photo_request", String, self.view_photo_callback)

        # Publisher for sending back image preview or path
        self.photo_preview_pub = rospy.Publisher("/photo_preview", String, queue_size=1)

        rospy.loginfo("Photo Manager Node Started.")

    def save_photo_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"photo_{timestamp}.jpg"
            file_path = os.path.join(self.photo_dir, filename)
            cv2.imwrite(file_path, cv_image)
            rospy.loginfo(f"Saved photo: {file_path}")
        except Exception as e:
            rospy.logerr(f"Failed to save photo: {e}")

    def view_photo_callback(self, msg):
        photo_name = msg.data
        file_path = os.path.join(self.photo_dir, photo_name)
        if os.path.exists(file_path):
            rospy.loginfo(f"Photo available at: {file_path}")
            self.photo_preview_pub.publish(f"Photo available: {file_path}")
        else:
            rospy.logwarn(f"Requested photo not found: {photo_name}")
            self.photo_preview_pub.publish(f"Photo not found: {photo_name}")

if _name_ == "_main_":
    rospy.init_node("photo_manager")
    PhotoManager()
    rospy.spin()
