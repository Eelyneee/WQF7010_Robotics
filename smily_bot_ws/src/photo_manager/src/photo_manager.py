#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Empty
from cv_bridge import CvBridge
import cv2
import os
import datetime

class PhotoManager:
    def __init__(self):
        self.bridge = CvBridge()
        self.latest_image = None
        # self.photo_dir = rospy.get_param("~photo_dir", "/tmp/smile_photos")
        self.photo_dir = "/home/mustar/smily_bot_ws/tmp/smile_photos"

        # Create photo directory if it doesn't exist
        os.makedirs(self.photo_dir, exist_ok=True)

        # Subscribers
        rospy.Subscriber("/save_photo", Image, self.save_photo_callback)
        rospy.Subscriber("/view_photo_request", Empty, self.view_photo_callback)

        # Publisher for sending back image preview or path
        self.photo_preview_pub = rospy.Publisher("/photo_preview", String, queue_size=1)
        self.photo_feedback_pub = rospy.Publisher("/photo_saved_feedback", String, queue_size=1)


        rospy.loginfo("Photo Manager Node Started.")
        # rospy.spin()

    def save_photo_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"photo_{timestamp}.jpg"
            file_path = os.path.join(self.photo_dir, filename)
            cv2.imwrite(file_path, cv_image)
            rospy.loginfo(f"Saved photo: {file_path}")
            feedback_msg = f"photo_saved:{file_path}"
            self.photo_feedback_pub.publish(feedback_msg)
        except Exception as e:
            rospy.logerr(f"Failed to save photo: {e}")
            self.photo_feedback_pub.publish("photo_save_failed")

    def view_photo_callback(self, msg):
        """
        Callback function for the /view_photo_request topic.
        This function is now modified to ALWAYS display the most recently saved photo,
        regardless of the content of the incoming message.
        """
        rospy.loginfo("Received view photo request. Displaying the latest photo.")
        file_path = None

        try:
            # List all files in the photo directory
            list_of_files = os.listdir(self.photo_dir)
            # Filter for .jpg files and get their full paths
            jpg_files = [os.path.join(self.photo_dir, f) for f in list_of_files if f.endswith('.jpg')]
            
            if jpg_files:
                # Sort files by modification time (most recent last)
                jpg_files.sort(key=os.path.getmtime)
                # Get the path of the latest photo
                file_path = jpg_files[-1]
                rospy.loginfo(f"Identified latest photo: {file_path}")
            else:
                rospy.logwarn("[PhotoManager] No .jpg photos found in directory.")
                self.photo_preview_pub.publish("No photos found.")
                return
        except Exception as e:
            rospy.logerr(f"Error finding latest photo: {e}")
            self.photo_preview_pub.publish("Error finding latest photo.")
            return

        # Proceed to display the photo if a file_path was determined
        if file_path and os.path.exists(file_path):
            rospy.loginfo(f"Displaying photo: {file_path}")
            self.photo_preview_pub.publish(f"Photo available: {file_path}")

            # Read and show the image using OpenCV
            image = cv2.imread(file_path)
            if image is not None:
                cv2.imshow("Smile Detector - Photo Viewer", image)
                 # cv2.waitKey(1) is essential to allow OpenCV to process events
                 # and display the window. A value of 1 means it waits for 1ms.
                cv2.waitKey(0) 
            else:
                rospy.logerr(f"[PhotoManager] Could not read image file: {file_path}")
                self.photo_preview_pub.publish(f"Could not read image: {file_path}")
        else:
            # This else block should ideally not be reached if file_path is None due to no photos
            # but it's kept for robustness in case of unexpected file system issues.
            rospy.logwarn(f"[PhotoManager] Latest photo not found or path invalid.")
            self.photo_preview_pub.publish(f"Latest photo not found.")

if __name__ == "__main__":
    rospy.init_node("photo_manager")
    PhotoManager()
    rospy.spin()