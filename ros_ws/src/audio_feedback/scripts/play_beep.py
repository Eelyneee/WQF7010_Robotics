#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge # For converting ROS Image messages to OpenCV images
import os
import pygame.mixer

# Initialize CvBridge
bridge = CvBridge()

# Path to your beep audio file
# Make sure 'beep.mp3' or 'beep.wav' is in the same directory as this script
AUDIO_FILE_BEEP = os.path.join(os.path.dirname(__file__), "..", "audio", "beep-25.mp3") # Change to .wav if needed

def save_photo_and_beep_callback(data):
    """
    Callback function for the /save_photo topic.
    Receives an Image message, saves it, and plays a beep sound.
    """
    rospy.loginfo("Received /save_photo command. Attempting to save photo and play beep...")
    
    # 1. Save the photo
    try:
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        
        # Define filename with timestamp
        photo_filename = os.path.join(os.getcwd(), f"smile_photo_{rospy.Time.now().secs}.png")
        
        # Save the image
        cv2.imwrite(photo_filename, cv_image)
        rospy.loginfo(f"Photo saved: {photo_filename}")
        
    except Exception as e:
        rospy.logerr(f"Error processing or saving photo from /save_photo: {e}")

    # 2. Play beep sound
    try:
        # Ensure pygame mixer is initialized
        if not pygame.mixer.get_init():
            pygame.mixer.init()
            pygame.mixer.music.set_volume(0.7) # Optional: Set volume
        
        pygame.mixer.music.load(AUDIO_FILE_BEEP) # Load the beep sound file
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            rospy.sleep(0.1) # Wait for the sound to finish playing
    except pygame.error as e:
        rospy.logerr(f"Error playing beep sound: {e}. Make sure '{AUDIO_FILE_BEEP}' exists and pygame is installed correctly.")
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred during beep sound playback: {e}")


def play_beep_node():
    """
    Initializes the play_beep ROS node, initializes pygame mixer,
    and subscribes only to /save_photo.
    """
    rospy.init_node('play_beep', anonymous=True)

    # Initialize pygame mixer once when the node starts
    try:
        pygame.mixer.init()
        pygame.mixer.music.set_volume(0.7) 
        rospy.loginfo("Pygame mixer initialized for play_beep.")
    except Exception as e:
        rospy.logerr(f"Failed to initialize Pygame mixer for play_beep: {e}")
        rospy.signal_shutdown("Could not initialize sound system.")
        return

    # Subscribe only to /save_photo for saving the image and playing the beep
    rospy.Subscriber("/save_photo", Image, save_photo_and_beep_callback) 
    
    rospy.spin()

if __name__ == '__main__':
    try:
        play_beep_node()
    except rospy.ROSInterruptException:
        # Clean up pygame mixer when node shuts down
        if pygame.mixer.get_init():
            pygame.mixer.quit()
        pass