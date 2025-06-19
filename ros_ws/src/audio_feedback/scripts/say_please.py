#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
import pygame.mixer
import os

# Path to your audio file
# Make sure 'please_smile.mp3' is in the same directory as this script
AUDIO_FILE_PLEASE_SMILE = os.path.join(os.path.dirname(__file__), "..", "audio", "please_smile.mp3")

def smile_detected_callback(data):
    """
    Callback function for the /smile_detected topic.
    If no smile is detected, it plays "Please smile!" audio.
    """
    if not data.data:  # If smile_detected is False (no smile detected)
        rospy.loginfo("No smile detected. Playing 'Please smile!' audio.")
        try:
            # Ensure pygame mixer is initialized
            if not pygame.mixer.get_init():
                pygame.mixer.init()
            
            pygame.mixer.music.load(AUDIO_FILE_PLEASE_SMILE)
            pygame.mixer.music.play()
            
            # Wait for the sound to finish playing (optional, can be removed if you don't want to block)
            while pygame.mixer.music.get_busy():
                rospy.sleep(0.1) 
                
        except pygame.error as e:
            rospy.logerr(f"Error playing 'please_smile' sound: {e}. "
                         f"Make sure '{AUDIO_FILE_PLEASE_SMILE}' exists and pygame is installed correctly.")
        except Exception as e:
            rospy.logerr(f"An unexpected error occurred during 'please_smile' sound playback: {e}")
            
    else:
        rospy.loginfo("Smile detected! (from say_please.py)")

def say_please_node():
    """
    Initializes the say_please ROS node and subscribes to /smile_detected.
    Also initializes pygame mixer.
    """
    rospy.init_node('say_please', anonymous=True)
    
    # Initialize pygame mixer once when the node starts
    try:
        pygame.mixer.init()
        pygame.mixer.music.set_volume(0.8) # Adjust volume as needed (0.0 to 1.0)
        rospy.loginfo("Pygame mixer initialized for say_please.")
    except Exception as e:
        rospy.logerr(f"Failed to initialize Pygame mixer for say_please: {e}")
        rospy.signal_shutdown("Could not initialize sound system.")
        return

    rospy.Subscriber("/smile_detected", Bool, smile_detected_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        say_please_node()
    except rospy.ROSInterruptException:
        # Clean up pygame mixer when node shuts down
        if pygame.mixer.get_init():
            pygame.mixer.quit()
        pass