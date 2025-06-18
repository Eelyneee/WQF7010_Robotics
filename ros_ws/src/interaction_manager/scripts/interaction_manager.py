#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Bool
from threading import Timer

class InteractionManager:
    def __init__(self):
        rospy.init_node('interaction_manager')

        # Publishers
        self.start_pub = rospy.Publisher('/start_detection', Bool, queue_size=1)
        self.retry_pub = rospy.Publisher('/retry_signal', Bool, queue_size=1)
        self.tts_pub = rospy.Publisher('/say_text', String, queue_size=1)
        self.view_photo_pub = rospy.Publisher('/view_photo_request', Bool, queue_size=1)

        # Subscriber
        rospy.Subscriber('/voice_command', String, self.command_callback)

        # Internal state
        self.retry_count = 0
        self.max_retries = 3
        self.timeout_timer = None

        rospy.loginfo("Interaction Manager is ready.")
        rospy.spin()

    def command_callback(self, msg):
        command = msg.data.lower()
        rospy.loginfo(f"Received command: {command}")

        if not command.startswith("hey bot"):
            rospy.loginfo("Wake word not detected. Ignoring command.")
            return

        self.tts_pub.publish("Command received")

        if "take photo" in command or "check smile" in command:
            self.initiate_smile_check()
        elif "try again" in command:
            self.retry_smile_check()
        elif "view photo" in command:
            self.view_photo_pub.publish(True)
        elif "say that again" in command:
            self.tts_pub.publish("Say that again?")
        else:
            self.tts_pub.publish("Sorry, I didn't understand that.")

    def initiate_smile_check(self):
        self.retry_count = 0
        self.countdown_and_start()

    def retry_smile_check(self):
        if self.retry_count < self.max_retries:
            self.retry_count += 1
            self.tts_pub.publish("Let's try again.")
            self.countdown_and_start()
        else:
            self.tts_pub.publish("Too many failed attempts. Please reset.")

    def countdown_and_start(self):
        self.tts_pub.publish("3")
        rospy.sleep(1)
        self.tts_pub.publish("2")
        rospy.sleep(1)
        self.tts_pub.publish("1")
        rospy.sleep(1)

        self.start_pub.publish(True)

        # Start timeout
        if self.timeout_timer:
            self.timeout_timer.cancel()
        self.timeout_timer = Timer(15.0, self.timeout_handler)
        self.timeout_timer.start()

    def timeout_handler(self):
        rospy.loginfo("Smile detection timeout reached.")
        self.tts_pub.publish("Timeout reached. No smile detected.")
        self.retry_smile_check()

if __name__ == '__main__':
    try:
        InteractionManager()
    except rospy.ROSInterruptException:
        pass
