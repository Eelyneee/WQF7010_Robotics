#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Empty, Bool
import random
import time

class InteractionManager:
    def __init__(self):
        rospy.init_node('interaction_manager', anonymous=True)

        self.say_text_publisher = rospy.Publisher('/say_text', String, queue_size=10)
        self.conversation_state_publisher = rospy.Publisher('/milo_conversation_state', String, queue_size=1, latch=True)
        self.start_detection_publisher = rospy.Publisher('/start_detection', String, queue_size=1)
        self.view_photo_publisher = rospy.Publisher('/view_photo_request', Empty, queue_size=1)
        self.command_subscriber = rospy.Subscriber('/voice_command', String, self.command_callback)

        rospy.Subscriber('/tts_status', String, self.tts_status_callback)
        rospy.Subscriber('/camera_ready', String, self.camera_ready_callback)
        rospy.Subscriber('/image_view_ready',String,self.image_view_ready_callback)
        rospy.Subscriber("/photo_saved_feedback", String, self.photo_saved_callback)
        rospy.Subscriber("/face_detection_started", Empty, self.face_detection_started_callback)
        rospy.Subscriber('/smile_detected', Bool, self.smile_detected_callback)

        self.smile_classification_start_time = None
        self.SMILE_CLASSIFICATION_PHASE_TIMEOUT = 15 

        self.camera_ready = False
        self.image_view_ready = False   
        self.detection_active = False 
        self.countdown_started = False 

        self.last_spoken_phrase = ""
        self.last_activity_time = time.time()
        self.conversation_active_by_manager = False

        # Timeout settings
        self.timeout_duration = 10 
        self.long_timeout_duration = 30
        self.last_prompt_time = 0

        self.tts_busy = False 

        # Define all possible responses for the robot
        self.response_phrases = {
            "robot_greet": [
                "Hi, I'm Milo. I am your personal photograph assistant. What would you like me to do?",
                "I'm here and ready to help you capture great moments!"
            ],
            "processing_command": [
                "Hmm...",
                "One moment..."
            ],
            "take_photo_initial": [
                "Booting up camera, please wait.",
                "Starting the camera system."
            ],
            "countdown_smile": [ 
                "3, 2, 1, ready smile!",
                "Get ready for your picture! Three, two, one, smile!"
            ],
            "photo_taken_feedback": [
                "Photo taken! That's a great shot!",
                "Captured! Everyone's smiling beautifully!"
            ],
            "no_smile_feedback": [
                "Hmm, I didn't see everyone smiling. Would you like to try again?",
                "Looks like someone wasn't quite ready. Let's try once more?"
            ],
            "view_photo": [
                "Here's the last photo we took. What do you think?",
                "Displaying the picture now.",
                "Take a look at this one!"
            ],
            "save_photo": [
                "Saving the photo now.",
                "Photo saved!"
            ],
            "goodbye_command": [
                "Goodbye! It was a pleasure assisting you. Have a great day!",
                "See you next time! I'll be here if you need me."
            ],
            "timeout_prompt": [
                "Are you still there?",
                "Hello? Do you still need assistance?"
            ],
            "unrecognized_command": [""],
            "long_timeout_exit": [
                "It seems like you're busy. I'll go to sleep now. Say 'Hi Milo' to wake me up.",
                "I'll be here when you need me. Going into standby."
            ]
        }

        rospy.loginfo("Interaction Manager Node initialized. Waiting for commands on /voice_command topic.")
        self.set_conversation_state("passive")

    def set_conversation_state(self, state):
        """Publishes the conversation state to voice_listener."""
        if state == "active":
            rospy.loginfo("Interaction Manager: Setting conversation state to ACTIVE.")
            self.conversation_active_by_manager = True
            self.conversation_state_publisher.publish(state)
        elif state == "passive":
            rospy.loginfo("Interaction Manager: Setting conversation state to PASSIVE.")
            self.conversation_active_by_manager = False
            self.conversation_state_publisher.publish(state)

    def camera_ready_callback(self, msg):
        if msg.data == "camera ready":
            rospy.loginfo("InteractionManager: Camera ready.")
            self.camera_ready = True
            self.check_and_start_detection()

    def image_view_ready_callback(self, msg):
        if msg.data == "image_view ready":
            rospy.loginfo("InteractionManager: Image viewer ready.")
            self.image_view_ready = True
            self.check_and_start_detection()

    def check_and_start_detection(self):
        if self.camera_ready and self.image_view_ready:
            rospy.loginfo("InteractionManager: Both camera and image view ready. Waiting for face detection.")
            self.detection_active = True
            self.countdown_started = False
            self.camera_ready = False
            self.image_view_ready = False

    def face_detection_started_callback(self, msg):
        if self.detection_active and not self.countdown_started:
            rospy.loginfo("InteractionManager: Received face detection signal. Starting photo countdown.")
            self.countdown_started = True 
            self.say_text(self.get_random_response("countdown_smile"))
            
            rospy.sleep(6)

            self.start_detection_publisher.publish("activate_smile_classification")
            rospy.loginfo("InteractionManager: Sent 'activate_smile_classification' to SmileDetector.")


    def smile_detected_callback(self, msg):
        if msg.data: # Smile detected (True)
            rospy.loginfo("InteractionManager: Received smile detected (TRUE).")
            self.say_text(self.get_random_response("photo_taken_feedback"))
            rospy.sleep(3) 
            self.start_detection_publisher.publish("stop")
            self.detection_active = False
        else: # No smile detected (False)
            self.say_text(self.get_random_response("no_smile_feedback"))
            rospy.loginfo("InteractionManager: Received smile detected (FALSE).")
            self.detection_active = False
            self.start_detection_publisher.publish("stop")

    def photo_saved_callback(self, msg):
        rospy.loginfo(f"[InteractionManager] Received photo feedback: {msg.data}")
        self.say_text("Photo saved.")

    def tts_status_callback(self, msg):
        """Callback to receive TTS busy status."""
        self.tts_busy = (msg.data == "busy")
        rospy.set_param('tts_busy', self.tts_busy)


    def say_text(self, text):
        """Helper function to publish text to the /say_text topic."""
        rospy.loginfo("Interaction Manager says: '%s'", text)
        self.say_text_publisher.publish(text)
        self.last_spoken_phrase = text
        self.last_activity_time = time.time()
        self.last_prompt_time = time.time()


    def get_random_response(self, command_key):
        """Returns a random response from the defined list for a given command key."""
        responses = self.response_phrases.get(command_key, self.response_phrases["unrecognized_command"])
        return random.choice(responses)

    def command_callback(self, msg):
        """
        Callback function for when a command is received from the voice listener.
        Determines the appropriate robot response and manages state.
        """
        command = msg.data
        rospy.loginfo("Received command from voice_listener: '%s'", command)
        
        self.last_activity_time = time.time()
        self.last_prompt_time = time.time()

        self.set_conversation_state("active") 

        if command not in ["robot_greet", "goodbye_command", "repeat"]:
            rospy.sleep(1)

        if command == "repeat":
            if self.last_spoken_phrase:
                self.say_text("I just said: " + self.last_spoken_phrase)
            else:
                self.say_text(self.get_random_response("repeat_fallback"))
     
        elif command == "goodbye_command":
            self.say_text(self.get_random_response(command))
            rospy.sleep(3)
            self.set_conversation_state("passive")

        elif command == "take_photo": 
            if not self.detection_active:
                self.say_text(self.get_random_response("take_photo_initial"))
                rospy.sleep(1) 
                self.start_detection_publisher.publish("init_camera")
            else:
                self.say_text("I'm already in a photo session. Please wait or say 'stop' to end.")

        elif command == "view_photo":
            self.view_photo_publisher.publish()
            rospy.sleep(1)
            self.say_text(self.get_random_response(command))

        elif command == "retry": 
            if not self.detection_active: 
                self.say_text(self.get_random_response("take_photo_initial"))
                rospy.sleep(1) 
                self.start_detection_publisher.publish("init_camera")
            else:
                self.say_text("I'm already in a photo session. Please wait or say 'stop' to end.")

        elif command == "stop": 
            if self.detection_active:
                self.start_detection_publisher.publish("stop")
                self.detection_active = False
                self.say_text("Photo session stopped.")
        
        else:
            self.say_text(self.get_random_response(command))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        manager = InteractionManager()
        manager.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Interaction Manager node shut down cleanly.")
    except Exception as e:
        rospy.logerr("Failed to start Interaction Manager node: %s", str(e))