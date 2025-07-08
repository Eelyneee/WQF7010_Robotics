#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Empty, Bool # Import Bool for /smile_detected
import random
import threading
import time

class InteractionManager:
    def __init__(self):
        rospy.init_node('interaction_manager', anonymous=True)

        self.command_subscriber = rospy.Subscriber('/voice_command', String, self.command_callback)
        self.say_text_publisher = rospy.Publisher('/say_text', String, queue_size=10)
        self.conversation_state_publisher = rospy.Publisher('/milo_conversation_state', String, queue_size=1, latch=True)

        rospy.Subscriber('/tts_status', String, self.tts_status_callback)
        rospy.Subscriber('/camera_ready', String, self.camera_ready_callback)
        rospy.Subscriber('/image_view_ready',String,self.image_view_ready_callback)
        rospy.Subscriber("/photo_saved_feedback", String, self.photo_saved_callback)
        rospy.Subscriber("/face_detection_started", Empty, self.face_detection_started_callback) # Subscriber for new signal
        rospy.Subscriber('/smile_detected', Bool, self.smile_detected_callback) # NEW: Subscriber to get smile feedback

        self.smile_classification_start_time = None
        self.SMILE_CLASSIFICATION_PHASE_TIMEOUT = 15 

        self.camera_ready = False
        self.image_view_ready = False   
        self.detection_active = False # Tracks if we're in an active photo session
        self.countdown_started = False # New flag to prevent multiple countdowns in one session

        # Trigger channels for other nodes
        self.start_detection_publisher = rospy.Publisher('/start_detection', String, queue_size=1)
        self.retry_publisher = rospy.Publisher('/retry_signal', Empty, queue_size=1)
        self.view_photo_publisher = rospy.Publisher('/view_photo_request', Empty, queue_size=1)

        self.last_spoken_phrase = ""
        self.last_activity_time = time.time() # Timestamp of last command or robot speech
        self.conversation_active_by_manager = False # Internal state for manager's logic

        # Timeout settings
        self.timeout_duration = 10 # Seconds to wait for user response before prompting (e.g., "Are you still there?")
        self.long_timeout_duration = 30 # Seconds of total inactivity before completely going passive (optional, for robustness)
        self.last_prompt_time = 0 # To avoid prompting too frequently

        self.tts_busy = False # Flag to track if TTS is currently speaking

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
            "take_photo_initial": [ # New initial phrase before camera setup
                "Booting up camera, please wait.",
                "Starting the camera system."
            ],
            "countdown_smile": [ # New phrase for the actual countdown
                "3, 2, 1, ready smile!",
                "Get ready for your picture! Three, two, one, smile!"
            ],
            "photo_taken_feedback": [ # New phrase for successful photo
                "Photo taken! That's a great shot!",
                "Captured! Everyone's smiling beautifully!"
            ],
            "no_smile_feedback": [ # New phrase for no smile detected
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
            "help_command": [
                "I can help you take photos, check smiles, view your last shot, and more! Just tell me what you need.",
                "I'm here to assist with your photography needs. You can ask me to take a photo, check your smile, or even suggest a pose!"
            ],
            "retry": [
                "Okay, let's try that again. What was the command?"
            ],
            "repeat_fallback": [
                "I haven't said anything yet in this conversation."
            ],
            "timeout_prompt": [
                "Are you still there?",
                "Hello? Do you still need assistance?"
            ],
            "unrecognized_command": [""],
            "long_timeout_exit": [ # New phrase for truly going passive
                "It seems like you're busy. I'll go to sleep now. Say 'Hi Milo' to wake me up.",
                "I'll be here when you need me. Going into standby."
            ]
        }

        rospy.loginfo("Interaction Manager Node initialized. Waiting for commands on /voice_command topic.")
        
        # Start the timeout monitoring thread (optional, if you want this feature)
        # self.monitor_thread = threading.Thread(target=self.monitor_activity)
        # self.monitor_thread.daemon = True
        # self.monitor_thread.start()
        
        # rospy.sleep(3) # Give publisher time to connect
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
        # This function now only confirms camera/viewer are up
        if self.camera_ready and self.image_view_ready:
            rospy.loginfo("InteractionManager: Both camera and image view ready. Waiting for face detection.")
            self.detection_active = True # Set overall session active
            self.countdown_started = False # Reset countdown flag for new session
            # Reset flags for next time
            self.camera_ready = False
            self.image_view_ready = False

    def face_detection_started_callback(self, msg):
        # Triggered when SmileDetectorRealTime first detects a face
        if self.detection_active and not self.countdown_started:
            rospy.loginfo("InteractionManager: Received face detection signal. Starting photo countdown.")
            self.countdown_started = True # Prevent multiple countdowns
            self.say_text(self.get_random_response("countdown_smile"))
            
            rospy.sleep(6)

            self.start_detection_publisher.publish("activate_smile_classification")
            rospy.loginfo("InteractionManager: Sent 'activate_smile_classification' to SmileDetector.")


    def smile_detected_callback(self, msg):
        # Reset the smile classification timer as we've received a result
        # self.smile_classification_start_time = None 

        if msg.data: # Smile detected (True)
            rospy.loginfo("InteractionManager: Received smile detected (TRUE).")
            self.say_text(self.get_random_response("photo_taken_feedback"))
            rospy.sleep(3) 
            # If a smile is detected, we always stop the session.
            self.start_detection_publisher.publish("stop")
            self.detection_active = False
            # self.set_conversation_state("passive")
        else: # No smile detected (False)
            self.say_text(self.get_random_response("no_smile_feedback"))
            rospy.loginfo("InteractionManager: Received smile detected (FALSE).")
            self.detection_active = False
            self.start_detection_publisher.publish("stop")

    def photo_saved_callback(self, msg):
        rospy.loginfo(f"[InteractionManager] Received photo feedback: {msg.data}")
        # This callback is for external saving feedback, not directly part of smile detection loop
        self.say_text("Photo saved.")

    def tts_status_callback(self, msg):
        """Callback to receive TTS busy status."""
        self.tts_busy = (msg.data == "busy")
        rospy.set_param('tts_busy', self.tts_busy) # Update ROS param for voice_listener to check


    def say_text(self, text):
        """Helper function to publish text to the /say_text topic."""
        rospy.loginfo("Interaction Manager says: '%s'", text)
        self.say_text_publisher.publish(text)
        self.last_spoken_phrase = text
        self.last_activity_time = time.time() # Update activity time after speaking
        self.last_prompt_time = time.time() # Reset prompt timer when Milo speaks


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
        
        # Always update last activity time on user command
        self.last_activity_time = time.time()
        self.last_prompt_time = time.time() # Reset prompt timer on user command

        self.set_conversation_state("active") 

        if command not in ["robot_greet", "goodbye_command", "repeat"]:
            # self.say_text(self.get_random_response("processing_command"))
            rospy.sleep(1) # Give a small pause for the "thinking" sound to play and be heard

        if command == "repeat":
            if self.last_spoken_phrase:
                self.say_text("I just said: " + self.last_spoken_phrase)
            else:
                self.say_text(self.get_random_response("repeat_fallback"))
     
        elif command == "goodbye_command":
            self.say_text(self.get_random_response(command))
            rospy.sleep(3) # Give time for goodbye message to be spoken
            self.set_conversation_state("passive")

        elif command == "take_photo": # "check_smile" will also go through photo process
            if not self.detection_active: # Prevent starting multiple sessions
                self.say_text(self.get_random_response("take_photo_initial"))
                rospy.sleep(1) # Give time for this phrase
                self.start_detection_publisher.publish("init_camera")
                # The rest of the flow is driven by the camera_ready/image_view_ready/face_detection_started callbacks
            else:
                self.say_text("I'm already in a photo session. Please wait or say 'stop' to end.")

        elif command == "view_photo":
            self.view_photo_publisher.publish()
            rospy.sleep(1)
            self.say_text(self.get_random_response(command))

        elif command == "retry" or command == "check_smile": # "check_smile" will also go through photo process
            if not self.detection_active: # Prevent starting multiple sessions
                self.say_text(self.get_random_response("take_photo_initial"))
                rospy.sleep(1) # Give time for this phrase
                self.start_detection_publisher.publish("init_camera")
                # The rest of the flow is driven by the camera_ready/image_view_ready/face_detection_started callbacks
            else:
                self.say_text("I'm already in a photo session. Please wait or say 'stop' to end.")


        elif command == "stop": # Allow user to stop photo session explicitly
            if self.detection_active:
                self.start_detection_publisher.publish("stop")
                self.detection_active = False
                self.say_text("Photo session stopped.")
                # self.set_conversation_state("passive")
            # else:
            #     self.say_text("I'm not currently in a photo session.")
        
        else:
            self.say_text(self.get_random_response(command))


    def monitor_activity(self):
        """
        Monitors for user inactivity and prompts or transitions the conversation to passive state.
        Runs in a separate thread.
        """
        while not rospy.is_shutdown():
            if self.conversation_active_by_manager and not self.tts_busy:
                current_time = time.time()
                time_since_last_activity = current_time - self.last_activity_time
                time_since_last_prompt = current_time - self.last_prompt_time

                if time_since_last_activity > self.timeout_duration and time_since_last_prompt > self.timeout_duration:
                    rospy.loginfo("Interaction Manager: Inactivity detected (%.2f seconds). Prompting user.", time_since_last_activity)
                    self.say_text(self.get_random_response("timeout_prompt"))
                    self.last_prompt_time = time.time()

                if time_since_last_activity > self.long_timeout_duration and not self.detection_active: # Don't go passive if actively detecting smile
                    rospy.loginfo("Interaction Manager: Long inactivity detected (%.2f seconds). Going to passive mode.", time_since_last_activity)
                    self.say_text(self.get_random_response("long_timeout_exit"))
                    rospy.sleep(3)
                    self.set_conversation_state("passive")
            
            rospy.sleep(3)


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