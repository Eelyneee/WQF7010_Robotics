#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import random
import threading
import time

class InteractionManager:
    def __init__(self):
        rospy.init_node('interaction_manager', anonymous=True)

        self.command_subscriber = rospy.Subscriber('/voice_command', String, self.command_callback)
        self.say_text_publisher = rospy.Publisher('/say_text', String, queue_size=10)
        self.conversation_state_publisher = rospy.Publisher('/milo_conversation_state', String, queue_size=1)
        rospy.Subscriber('/tts_status', String, self.tts_status_callback)

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
            # NEW: Processing phrases for internal delay feedback
            "processing_command": [
                "Hmm...",
                "One moment..."
            ],
            "take_photo": [
                "Okay, getting ready to take a photo. Smile!",
                "On my mark, get set, smile!",
                "Picture in 3, 2, 1... captured!"
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
            "check_smile": [
                "I'm checking for smiles... Looking good!",
                "A little wider please!",
                "Perfect smiles everyone!"
            ],
            "check_expressions": [
                "I see happy faces all around!",
                "It looks like someone is a little tired, but still cute!"
            ],
            "suggest_pose": [
                "Try standing back to back, or maybe a jumping pose!",
                "How about everyone looking up at the sky?"
            ],
            "check_framing": [
                "Yes, everyone is perfectly in frame!",
                "Looks like the person on the left is a little cut off. Could you shift slightly!"
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
        
        # Start the timeout monitoring thread
        self.monitor_thread = threading.Thread(target=self.monitor_activity)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        
        # Initially, set the state to passive
        rospy.sleep(3) # Give publisher time to connect
        self.set_conversation_state("passive")


    def set_conversation_state(self, state):
        """Publishes the conversation state to voice_listener."""
        if state == "active" and not self.conversation_active_by_manager:
            rospy.loginfo("Interaction Manager: Setting conversation state to ACTIVE.")
            self.conversation_active_by_manager = True
            self.conversation_state_publisher.publish(state)
        elif state == "passive" and self.conversation_active_by_manager:
            rospy.loginfo("Interaction Manager: Setting conversation state to PASSIVE.")
            self.conversation_active_by_manager = False
            self.conversation_state_publisher.publish(state)


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

        # If we receive a command, it means the conversation should be active
        self.set_conversation_state("active") 

        # NEW: Give quick processing feedback before main response
        # We avoid this for "robot_greet" (initial greeting), "goodbye_command" (immediate exit),
        if command not in ["robot_greet", "goodbye_command", "repeat"]:
            self.say_text(self.get_random_response("processing_command"))
            rospy.sleep(3) # Give a small pause for the "thinking" sound to play and be heard

        if command == "repeat":
            if self.last_spoken_phrase:
                self.say_text("I just said: " + self.last_spoken_phrase)
            else:
                self.say_text(self.get_random_response("repeat_fallback"))
        elif command == "goodbye_command":
            self.say_text(self.get_random_response(command))
            # Give time for goodbye message to be spoken before going passive
            rospy.sleep(3) # Ensure speech completes before changing state
            self.set_conversation_state("passive") # ONLY go passive on goodbye
        else:
            response_text = self.get_random_response(command)
            self.say_text(response_text)


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

                # If inactive for 'timeout_duration' and not just prompted
                if time_since_last_activity > self.timeout_duration and time_since_last_prompt > self.timeout_duration:
                    rospy.loginfo("Interaction Manager: Inactivity detected (%.2f seconds). Prompting user.", time_since_last_activity)
                    self.say_text(self.get_random_response("timeout_prompt"))
                    self.last_prompt_time = time.time() # Update prompt time so it doesn't prompt immediately again

                # If inactive for a very long time, truly go passive
                if time_since_last_activity > self.long_timeout_duration:
                    rospy.loginfo("Interaction Manager: Long inactivity detected (%.2f seconds). Going to passive mode.", time_since_last_activity)
                    self.say_text(self.get_random_response("long_timeout_exit"))
                    rospy.sleep(3) # Give time for message to be delivered
                    self.set_conversation_state("passive")
            
            rospy.sleep(3) # Check activity every second


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