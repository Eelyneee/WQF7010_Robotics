#!/usr/bin/env python
import rospy
from std_msgs.msg import String

# Importing time for simulated delays, in a real scenario, this would be replaced by actual voice processing.
import time

class VoiceListener:
    """
    ROS Node for listening to voice commands and publishing them.

    This node implements a specific interaction flow:
    1. Passively listens for the wake word "Hi Smily".
    2. Once the wake word is detected, it publishes a "robot_greet" command.
       (The interaction_manager will pick this up and use tts_feedback.py to say the greeting).
    3. After the greeting is triggered, the node enters a "conversation mode" where it
       actively processes subsequent commands without requiring the wake word again,
       until an explicit "goodbye" command is given, or the conversation times out
       (handled by interaction_manager).
    """
    def __init__(self):
        # Initialize the ROS node with a unique name
        rospy.init_node('voice_listener', anonymous=True)

        # Create a publisher for the /voice_command topic
        # std_msgs.msg.String is used for simple text messages
        self.command_publisher = rospy.Publisher('/voice_command', String, queue_size=10)

        # Define the specific wake word for the robot
        self.wake_word = "hi smily"

        # A flag to indicate if the wake word has been detected and we are in conversation mode
        self.conversation_mode_active = False

        # Set the publishing rate (optional, but good practice for continuous nodes)
        self.rate = rospy.Rate(1) # 1 Hz

        rospy.loginfo("Voice Listener Node initialized. Passively waiting for wake word: '%s'", self.wake_word)

    def process_command(self, text_input):
        """
        Processes the input text to detect the wake word or subsequent commands.
        In a real application, this would involve more sophisticated Natural Language Understanding (NLU).
        """
        # Convert input to lowercase for case-insensitive matching and strip whitespace
        text_input_lower = text_input.lower().strip()

        if not self.conversation_mode_active:
            # We are in passive listening mode, only check for the wake word
            if self.wake_word in text_input_lower:
                self.conversation_mode_active = True
                rospy.loginfo("Wake word detected: '%s'. Publishing 'robot_greet' and entering conversation mode.", self.wake_word)
                # Publish a special command that interaction_manager will use to trigger the greeting
                self.command_publisher.publish("robot_greet")
            else:
                rospy.loginfo("Passively listening. Still waiting for wake word: '%s'...", self.wake_word)
        else:
            # We are in conversation mode, actively process commands
            # Remove the wake word if it was inadvertently included in later commands for cleaner processing
            command_text = text_input_lower.replace(self.wake_word, "").strip()

            if "take photo" in command_text or "let's take picture" in command_text:
                rospy.loginfo("Recognized command: take_photo")
                self.command_publisher.publish("take_photo")
                # Do not reset conversation_mode_active; stay in conversation mode for more commands
            elif "check smile" in command_text:
                rospy.loginfo("Recognized command: check_smile")
                self.command_publisher.publish("check_smile")
            elif "try again" in command_text:
                rospy.loginfo("Recognized command: retry")
                self.command_publisher.publish("retry")
            elif "say that again" in command_text or "repeat" in command_text:
                rospy.loginfo("Recognized command: repeat")
                self.command_publisher.publish("repeat")
            elif "view photo" in command_text:
                rospy.loginfo("Recognized command: view_photo")
                self.command_publisher.publish("view_photo")
            # Add a command to explicitly exit conversation mode
            elif "goodbye" in command_text or "stop listening" in command_text or "exit conversation" in command_text:
                rospy.loginfo("Recognized command to exit conversation. Exiting conversation mode.")
                self.command_publisher.publish("goodbye_command") # Let interaction_manager handle the "goodbye" response
                self.conversation_mode_active = False # Go back to passive listening
            else:
                rospy.loginfo("Command not recognized: '%s'. Staying in conversation mode.", command_text)
                self.command_publisher.publish("unrecognized_command") # Notify interaction_manager about unrecognized command

    def run(self):
        """
        Main loop for the node.
        In this simulation, it prompts for user input via the console.
        In a real robot system, this would listen to a microphone and use a speech recognition library
        to get 'text_input'.
        """
        while not rospy.is_shutdown():
            try:
                # Prompt user for input, simulating voice input
                # The prompt changes based on whether conversation mode is active
                prompt_message = "Say 'Hi Smily' to start, or a command if already active: "
                if self.conversation_mode_active:
                    prompt_message = "Say a command (e.g., 'take photo', or 'goodbye'): "

                user_input = raw_input(prompt_message)
                self.process_command(user_input)
            except EOFError:
                # Handle Ctrl+D or end of input gracefully for console simulation
                rospy.loginfo("Exiting voice listener due to EOF.")
                break
            except Exception as e:
                rospy.logerr("Error in voice listener loop: %s", str(e))
            self.rate.sleep() # Maintain the loop rate

if __name__ == '__main__':
    try:
        listener = VoiceListener()
        listener.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Voice Listener node shut down cleanly.")
    except Exception as e:
        rospy.logerr("Failed to start Voice Listener node: %s", str(e))

