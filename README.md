# WQF7010_Robotics

## Manual to Launch Robot Application
1. Download the smily_bot_ws folder from github and open in Juno 2 Robot.
2. Open Command Line Terminal and change the directory to smily_bot_ws. And run ‘roscore’ command after changing to the '''smily_bot_ws directory. 
cd smily_bot_ws'''s
roscore
Open another Command Line Terminal and run ‘source devel/setup.bash’ to set up your terminal environment to recognize your ROS workspace. Then, launches a set of ROS nodes and parameters defined in the smily_bot.launch file which is located in the interaction_manager package.
source devel/setup.bash
roslaunch interaction_manager smily_bot.launch

The user can wake the robot up by one of the following commands.
"hi milo"
"milo wake up"
"milo are you there"
If the user would like to take a photo, the user can say any of the following commands. Then the camera will be initialized and start detecting the face in frame and counting down. After counting 3, 2, 1, the robot will start smile detection. If all the smile labels are true, the photo will be taken. If there is at least 1 smile label detected as false, the robot will keep the smile detection until the end of the timeout period(30 seconds). 
"take photo"
"take a photo"
"take a picture"
"let's take a picture"
"click the shutter"
If the user would like to view a photo, the user can say any of the following commands. Then, an image view window will pop out and show the last photo shoot. 
"view photo"
"show me the last photo"
"view the picture"
"can i see that"
"review photos"
If the user would like to try again after the timeout period, the user can say any of the following commands. Then the camera will be initialized again and start detecting the face in frame and counting down. After counting 3, 2, 1, the robot will start smile detection.
"retry"
"try again"
"yes try again"
