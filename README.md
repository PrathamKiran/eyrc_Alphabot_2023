# EYRC 2023 Workspace

## Clone the workspace into your home directory
```
git clone git@github.com:PrathamKiran/eyrc_Alphabot.git
```
## Source Files
Add these files into the .bashrc file using the command
``` 
gedit ~/.bashrc
```
Copy and paste these 4 commands to the end of the file
```
# Set up autocomplete for colcon workspaces
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

# Sourcing for ROS and Turtlebot
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle

# Sourcing Gazebo Setup
source /usr/share/gazebo-11/setup.bash

# Sourcing the workspace
source ~/eyrc_Alphabot/install/setup.bash
```

## Build the workspace
```
cd ~/eyrc_Alphabot
colcon build
```

# Evaluate the tasks
```
cd ~/eyrc_Alphabot/eval/
```
Then run the evaluation command for the particular task