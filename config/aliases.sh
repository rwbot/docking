#!/usr/bin/env bash

# Source this file for the current terminal
#       source ~/dock_ws/src/docking/config/aliases.sh
# or Add it to your .bashrc to auto-source once a terminal opens
#       echo "source ~/dock_ws/src/docking/config/aliases.sh" >> ~/dock_ws/src/.bashrc

alias cb="catkin build"
alias ccy="catkin clean -y"
alias ccdy="catkin clean docking -y"
alias ccb="ccy && catkin build"
alias dws="cd ~/dock_ws/"
alias cdd="cd ~/dock_ws/src/docking"
alias dcy="cdd; ccy"
alias dcb="cdd; cb"
alias dcyb="cdd; ccy; dcb; sws"

alias swsd="source ~/dock_ws/devel/setup.bash"
alias swsm="source ~/mappy_ws/devel/setup.bash"

alias rcpy="rosclean purge -y"

alias rlmd="rcpy; swsm; roslaunch docking mappy_dock.launch"
alias rltsd="rcpy; swsd; roslaunch docking turtlebot_sim_dock.launch"
alias rltd="rcpy; swsd; roslaunch docking turtlebot_dock.launch"

alias rld="swsd; roslaunch docking detection_node.launch"
alias rlc="swsd; roslaunch docking controller_node.launch"
alias rlp="swsd; roslaunch docking planner_node.launch"

alias rrim="swsd; rosrun dockbot dock_gazebo_interactive_marker"
alias rlim="swsd; roslaunch dockbot dock_gazebo_interactive_marker.launch"
alias rldrz="swsd; swsm; roslaunch docking rviz.launch"

alias rlpc="swsd; roslaunch docking pose_controller_node.launch"

alias sshpi="ssh rw@10.252.165.216"
alias tb3rob="roslaunch turtlebot3_bringup turtlebot3_robot.launch"

alias tb3up="sshpi; tb3rob"
alias tb3rem="roslaunch turtlebot3_bringup turtlebot3_remote.launch"

alias swss="source ~/shuttle_ws/devel/setup.bash"
alias rlss="swss; roslaunch elevator_and_grid_gazebo elevator_world_with_shuttle.launch"