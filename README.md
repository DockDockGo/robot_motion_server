# Action Servers and Clients to give Robot Navigation Commands

# Nodes Included

Each node represents one form of Navigation

1. Navigation Action Server
2. Dock/Undock Action Server

# Action Messages
The message formats for both action servers are present in the robot_action_interface package
along with all other interface definitions

# Action Server Logic

## High Level Logic

The action servers are part of the state machine on the robot which will call on each
action server independently

![](images/High%20Level.png)

## Low Level Logic

Each action server communicates vital feedback or results which need to be propogated to
the top level nodes. This specific communication is shown below

![](images/Low_level.png)

# Launching Action Servers

```bash
ros2 launch robot_motion_server motion_servers.launch.py
# or
ros2 run robot_motion_server robot_motion_server_node

# then
ros2 run robot_action_interfaces robot_pose_server.py

# then two action client calls
ros2 action send_goal /Navigate robot_action_interfaces/action/Navigate "{secs: 1.0}"

ros2 action send_goal /DockUndock robot_action_interfaces/action/DockUndock "{secs: 1.0}"
```