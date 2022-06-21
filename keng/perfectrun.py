#!/usr/bin/env python3

import serial
import time

import rospy

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client(x,y,z,w):

   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
   # Move 0.5 meters forward along the x axis of the "map" coordinate frame 
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
   # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.z = z
    goal.target_pose.pose.orientation.w = w

   # Sends the goal to the action server.
    client.send_goal(goal)
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:

        #pos
        x1 = 2
        y1 = 0
        #orien
        z1 = 0.707
        w1 = 0.707

        #pos
        x2 = 2
        y2 = 2
        #orien
        z2 = 0.707
        w2 = 0.707

        arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=.1)

        #Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')

        result = movebase_client(x1,y1,z1,w1)
        if result:
            rospy.loginfo("Goal execution done!")

        arduino.write(bytes("up", 'utf-8'))

        result2 = movebase_client(x2,y2,z2,w2)
        if result2:
            rospy.loginfo("Goal execution done!")

        arduino.write(bytes("down", 'utf-8'))

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
