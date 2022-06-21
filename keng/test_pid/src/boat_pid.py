#! /usr/bin/env python3

import rospy 
from std_msgs.msg import Float64
import time

class R_PID(object): 
    def __init__(self, init_setPoint_value, init_state):
        self.setPoint_value = init_setPoint_value
        self.state_value = init_state

        self._setpoint_pub = rospy.Publisher("/control_right_wheel/command", Float64, queue_size=1)
        self._state_pub = rospy.Publisher("/enc_vel_r", Float64, queue_size=1)       

        self._control_effort_sub = rospy.Subscriber('/r_control_effort', Float64, self.control_effort_callback) 
        self._control_effort_value = Float64() 

        self.control_effort_ready()

    def control_effort_callback(self, data):
        try:
            self._control_effort_value.data = data.data 
        except:
            rospy.logerr(
                "Current /r_control_effort not ready yet, retrying for getting control_effort"
            )       

    def control_effort_ready(self): 
        self._control_effort_value = None 

        rospy.logdebug("Waiting for /r_control_effort to be READY...")

        while self._control_effort_value is None and not rospy.is_shutdown():
            rospy.logdebug("Publishing Initial State and Setpoint...")
            rospy.logdebug("State="+str(self.state_value)+",SetValue="+str(self.setPoint_value))

            self.state_update(value=self.setPoint_value)
            self.setpoint_update(value=self.setPoint_value)

            rospy.logdebug("Publishing Initial State and Setpoint...DONE")
            try:
                self._control_effort_value = rospy.wait_for_message(
                    '/r_control_effort', Float64, timeout=1.0)
                rospy.logdebug("Current /r_control_effort READY=>")

            except:
                rospy.logerr(
                    "Current /r_control_effort not ready yet, retrying for getting control_effort")

    def setpoint_update(self, value):
        msg = Float64()
        msg.data = value
        self._setpoint_pub.publish(msg)

    def state_update(self, value):
        msg = Float64()
        msg.data = value
        self._state_pub.publish(msg)

    def get_control_effort(self):
        return self._control_effort_value.data

class L_PID(object): 
    def __init__(self, init_setPoint_value, init_state):
        self.setPoint_value = init_setPoint_value
        self.state_value = init_state

        self._setpoint_pub = rospy.Publisher("/control_left_wheel/command", Float64, queue_size=1)
        self._state_pub = rospy.Publisher("/enc_vel_l", Float64, queue_size=1)       

        self._control_effort_sub = rospy.Subscriber('/l_control_effort', Float64, self.control_effort_callback) 
        self._control_effort_value = Float64() 

        self.control_effort_ready()

    def control_effort_callback(self, data):
        try:
            self._control_effort_value.data = data.data
        except:
            rospy.logerr(
                "Current /l_control_effort not ready yet, retrying for getting control_effort"
            )       

    def control_effort_ready(self): 
        self._control_effort_value = None 

        rospy.logdebug("Waiting for /l_control_effort to be READY...")

        while self._control_effort_value is None and not rospy.is_shutdown():
            rospy.logdebug("Publishing Initial State and Setpoint...")
            rospy.logdebug("State="+str(self.state_value)+",SetValue="+str(self.setPoint_value))

            self.state_update(value=self.setPoint_value)
            self.setpoint_update(value=self.setPoint_value)

            rospy.logdebug("Publishing Initial State and Setpoint...DONE")
            try:
                self._control_effort_value = rospy.wait_for_message(
                    '/l_control_effort', Float64, timeout=1.0)
                rospy.logdebug("Current /l_control_effort READY=>")

            except:
                rospy.logerr(
                    "Current /l_control_effort not ready yet, retrying for getting control_effort")
   
    def setpoint_update(self, value):
        msg = Float64()
        msg.data = value
        self._setpoint_pub.publish(msg)

    def state_update(self, value):
        msg = Float64()
        msg.data = value
        self._state_pub.publish(msg)

    def get_control_effort(self):
        return self._control_effort_value.data

class VR(object):
    def __init__(self):
        self._vr_from_enc = rospy.Subscriber('/enc_vel_r', Float64, self.vr_from_enc_callback) 
        self._vr_enc = Float64() 
        self._vr_from_command = rospy.Subscriber('/control_right_wheel/command', Float64, self.vr_from_command_callback) 
        self._vr_command = Float64()     
        
    def vr_from_enc_callback(self, data):
        self._vr_enc = data
    def get_vr_from_enc(self):
        return self._vr_enc.data
    def vr_from_command_callback(self, data):
        self._vr_command = data
    def get_vr_from_command(self):
        return self._vr_command.data

class VL(object):
    def __init__(self):
        self._vl_from_enc = rospy.Subscriber('/enc_vel_l', Float64, self.vl_from_enc_callback) 
        self._vl_enc = Float64() 
        self._vl_from_command = rospy.Subscriber('/control_left_wheel/command', Float64, self.vl_from_command_callback) 
        self._vl_command = Float64()
        
    def vl_from_enc_callback(self, data):
        self._vl_enc = data
    def get_vl_from_enc(self):
        return self._vl_enc.data
    def vl_from_command_callback(self, data):
        self._vl_command = data   
    def get_vl_from_command(self):
        return self._vl_command.data

def pid_test():
    
    rospy.init_node('pid_test', anonymous=True)

    vr = VR()
    pid_vr = R_PID(vr.get_vr_from_command, vr.get_vr_from_enc)

    vl = VL()
    pid_vl = L_PID(vl.get_vl_from_command, vl.get_vl_from_enc)

    rate = rospy.Rate(15.0)
    ctrl_c = False 
    def shutdownhook(): 
        rospy.loginfo("shutdown time!")
        ctrl_c = True 
    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:

        pid_vr.setpoint_update(vr.get_vr_from_command)
        pid_vr.state_update(vr.get_vr_from_enc)

        pid_vl.setpoint_update(vl.get_vl_from_command)
        pid_vl.state_update(vl.get_vl_from_enc)

        rospy.loginfo("r_setPoint_value ==>"+str(vr.get_vr_from_command))
        rospy.loginfo("r_state_value ==>"+str(vr.get_vr_from_enc))
        rospy.loginfo("r_effort_value ==>"+str(pid_vr.get_control_effort))

        rospy.loginfo("l_setPoint_value ==>"+str(vl.get_vl_from_command))
        rospy.loginfo("l_state_value ==>"+str(vl.get_vl_from_enc))
        rospy.loginfo("l_effort_value ==>"+str(pid_vl.get_control_effort))

        rate.sleep()

if __name__ == '__main__': 
    pid_test()