#!/usr/bin/env python
import rospy
from rosplane_msgs.msg import State
from rosflight_msgs.msg import GPS, Command

class autopilot:
    def __init__(self):
        self.update_rate = 20

        self.state_subscriber = rospy.Subscriber("/fixedwing/state", State, self.stateCallback, queue_size=20)
        self.gps_subscriber = rospy.Subscriber("/fixedwing/gps", GPS, self.gpsCallback, queue_size=10)
        self.currentState = State()
        self.currentGPS = GPS()

        rospy.Timer(rospy.Duration(1./self.update_rate), self.control)
        self.command_publisher = rospy.Publisher("/fixedwing/command", Command, queue_size=1)

    def stateCallback(self, msg):
        self.currentState = msg

    def gpsCallback(self, msg):
        if msg.fix and msg.NumSat > 3:
            self.currentGPS = msg

    def control(self, event):

        command = Command()
        command.mode = command.MODE_PASS_THROUGH
        command.ignore = command.IGNORE_NONE

        # edit starting right here!!!!

        # you can get GPS sensor information:
        # self.currentGPS.latitude # Deg
        # self.currentGPS.longitude # Deg
        # self.currentGPS.altitude # m
        # self.currentGPS.speed # m/s
        # self.currentGPS.ground_course # rad clockwise from the north

        # you can get the state estimate information:
        # self.currentState.position[0:3] # north, east, down (m) from starting location
        # self.currentState.Va		# Airspeed (m/s)
        # self.currentState.phi		# Roll angle (rad)
        # self.currentState.theta		# Pitch angle (rad)
        # self.currentState.chi_deg		# Wrapped (-180 to 180) course angle (deg)
        # self.currentState.p		# Body frame rollrate (rad/s)
        # self.currentState.q		# Body frame pitchrate (rad/s)
        # self.currentState.r		# Body frame yawrate (rad/s)

        command.F = 0.75 # throttle command 0.0 to 1.0
        command.x = 0.0 # aileron servo command -1.0 to 1.0  positive rolls to right
        command.y = -0.03 # elevator servo command -1.0 to 1.0  positive pitches up
        command.z = 0.0 # rudder servo command -1.0 to 1.0  positive yaws to left

        self.command_publisher.publish(command)

if __name__ == '__main__':
    rospy.init_node('autopilot_py', anonymous=True)
    ap = autopilot()
    rospy.spin()
