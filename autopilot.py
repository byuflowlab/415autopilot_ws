#!/usr/bin/env python
import rospy
from rosflight_msgs.msg import GPS, Command
import numpy as np


class autopilot:
    def __init__(self):
        self.update_rate = 10  # Hz

        self.gps_subscriber = rospy.Subscriber("/fixedwing/gps", GPS, self.gpsCallback, queue_size=10)
        self.currentGPS = GPS()

        rospy.Timer(rospy.Duration(1./self.update_rate), self.control)
        rospy.Timer(rospy.Duration(1./self.update_rate), self.check_status)
        self.command_publisher = rospy.Publisher("/fixedwing/command", Command, queue_size=1)

        self.wp_state_machine = 0
        self.minAlt = rospy.get_param('~minAlt', 1387.0 + 30.48)  # default is 100 ft
        self.maxAlt = rospy.get_param('~maxAlt', 1387.0 + 121.92)  # default is 400 ft
        self.wp1_lat = rospy.get_param('~wp1_lat', 40.267110)
        self.wp1_lon = rospy.get_param('~wp1_lon', -111.634983)
        self.wp2_lat = rospy.get_param('~wp2_lat', 40.267492)
        self.wp2_lon = rospy.get_param('~wp2_lon', -111.635755)
        self.wp3_lat = rospy.get_param('~wp3_lat', 40.266725)
        self.wp3_lon = rospy.get_param('~wp3_lon', -111.635809)
        self.wp4_lat = rospy.get_param('~wp4_lat', 40.267993)
        self.wp4_lon = rospy.get_param('~wp4_lon', -111.634930)

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

        command.F = 0.75  # throttle command 0.0 to 1.0
        command.x = 0.0  # aileron servo command -1.0 to 1.0  positive rolls to right
        command.y = -0.03  # elevator servo command -1.0 to 1.0  positive pitches up
        command.z = 0.0  # rudder servo command -1.0 to 1.0  positive yaws to left

        self.command_publisher.publish(command)

    def check_status(self, event):

        if self.currentGPS.altitude < self.minAlt:
            print "altitude too low!"
            return

        if self.currentGPS.altitude > self.maxAlt:
            print "altitude too high!"
            return

        if self.wp_state_machine == 0 and self.distance(self.wp1_lat, self.wp1_lon) < 10:
            self.wp_state_machine = 1
            print "achieved waypoint 1!"

        elif self.wp_state_machine == 1 and self.distance(self.wp2_lat, self.wp2_lon) < 10:
            self.wp_state_machine = 2
            print "achieved waypoint 2!"

        elif self.wp_state_machine == 2 and self.distance(self.wp3_lat, self.wp3_lon) < 10:
            self.wp_state_machine = 3
            print "achieved waypoint 3!"

        elif self.wp_state_machine == 3 and self.distance(self.wp4_lat, self.wp4_lon) < 10:
            self.wp_state_machine = 4
            print "achieved waypoint 4!"
            print "all waypoints achieved !!!"

    # def check_boundary(self, event):
    #     #---Define Boundary---#
    #     #majoraxis end points of boundary ellipse (Google Maps)
    #     ellipsemajor = np.array([40.268316 -111.635040; 40.266145 -111.636008])
    #
    #     #Midpoint of Major Axis set as center of ellipse
    #     center = np.array([mean(ellispemajor[:,0]), ellimpsmajor(wps[:,1])]) #center point of waypoints
    #
    #     #Selected point for Minor Axis Radius (Google Maps)
    #     ellipseminor = np.array([40.266994 -111.634407]) #rotated longitude axis point around the orange line
    #
    #     #Radius values for Major and Minor Axes
    #     rminalarm = np.sqrt((ellipsemionor[0]-center[0])**2 + (ellipseminor[1]-center[1])**2) #rotated longitude radius
    #     rmajalarm = np.sqrt((ellipsejamor[0,0]-center[0])**2 + (ellipsejamor[1,0]-center[1])**2) #rotated loatitude radius
    #
    #     #Selectd Rotation Angle
    #     rotate = -15*np.pi #angle of axes rotation 75 degrees from eastward axis.
    #
    #     #Equation of rotated Ellipse
    #     def ellipse(x,h,A,y,k,a,b):
    #         #x = gps longitude
    #         #y = gps latitude
    #         #h = longitude center (center[0])
    #         #k = latitude center (center[1])
    #         #A = angle of axes rotation
    #         #a = minor axis radius (rotated lonitude axis)
    #         #b = major axis radius (rotated latitude axis)
    #
    #         return (((x-h)*cos(A)+(y-k)*sin(A))^2)/(a^2) + (((x-h)*sin(A)-(y-k)*cos(A))^2)/(b^2)-1
    #
    #     #Alarm value (1 = on ellipse)
    #     alarm = ellipse() #TODO fill in inputs (see below function for self.gps stuff)
    #
    #     #Print Warning when approaching boundary
    #     if alarm > 0.9 && alarm <= 1.0:
    #         print "WARNING: OUTSIDE OF SAFE FLY ZONE."
    #
    #     #Print Alarm when crossing boundary
    #     if alarm > 1.0:
    #         print "ALERT!: IN NO FLY ZONE. TAKE OVER MANUAL CONTROL."

    def distance(self, wp_lat, wp_lon):
        EARTH_RADIUS = 6371000.0
        distN = EARTH_RADIUS*(self.currentGPS.latitude - wp_lat)*np.pi/180.0
        distE = EARTH_RADIUS*np.cos(wp_lat*np.pi/180.0)*(self.currentGPS.longitude - wp_lon)*np.pi/180.0
        return np.linalg.norm(np.array([distN, distE]))


if __name__ == '__main__':
    rospy.init_node('autopilot_py', anonymous=True)
    ap = autopilot()
    rospy.spin()
