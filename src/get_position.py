#! /usr/bin/env python3

"""
ROS Driver for Water Linked Underwater GPS G2

Get position from Water Linked Underwater GPS G2

Contact: Nayan Man Singh Pradhan (nayan.pradhan@hotmail.com)
"""

import argparse
import json
from logging import shutdown
import requests
import rospy 
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Point

NAME_OF_NODE = "waterlinked_underwater_gps_g2_node"
BASE_URL = rospy.get_param("base_url")
USE_ANTENNA = int(rospy.get_param("use_antenna"))

class GPS_G2:

    def __init__(self):
        rospy.loginfo("Using BASE_URL:"+str(BASE_URL))
        if (USE_ANTENNA):
            rospy.loginfo("Using Antenna: Use mid-point of base of antenna as origin for the acoustic " +
            "position. Default origin is the point at sea level directly " +
            "below/above that which the positions of the receivers/antenna " +
            "are defined with respect to")
        else:
            rospy.loginfo("Not using Antenna")
        
        self.acoustic_position_publisher = rospy.Publisher(NAME_OF_NODE+'/acoustic_position_publisher', Point, queue_size=10)
        self.global_position_publisher = rospy.Publisher(NAME_OF_NODE+'/global_position_publisher', GeoPoint, queue_size=10)

        while not rospy.is_shutdown():
            self.get_position_GPS_G2()

    
    def get_position_GPS_G2(self):
        
        acoustic_position_point = Point()
        acoustic_position_point.x = 0
        acoustic_position_point.y = 0
        acoustic_position_point.z = 0

        global_position_val = GeoPoint()
        global_position_val.latitude = 0
        global_position_val.longitude = 0
        global_position_val.altitude = 0


        acoustic_position = self.get_acoustic_position(BASE_URL)
        antenna_position = None 

        if (USE_ANTENNA):
            antenna_position = self.get_antenna_position(BASE_URL)

        depth = None 

        if acoustic_position:
            if antenna_position:
                _x = acoustic_position["x"] - antenna_position["x"]
                _y = acoustic_position["y"] - antenna_position["y"]
                _z = acoustic_position["z"] - antenna_position["depth"]
                print("Current acoustic position relative to antenna. X: {}, Y: {}, Z: {}".format(_x, _y, _z))

            else:
                _x = acoustic_position["x"]
                _y = acoustic_position["y"]
                _z = acoustic_position["z"]
                print("Current acoustic position. X: {}, Y: {}, Z: {}".format(_x, _y, _z))

            depth = acoustic_position["z"]

            acoustic_position_point.x = _x
            acoustic_position_point.y = _y
            acoustic_position_point.z = _z

        self.acoustic_position_publisher.publish(acoustic_position_point)

        global_position = self.get_global_position(BASE_URL)
        if global_position:
            lati = global_position["lat"]
            longi = global_position["lon"]

            if depth:
                depth = depth 
                print("Current global position. Latitude: {}, Longitude: {}, Depth: {}".format(lati, longi, depth))

            else: 
                depth = None 
                print("Current global position latitude :{} longitude :{}".format(lati, longi))

            global_position_val.latitude = lati
            global_position_val.longitude = longi
            global_position_val.altitude = depth
        
        self.global_position_publisher.publish(global_position_val)


    def get_data(self, url):
        try:
            r = requests.get(url)
        except requests.exceptions.RequestException as exc:
            print("Exception occured {}".format(exc))
            return None

        if r.status_code != requests.codes.ok:
            print("Got error {}: {}".format(r.status_code, r.text))
            return None

        return r.json()


    def get_acoustic_position(self, BASE_URL):
        return self.get_data("{}/api/v1/position/acoustic/filtered".format(BASE_URL))


    def get_antenna_position(self, BASE_URL):
        return self.get_data("{}/api/v1/config/antenna".format(BASE_URL))


    def get_global_position(self, BASE_URL, acoustic_depth = None):
        return self.get_data("{}/api/v1/position/global".format(BASE_URL))



def main():
    global NAME_OF_NODE
    global BASE_URL
    global USE_ANTENNA

    rospy.init_node(NAME_OF_NODE)
    rospy.loginfo("Starting node:   "+NAME_OF_NODE)

    GPS_G2()

    rospy.spin()


if __name__ == "__main__":
    main()