#! /usr/bin/python2
'''
Visualize Semantic Map of Guang-Fu road
'''
import rospy
import json
import os
from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import sensor_msgs.point_cloud2 as pcl2
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
from std_msgs.msg import Header

'''
Setting root dir of semantic file (.json)
'''
map_path = '/home/kevin/Documents/Dataset/hsinchu_guangfu_road/'

'''
Features to visualize
'''
map_types = [
        'roadlines', 
        'pedestrian_crossing', 
        'curbs', 
        'no_temperate_parking_zone', 
        'roadmarkers'
]
'''
Some feature is belong to enclosed type
'''
map_enclose_types = {
    'roadlines': False, 
    'pedestrian_crossing': True,
    'curbs': False,
    'no_temperate_parking_zone': True,
    'roadmarkers': True
}
'''
Setting colors
'''
map_colors = {
    'roadlines': [0.7, 0, 0.7], 
    'pedestrian_crossing': [0, 0, 1],
    'curbs': [0, 1, 0], 
    'no_temperate_parking_zone': [1, 0, 0],
    'roadmarkers': [0.7, 0.7, 1]
}

'''
Initialize wrapper
'''
map_dict = {}
rl_markers = MarkerArray()

'''
Load semantic map (.json file)
'''
def load_map(result_file):
    global map_dict

    for sub_map_name in map_types:
        with open (os.path.join(result_file, sub_map_name+'.json'), mode='r') as f:
            map_dict[sub_map_name] = json.load(f)[sub_map_name]
        
        print('Name: {}/ Len: {}'.format(sub_map_name, len(map_dict[sub_map_name])))


def viz_roadpolys(pub, sub_map_name):
    global rl_markers

    for index, road_poly in enumerate(map_dict[sub_map_name]):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.action = Marker.ADD
        marker.ns = sub_map_name
        marker.type = Marker.LINE_LIST
        marker.lifetime = rospy.Duration(0.1)

        marker.color.r = map_colors[sub_map_name][0]
        marker.color.g = map_colors[sub_map_name][1]
        marker.color.b = map_colors[sub_map_name][2]
        marker.color.a = 1
        marker.scale.x = 0.1

        marker.points = []
        marker.id = road_poly.get('id', index) 

        pts = road_poly['points']
        for index, pt in enumerate(pts):
            # Don't enclose polygon excepts first and last pts
            if (not map_enclose_types[sub_map_name]) and index == len(pts)-1:
                break
            marker.points.append(Point(
                pts[index%len(pts)]['x'], 
                pts[index%len(pts)]['y'],
                pts[index%len(pts)]['z']
                ))
            marker.points.append(Point(
                pts[(index+1)%len(pts)]['x'], 
                pts[(index+1)%len(pts)]['y'], 
                pts[(index+1)%len(pts)]['z']
                ))
        rl_markers.markers.append(marker)
            
if __name__ == "__main__":    
    rospy.init_node("pub_map_node", anonymous=True)
    mPubRoadlines = rospy.Publisher(
                        'roadlines', 
                        MarkerArray, 
                        queue_size=100
                    )
    
    load_map(map_path)

    for sub_map_name in map_dict.keys():
        viz_roadpolys(mPubRoadlines, sub_map_name)
    
    mPubRoadlines.publish(rl_markers)

    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        mPubRoadlines.publish(rl_markers)
        rate.sleep()
