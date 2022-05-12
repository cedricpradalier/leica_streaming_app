#!/usr/bin/python

import rospy
from leica_streaming_app.msg import LeicaPoint
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
import sys


import rosbag
paths={}
bag = rosbag.Bag(sys.argv[1])
for topic, msg, t in bag.read_messages(topics=['/leica/leica_point']):
    if msg.point_id in paths:
        paths[msg.point_id].append(msg)
    else:
        paths[msg.point_id]=[msg]
bag.close()

q=Quaternion()
q.x=0
q.y=0
q.z=0
q.w=1

rospy.init_node('bag_to_path')

geom_paths={}
for pid in paths:
    P=Path()
    P.header=paths[pid][0].header
    for msg in paths[pid]:
        p = PoseStamped()
        p.header = msg.header
        p.pose.position.x=msg.x
        p.pose.position.y=msg.y
        p.pose.position.z=msg.z
        p.pose.orientation = q
        P.poses.append(p)
    pub=rospy.Publisher("~"+pid.lower(),Path,queue_size=1,latch=True)
    pub.publish(P)
    geom_paths[pid]=pub

rospy.loginfo("Loaded %d path" % len(geom_paths))
rospy.spin()

