#!/usr/bin/env python
import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan

rospy.init_node('road_publisher')

scan_pub = rospy.Publisher('road_scan', LaserScan, queue_size=1)

num_readings = 180
laser_frequency = 1
max_range= 100
intensity = 100
count = 0
scale = 1.0

limit1=0
limit2=45
limit3=135
limit4=179

r = rospy.Rate(1.0)

# sub sample the image
# Pixel world coordinates
# Only black points
points = np.empty((180,4),dtype=float)

for i in range(0,180):
    points[i,0]=np.sin(np.radians(i))
    points[i,1]=np.cos(np.radians(i))


def get_angle(angle):
    angle=angle*180.0/np.pi
    angle=round(angle)
    if angle<limit1:
        angle=limit1
    elif angle>limit4:
        angle=limit4

    return int(angle)



while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    scan = LaserScan()

    scan.header.stamp = current_time
    scan.header.frame_id = 'multisense/left_camera_optical_frame'
    scan.angle_min = -1.57
    scan.angle_max = 1.57
    scan.angle_increment = 3.14 / num_readings
    scan.time_increment = (1.0 / laser_frequency) / (num_readings)
    scan.range_min = 0.0
    scan.range_max = 100.0

    scan.ranges = []
    scan.intensities = []
    for i in range(0, num_readings):
        scan.ranges.append(max_range)
        scan.intensities.append(100)


    # Assuming x up, y right from front of the image
    # print "----"
    for i in range(0,points.shape[0]):
        dist=np.sqrt(points[i,0]**2 + points[i,1]**2)
        direction=math.atan2(points[i,0],points[i,1])

        angle=get_angle(direction)

        if angle<limit2 or angle>limit3:
            if dist<scan.ranges[angle]:
                scan.ranges[angle]=dist

    scan_pub.publish(scan)
    count += 1
r.sleep()