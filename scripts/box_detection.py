#!/usr/bin/env python3 

import rospy 
from sensor_msgs.msg import LaserScan ,PointField ,PointCloud2
from std_msgs.msg import Header 
import sensor_msgs.point_cloud2 as pc2 
from geometry_msgs.msg import Pose 

from visualization_msgs.msg import Marker

import numpy as np 

scan_points = None 

def scan_callback(msg) : 
    global scan_points
    ranges = np.asarray(msg.ranges)
    angle_min = msg.angle_min
    angle_max = msg.angle_max
    angle_increment = msg.angle_increment

    angle = np.linspace(angle_min,angle_max,len(ranges))
    
    x = ranges * np.cos(angle)
    y = ranges * np.sin(angle)
    z = np.zeros_like(x)

    pts = np.array([x,y,z])
    idx = (np.abs(pts[0]) != np.inf) & ((np.abs(pts[1]) != np.inf))
    pts = pts[:,idx]
    scan_points = pts.T

def numpy_to_rosmsg(cloud_data,stamp=0,frame_id='map') : 
    

    header = Header()
    if stamp == 0 : 
        header.stamp = rospy.Time().now()
    else : 
        header.stamp = rospy.Time().from_sec(stamp/1e9)

    header.frame_id = frame_id 

    if len(cloud_data.T) > 3 : 
        POINTCLOUD_FIELDS = [
        PointField(name='x' ,offset=0 ,datatype=PointField.FLOAT32 ,count=1),
        PointField(name='y' ,offset=4 ,datatype=PointField.FLOAT32 ,count=1),
        PointField(name='z' ,offset=8 ,datatype=PointField.FLOAT32 ,count=1),
        PointField(name='intensity',offset=12,datatype=PointField.FLOAT32 ,count=1)]

        cloud_data = np.c_[cloud_data[:,0],cloud_data[:,1],cloud_data[:,2],cloud_data[:,3]]
    else : 
        POINTCLOUD_FIELDS = [
        PointField(name='x' ,offset=0 ,datatype=PointField.FLOAT32 ,count=1),
        PointField(name='y' ,offset=4 ,datatype=PointField.FLOAT32 ,count=1),
        PointField(name='z' ,offset=8 ,datatype=PointField.FLOAT32 ,count=1)]

        cloud_data = np.c_[cloud_data[:,0],cloud_data[:,1],cloud_data[:,2]]
    # print(cloud_data)

    return pc2.create_cloud(header,POINTCLOUD_FIELDS,cloud_data)

def create_marker(id,pose,marker_type,color,scale,frame_id="odom") : 
    marker = Marker()

    marker.header.frame_id = frame_id 
    marker.type = marker_type 
    marker.id = id 

    marker.scale.x = scale[0]
    marker.scale.y = scale[1]
    marker.scale.z = scale[2]

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]

    marker.pose = pose 

    return marker 

if __name__ == "__main__" : 
    rospy.init_node("box_dectection")

    scan_sub = rospy.Subscriber("scan",LaserScan,scan_callback)
    cloud_detect_pub = rospy.Publisher("cloud_detect",PointCloud2,queue_size=10)
    cloud_object_pub = rospy.Publisher("cloud_object",PointCloud2,queue_size=10)
    object_radius_marker_pub = rospy.Publisher("object_radius_marker",Marker,queue_size=10)
    object_marker_pub = rospy.Publisher("object_marker",Marker,queue_size=10)
    object_pose_pub = rospy.Publisher("object_pose",Pose,queue_size=10)

    max_range = rospy.get_param("~max_range",2)
    object_diameter = rospy.get_param("~object_diameter",0.5)

    rate = rospy.Rate(10)

    try : 
        while not rospy.is_shutdown() : 
            if scan_points is not None :

                inrange_idx = np.linalg.norm(scan_points,axis=1) <= max_range 
                front_idx = scan_points[:,0] >= 0 

                selected_points = scan_points[inrange_idx & front_idx]
                distance = np.linalg.norm(selected_points,axis=1)
                nearest_idx = distance == np.min(distance)
                nearest_point = selected_points[nearest_idx][0]

                object_distance = np.linalg.norm(selected_points - nearest_point,axis=1)
                object_points = selected_points[object_distance <= object_diameter]

                object_position = np.mean(object_points,axis=0)

                object_radius_marker_pose = Pose()
                object_radius_marker_pose.position.x = nearest_point[0]
                object_radius_marker_pose.position.y = nearest_point[1]
                object_radius_marker_pose.position.z = nearest_point[2]
                object_radius_marker_pose.orientation.x = 0 
                object_radius_marker_pose.orientation.y = 0 
                object_radius_marker_pose.orientation.z = 0 
                object_radius_marker_pose.orientation.w = 1 

                object_marker_pose = Pose()
                object_marker_pose.position.x = object_position[0]
                object_marker_pose.position.y = object_position[1]
                object_marker_pose.position.z = object_position[2]
                object_marker_pose.orientation.x = 0
                object_marker_pose.orientation.y = 0
                object_marker_pose.orientation.z = 0
                object_marker_pose.orientation.w = 1
                
                object_radius_marker = create_marker(0,object_radius_marker_pose,Marker.SPHERE,(1,0,0,0.3),(object_diameter,object_diameter,object_diameter),frame_id="base_footprint")
                object_marker = create_marker(0,object_marker_pose,Marker.SPHERE,(0,0,1,1),(0.1,0.1,0.1),frame_id="base_footprint")

                object_radius_marker_pub.publish(object_radius_marker)
                object_marker_pub.publish(object_marker)
                object_pose_pub.publish(object_marker_pose)
                cloud_detect_pub.publish(numpy_to_rosmsg(selected_points,frame_id="base_footprint"))
                cloud_object_pub.publish(numpy_to_rosmsg(object_points,frame_id="base_footprint"))

            rate.sleep() 

    except rospy.ROSInterruptException : 
        pass 