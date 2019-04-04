#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from math import atan2, sqrt, pi, cos

def odom_callback(data):
    global linear_speed
    global angular_speed
    global x_pose
    global y_pose
    global angle
    linear_speed = (data.twist.twist.linear.x)
    angular_speed = (data.twist.twist.angular.z)
    x_pose = data.pose.pose.position.x
    y_pose = data.pose.pose.position.y

    q = data.pose.pose.orientation
    siny_cosp = 2*(q.w * q.z)
    cosy_cosp = 1-2*(q.z*q.z)
    angle = atan2(siny_cosp, cosy_cosp)

def f_lidar_callback(data):
    global f_lidar
    f_lidar = lidarToZones(np.flip(data))


def b_lidar_callback(data):
    global b_lidar
    b_lidar = lidarToZones(data)

def lidarToZones(data):
    hits = np.zeros(len(data.ranges))

    overlap = 13
    zones = 8

    for x in range(0,541):

        lidarRange = data.ranges[x]

        if lidarRange > 5 or lidarRange < 0.1:
            hits[x]=1
        else:
            hits[x]=(lidarRange-safetyDistances[x])/5

    lidarInput = np.ones(zones)

    x_range = int(541/8+2*overlap)

    for x in range(0,x_range):
        if x<((541/zones)+overlap):
            if hits[x] < lidarInput[0]:
                lidarInput[0] = hits[x]

            if hits[int(x+((541/zones)*(zones-1)-overlap))]<lidarInput[zones-1]:
                lidarInput[zones-1] = hits[int(x+((541/zones)*(zones-1)-overlap))]

        for y in range(1,zones-1):
            if hits[int(x+((541/zones)*y)-overlap)] < lidarInput[y]:
                lidarInput[y]=hits[int(x+((541/zones)*y)-overlap)]

    return lidarInput

def path_callback(data):
    global glo_path
    global path_index
    global running
    global offset_x
    global offset_y
    global offset_angle
    global x_pose
    global y_pose

    running = False
    path_index = 0
    length_a = len(data.poses)

    if length_a > 0:
        print("New path")
        running = True
        glo_path = np.zeros((2,length_a))
        for x in reversed(range(0,length_a)):
            glo_path[0,x] = data.poses[x].pose.position.x
            glo_path[1,x] = data.poses[x].pose.position.y
        #np.flip(glo_path,0)
        index = 0
        while glo_path[0,0] == glo_path[0,index]:
            index +=1
        path_index = index
        v1 = np.array([glo_path[0,0],glo_path[1,0]])
        v2 = np.array([glo_path[0,index],glo_path[1,index]])
        v3 = v1 - v2
        simple = np.array([0,1])
        offset_angle = angle_between(simple,v3)
        offset_x = glo_path[0,0] - x_pose
        offset_y = glo_path[1,0] - y_pose



def unit_vector(vector):
    return vector/np.linalg.norm(vector)

def angle_between(v1, v2):
    v1_u=unit_vector(v1)
    v2_u=unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u,v2_u),-1.0,1.0))

def update_rob_pos():
    global robot_angle
    global path_index
    global glo_path
    global x_pose
    global y_pose
    global angle
    global offset_x
    global offset_y

    rob_x = (x_pose + offset_x)
    rob_y = (y_pose + offset_y)

    if (glo_path.size/2) != 0 and (glo_path.size/2) > path_index+8:
        print(path_index)
        d1 = sqrt((glo_path[0,path_index]       -rob_x)**2+(glo_path[1,path_index]      -rob_y)**2)
        d2 = sqrt((glo_path[0,(path_index+1)]   -rob_x)**2+(glo_path[1,(path_index+1)]  -rob_y)**2)
        print(d1)
        print(d2)
        while d1 >= d2 and (glo_path.size/2) > path_index+2:
            path_index = path_index + 1
            d1 = d2
            d2 = sqrt((glo_path[0,path_index+1]-rob_x)**2+(glo_path[1,path_index+1]-rob_y)**2)

        v1 = np.array([glo_path[0,path_index+4],glo_path[1,path_index+4]])
        v2 = np.array([rob_x,rob_y])
        v3 = v1 - v2
        simple = np.array([0,1])
        robot_angle = -1* angle_between(simple,v3)


def move():
    # calc safety distances
    global safetyDistances
    vinkelB = 33.09
    lengthB = 0.6
    lengthC = 0.531
    safetyDistances = np.zeros(541)
    degreesPrLaser = 270/(541-1)
    for i in range(0,541):
        vinkelB = (i * degreesPrLaser) + 33.09
        if vinkelB > 180.0:
            vinkelB = 180.0 - vinkelB - 180.0
        safetyDistances[i] = lengthC * cos(np.deg2rad(vinkelB)) + sqrt(lengthB**2+lengthC**2 * cos(np.deg2rad(vinkelB))**2 - lengthC**2)
        #safetyDistances[i] = lengthC * Mathf.Cos(vinkelB * Mathf.Deg2Rad) + Mathf.Sqrt(Mathf.Pow(lengthB, 2) + Mathf.Pow(lengthC, 2) * Mathf.Pow(Mathf.Cos(vinkelB * Mathf.Deg2Rad), 2) - Mathf.Pow(lengthC, 2));

    # Starts a new node
    rospy.init_node('MiR_controller', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    b_laserScan_sub = rospy.Subscriber("/b_scan", LaserScan, b_lidar_callback)  ## len(data.ranges)
    f_laserScan_sub = rospy.Subscriber("/f_scan", LaserScan, f_lidar_callback)
    odom_sub = rospy.Subscriber("/odom",Odometry,odom_callback)
    plan_sub = rospy.Subscriber("/move_base_node/global_plan",Path,path_callback)
    vel_msg = Twist()

    #Receiveing the user's input
    print("Let's move your robot")

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()
        if running:
            update_rob_pos()
            newAngle = angle - robot_angle
            if newAngle < -pi:
                newAngle += 2*pi
            elif newAngle > pi:
                newAngle -= 2*pi
            newAngle += offset_angle
            if newAngle < -pi:
                newAngle += 2*pi
            elif newAngle > pi:
                newAngle -= 2*pi
            #print(newAngle)


#        speed = float(input("Input your speed:"))
#        angular = float(input("Input your angular speed:"))
#        vel_msg.linear.x = speed
#        vel_msg.linear.y = 0
#        vel_msg.linear.z = 0
#        vel_msg.angular.x = 0
#        vel_msg.angular.y = 0
#        vel_msg.angular.z = angular
#        timer = 0
#        while timer < 10:
#            velocity_publisher.publish(vel_msg)
#            rate.sleep()
#            timer = timer + 1

#        vel_msg.linear.x = 0
#        vel_msg.linear.y = 0
#        vel_msg.linear.z = 0
#        vel_msg.angular.x = 0
#        vel_msg.angular.y = 0
#        vel_msg.angular.z = 0
#        velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass
