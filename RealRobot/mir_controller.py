#!/usr/bin/env python
import numpy as np
import rospy
import predictor
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path, MapMetaData
from math import atan2, sqrt, pi, cos 

def odom_callback(data):
    global linear_speed
    global angular_speed
    linear_speed = (data.twist.twist.linear.x)
    angular_speed = (data.twist.twist.angular.z)

   

def f_lidar_callback(data):
    global f_lidar
    f_lidar = lidarToZones(np.flip(data.ranges))
    #print("F lidar")
    #print(f_lidar.tolist())


def b_lidar_callback(data):
    global b_lidar
    b_lidar = lidarToZones(np.flip(data.ranges))
    #print("B lidar")
    #print(b_lidar.tolist())

def f_lidarToZones(data):
    global FrontSafetyDistances
    hits = np.zeros(len(data))

    lidarMax = 2
    overlap = 13
    zones = 8

    for x in range(0,541):

        lidarRange = data[x]

        if lidarRange > (lidarMax+FrontSafetyDistances[x]) or lidarRange < 0.1:
            hits[x]=1
        else:
            hits[x]=round(((lidarRange-FrontSafetyDistances[x])*20))/(lidarMax*20)

    lidarInput = np.ones(zones)

    x_range = int((541/8)+2*overlap)

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

def b_lidarToZones(data):
    global BackSafetyDistances
    hits = np.zeros(len(data))

    lidarMax = 2
    overlap = 13
    zones = 8

    for x in range(0,541):

        lidarRange = data[x]

        if lidarRange > (lidarMax+BackSafetyDistances[x]) or lidarRange < 0.1:
            hits[x]=1
        else:
            hits[x]=round(((lidarRange-BackSafetyDistances[x])*20))/(lidarMax*20)

    lidarInput = np.ones(zones)

    x_range = int((541/8)+2*overlap)

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
    global x_pose
    global y_pose
    global path_length
    if running == False:
        path_index = 0
        length_a = len(data.poses)
        path_length = length_a
        if length_a > 0:
            #print("New path")
            running = True
            glo_path = np.zeros((2,length_a))
            for x in reversed(range(0,length_a)): #reversed(
                glo_path[0,x] = data.poses[x].pose.position.x
                glo_path[1,x] = data.poses[x].pose.position.y

            #print(glo_path)
                


def robot_pose_callback(data):
    #print(data.position)
    global x_pose
    global y_pose
    global angle

    x_pose = data.position.x
    y_pose = data.position.y

    q = data.orientation
    siny_cosp = 2*(q.w * q.z)
    cosy_cosp = 1-2*(q.z*q.z)
    angle = atan2(siny_cosp, cosy_cosp)


def unit_vector(vector):
    return vector/np.linalg.norm(vector)

def angle_between(v1, v2):
    v1_u=unit_vector(v1)
    v2_u=unit_vector(v2)
    return atan2(v1_u[0]*v2_u[1]-v1_u[1]*v2_u[0],v1_u[0]*v2_u[0]+v1_u[1]*v2_u[1]) #np.arccos(np.clip(np.dot(v1_u,v2_u),-1.0,1.0)) #a = atan2(v1_u[0]*v2_u[1]-v1_u[1]*v2_u[0],v1_u[0]*v2_u[0]+v1_u[1]*v2_u[1])

def update_rob_pos():
    global running
    global robot_angle
    global path_index
    global glo_path
    global x_pose
    global y_pose
    global angle
    global d1
    global path_length

    rob_x = x_pose
    rob_y = y_pose

    if path_length != 0 and path_length > path_index+2:
        #print(path_index)
        d1 = sqrt((glo_path[0,path_index]       -rob_x)**2+(glo_path[1,path_index]      -rob_y)**2)
        d2 = sqrt((glo_path[0,(path_index+1)]   -rob_x)**2+(glo_path[1,(path_index+1)]  -rob_y)**2)

        while d1 >= d2 and path_length > path_index+2:
            path_index = path_index + 1
            d1 = d2
            d2 = sqrt((glo_path[0,path_index+1]-rob_x)**2+(glo_path[1,path_index+1]-rob_y)**2)

        if path_length > path_index+41:
            v1 = np.array([glo_path[0,path_index+40],glo_path[1,path_index+40]])
        else:
            v1 = np.array([glo_path[0,path_length-1],glo_path[1,path_length-1]])
            d = sqrt((glo_path[0,path_length-1]       -rob_x)**2+(glo_path[1,path_length-1]      -rob_y)**2)
            if d < 1:
                running = False

        v2 = np.array([rob_x,rob_y])
        v3 = v1 - v2
        simple = np.array([1,0])
        robot_angle = angle_between(simple,v3)


def move():
    global linear_speed
    global angular_speed
    global b_lidar
    global f_lidar
    global running
    global robot_angle
    global path_index
    global glo_path
    global x_pose
    global y_pose
    global angle
    global d1
    # output to file
    #f = open('recording.txt','w')
    #Tensorflow stuff
    pred=predictor.Predictions("MiR_Robot_LBrain.pb")
    # calc safety distances
    global FrontSafetyDistances
    global BackSafetyDistances

    lengthB = 0.6

    vinkelB = 30.7
    lengthC = 0.4594

    FrontSafetyDistances = np.zeros(541)
    degreesPrLaser = 240.0/(541-1)
    for i in range(0,541):
        vinkelB = (i * degreesPrLaser) + 30.7
        if vinkelB > 180.0:
            vinkelB = 180.0 - (vinkelB - 180.0)
        FrontSafetyDistances[i] = lengthC * cos(np.deg2rad(vinkelB)) + sqrt(lengthB**2+lengthC**2 * cos(np.deg2rad(vinkelB))**2 - lengthC**2)

    vinkelB = 30.96
    lengthC = 0.4558

    BackSafetyDistances = np.zeros(541)
    degreesPrLaser = 240.0/(541-1)
    for i in range(0,541):
        vinkelB = (i * degreesPrLaser) + 30.96
        if vinkelB > 180.0:
            vinkelB = 180.0 - (vinkelB - 180.0)
        BackSafetyDistances[i] = lengthC * cos(np.deg2rad(vinkelB)) + sqrt(lengthB**2+lengthC**2 * cos(np.deg2rad(vinkelB))**2 - lengthC**2)

    # Starts a new node
    rospy.init_node('MiR_controller', anonymous=True)

    rospos_sub = rospy.Subscriber("/robot_pose",Pose,robot_pose_callback)
    b_laserScan_sub = rospy.Subscriber("/b_scan", LaserScan, b_lidar_callback)  ## len(data.ranges)
    f_laserScan_sub = rospy.Subscriber("/f_scan", LaserScan, f_lidar_callback)

    time = 2.5
    timeStep = 1/time
    rate = rospy.Rate(time)
    rate.sleep()

    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    odom_sub = rospy.Subscriber("/odom",Odometry,odom_callback)
    plan_sub = rospy.Subscriber("/move_base_node/global_plan",Path,path_callback)
    
    vel_msg = Twist()

    #Variabler
    maxDeviation = 2.0

    maxLinearVel = 1.1
    maxReverseVel = 0.3

    maxAngularVel = 1

    simLinearVel = 0.8
    simReverseVel = 0.2
    

    #Receiveing the user's input
    #print("Let's move your robot")

    time_frame_1 = np.zeros(20)
    time_frame_1.tolist()

    time_frame_2 = np.zeros(20)
    time_frame_2.tolist()

    # time_frame_3 = np.zeros(20)
    # time_frame_3.tolist()

    # time_frame_4 = np.zeros(20)
    # time_frame_4.tolist()
    
    # time_frame_5 = np.zeros(20)
    # time_frame_5.tolist()

    running = False
    key = bool(input("Start?"))

    while not rospy.is_shutdown():
        rate.sleep()

        if running:
            update_rob_pos()

            angle_dif = (angle - robot_angle)
            if angle_dif > pi:
                angle_dif = angle_dif - 2 * pi
            elif angle_dif < -pi:
                angle_dif = angle_dif + 2 * pi

           
            linear_speed = (linear_speed * (maxLinearVel/simLinearVel))
            angular_speed = angular_speed * (1/maxAngularVel)

            input_array = ([round(linear_speed,2),round(angular_speed,2), round((angle_dif/pi),2),round(d1/maxDeviation,2)])   
            
            input_array.extend(f_lidar.tolist())
            input_array.extend(b_lidar.tolist())

            #time_frame_5 = time_frame_4
            #time_frame_4 = time_frame_3
            #time_frame_3 = time_frame_2
            time_frame_2 = time_frame_1
            time_frame_1 = input_array

            input_tensor = np.array([time_frame_1,time_frame_2]) # ,time_frame_3,time_frame_4,time_frame_5
            input_tensor = input_tensor.flatten()

            output_tensor = np.array([0,0])
            eps = np.array([0.2,0.2])
            output_tensor = pred.getPrediction([eps], [input_tensor])

            linear_vel = output_tensor[0][1]

            angular_vel = output_tensor[0][0] * maxAngularVel


            if sqrt(linear_vel**2+angular_vel**2) > 1 and linear_vel > 0:
                linear_vel = sqrt(1-angular_vel**2)
            
            linear_vel = linear_vel * (simLinearVel/maxLinearVel)
            
            if linear_vel < 0:
                linear_vel = linear_vel * 0.25

            print("Input array")
            print(input_array)
            #print("Angle dif")
            #print(angle_dif)


            vel_msg.linear.x = linear_vel
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = angular_vel
            velocity_publisher.publish(vel_msg)
            print(vel_msg)
        else:
            #f.close()
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)



if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass
