#! /usr/bin/python

import numpy as np
import rospy
import roslib
from std_msgs.msg import String, Int32, Float32, Float64
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray, FiducialArray
from geometry_msgs.msg import Transform, Quaternion, Vector3
from nav_msgs.msg import Odometry
import math as m
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rosservice

transform = FiducialTransform()
img_seq = Int32()

name = String()

tx = 0
ty = 0
tz = 0

rz = 0

x0 = 0
y0 = 0
x1 = 0
y1 = 0
x2 = 0
y2 = 0
x3 = 0
y3 = 0

pose_x = 0
pose_y = 0
pose_z = 0

goal_x = 0
goal_y = 0
global yaw
yaw = 0


markerLength = 0.2

f = FiducialTransform()
  
def fiducial_callback(msg):

    global img_seq, transform, tx, ty, tz, rz, name
    header = msg.header
    img_seq = msg.image_seq
    transform = msg.transforms
    name = header.frame_id

    for f in transform:
        tx = f.transform.translation.x
        ty = f.transform.translation.y
        tz = f.transform.translation.z
        rz = f.transform.rotation.z

def vertecies_callback(msg):

    global x0, y0, x1, y1, x2, y2, x3, y3
    
    fiducials = msg.fiducials
    for n in fiducials:
        
        x0 = n.x0
        y0 = n.y0
        x1 = n.x1
        y1 = n.y1
        x2 = n.x2
        y2 = n.y2
        x3 = n.x3
        y3 = n.y3

def odom_callback(msg):
    global pose_x, pose_y, pose_z, yaw

    pose_x = msg.pose.pose.position.x
    pose_y = msg.pose.pose.position.y
    pose_z = msg.pose.pose.position.z

    yaw = msg.pose.pose.orientation.z

def movebase_client(target_x, target_y, target_r_z, target_r_w):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    
    goal.target_pose.pose.position.x = target_x
    goal.target_pose.pose.position.y = target_y
   
    #goal.target_pose.pose.orientation.x = 0.0
    #goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = target_r_z
    goal.target_pose.pose.orientation.w = target_r_w

    client.send_goal(goal)


#    wait = client.wait_for_result()
#    if not wait:
#        rospy.logerr("Action server not available!")
#        rospy.signal_shutdown("Action server not available!")
#    else:
#        rospy.loginfo("Goal finished, send new one")
#        return 
#    rospy.loginfo("Goal Sent with (1,0)")
   
    

def main():
    #rospy.sleep(.25)
    
    # Initialize node
    rospy.init_node("aruco_2_navgoal", anonymous=True)
    rospy.loginfo("node Initialize")
    d = rospy.Duration(0,25)
    rospy.sleep(d)
    # Subsribers
    aruco_t_sub = rospy.Subscriber("fiducial_transforms", FiducialTransformArray, fiducial_callback)
    aruco_vetecies_sub = rospy.Subscriber("fiducial_vertices", FiducialArray, vertecies_callback)
    odom_pose_sub = rospy.Subscriber("odom", Odometry, odom_callback)
    
    #rospy.sleep(1)

    rate = rospy.Rate(10)
   
    rospy.loginfo(">> calculating navigation goal")
    
    if transform != [] and x0 != 0.0 and tx != 0.0:
        if name == "frnt_cam_opt":
            #beta = yaw
            iden = 1
            #print(1)
        elif name == "rear_cam_opt":
            #beta = yaw + m.pi
            iden = 2
            #print(2)
        elif name == "left_cam_opt":
            #beta = yaw + m.pi/2
            iden = 3
            #print(3)
        elif name == "right_cam_opt":
            #beta = yaw - m.pi/2
            iden = 4
            #print(4)
        else:
            print("No")
            
    else:
        return
    print("This is x0: " + str(x0))
    rospy.loginfo(">> Sending Navigation goal")
    
    if x0 != 0.0 and tx != 0.0:
        #print(tx, ty, tz)
        t = np.array([tx, ty, tz])
        aruco_x = np.linalg.norm(t)-1
        a_cx = ((x0 + x1 + x2 + x3)/4)
        
        goal_r_z = m.sin(yaw/2)
        goal_r_w = m.cos(yaw/2)
        ratio = markerLength/(((x1 - x0)+(x2 - x3))/2)

        aruco_y = (256 - a_cx) * ratio

        angle = m.atan(aruco_y/aruco_x)

        alpha = yaw + angle
        if iden == 1:
            if alpha >= 0 and alpha < m.pi/2:
                goal_x = pose_x - aruco_x*m.cos((alpha)) 
                goal_y = pose_y - aruco_y*m.sin((alpha))
            elif alpha > m.pi/2:
                goal_x = pose_x + aruco_y*m.sin((alpha))
                goal_y = pose_y - aruco_x*m.cos((alpha)) 
            elif alpha < 0 and alpha > -m.pi/2:
                goal_x = pose_x - aruco_x*m.cos((alpha)) 
                goal_y = pose_y + aruco_y*m.sin((alpha))
            elif alpha < -m.pi/2 :
                goal_x = pose_x + aruco_x*m.cos((alpha)) 
                goal_y = pose_y + aruco_y*m.sin((alpha))
            else:
                goal_x = 0
                goal_y = 0
        elif iden == 2:
            if alpha >= 0 and alpha < m.pi/2:
                goal_x = pose_x + aruco_x*m.cos((alpha)) 
                goal_y = pose_y + aruco_y*m.sin((alpha))
            elif alpha > m.pi/2:
                goal_x = pose_x - aruco_y*m.sin((alpha))
                goal_y = pose_y + aruco_x*m.cos((alpha)) 
            elif alpha < 0 and alpha > -m.pi/2:
                goal_x = pose_x + aruco_x*m.cos((alpha)) 
                goal_y = pose_y - aruco_y*m.sin((alpha))
            elif alpha < -m.pi/2 :
                goal_x = pose_x - aruco_x*m.cos((alpha)) 
                goal_y = pose_y - aruco_y*m.sin((alpha))
            else:
                goal_x = 0
                goal_y = 0
        elif iden == 3:
            if alpha >= 0 and alpha < m.pi/2:
                goal_y = pose_y - aruco_x*m.cos((alpha)) 
                goal_x = pose_x + aruco_y*m.sin((alpha))
            elif alpha > m.pi/2:
                goal_y = pose_y + aruco_y*m.sin((alpha))
                goal_x = pose_x + aruco_x*m.cos((alpha)) 
            elif alpha < 0 and alpha > -m.pi/2:
                goal_y = pose_y - aruco_x*m.cos((alpha)) 
                goal_x = pose_x - aruco_y*m.sin((alpha))
            elif alpha < -m.pi/2 :
                goal_y = pose_y + aruco_x*m.cos((alpha)) 
                goal_x = pose_x - aruco_y*m.sin((alpha))
            else:
                goal_x = 0
                goal_y = 0
        elif iden == 4:
            if alpha >= 0 and alpha < m.pi/2:
                goal_y = pose_y - aruco_x*m.cos((alpha)) 
                goal_x = pose_x - aruco_y*m.sin((alpha))
            elif alpha > m.pi/2:
                goal_y = pose_y - aruco_y*m.sin((alpha))
                goal_x = pose_x + aruco_x*m.cos((alpha)) 
            elif alpha < 0 and alpha > -m.pi/2:
                goal_y = pose_y + aruco_x*m.cos((alpha)) 
                goal_x = pose_x - aruco_y*m.sin((alpha))
            elif alpha < -m.pi/2 :
                goal_y = pose_y + aruco_x*m.cos((alpha)) 
                goal_x = pose_x + aruco_y*m.sin((alpha))
            else:
                goal_x = 0
                goal_y = 0
        else:
            goal_x = 0
            goal_y = 0

        rospy.loginfo(goal_x)
        rospy.loginfo(goal_y)
        print("This is yaw goal: " + str(goal_r_z))
        movebase_client(goal_x, goal_y, 0, 1)

        
    else: 
        movebase_client(pose_x, pose_y, 0, 1)
        print("No aruco detected")




if __name__ == "__main__":

    while True:
        main()
        
