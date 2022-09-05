#!/usr/bin/env python
# encoding: utf-8

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Int8
import actionlib
from move_base_msgs.msg import *
import sys, select, termios, tty
import time


def pose_callback(msg):
    global try_again, index, try_again, index
    if msg.status.status == 3 and count>0 :
        try_again = 1 
       
       
        if index == count:                  
            print ('Reach the target point '+str(index-1)+':')
            print('x:'+str(markerArray.markers[index-1].pose.position.x)+
                ', y:'+str(markerArray.markers[index-1].pose.position.y)+
                ', z:'+str(markerArray.markers[index-1].pose.orientation.z)+
                ', w:'+str(markerArray.markers[index-1].pose.orientation.w))   

            if count>1: print 'Complete instructions!' 
            index = 0;
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = markerArray.markers[index].pose.position.x
            pose.pose.position.y = markerArray.markers[index].pose.position.y
            pose.pose.orientation.z = markerArray.markers[index].pose.orientation.z
            pose.pose.orientation.w = markerArray.markers[index].pose.orientation.w
            goal_pub.publish(pose)
            index += 1 

        elif index < count:                   
            print ('Reach the target point '+str(index-1)+':')    
            print('x:'+str(markerArray.markers[index-1].pose.position.x)+
                ', y:'+str(markerArray.markers[index-1].pose.position.y)+
                ', z:'+str(markerArray.markers[index-1].pose.orientation.z)+
                ', w:'+str(markerArray.markers[index-1].pose.orientation.w)) 

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = markerArray.markers[index].pose.position.x
            pose.pose.position.y = markerArray.markers[index].pose.position.y
            pose.pose.orientation.z = markerArray.markers[index].pose.orientation.z
            pose.pose.orientation.w = markerArray.markers[index].pose.orientation.w
            goal_pub.publish(pose)
            index += 1 
        
    elif count>0: 
        rospy.logwarn('Can not reach the target point '+str(index-1)+':'+'\r\n'+
                      'x:'+str(markerArray.markers[index-1].pose.position.x)+
                    ', y:'+str(markerArray.markers[index-1].pose.position.y)+
                    ', z:'+str(markerArray.markers[index-1].pose.orientation.z)+
                    ', w:'+str(markerArray.markers[index-1].pose.orientation.w)) 

        
        if try_again == 1:
            rospy.logwarn('trying reach the target point '+str(index-1)+' again!'+'\r\n'+
                          'x:'+str(markerArray.markers[index-1].pose.position.x)+
                        ', y:'+str(markerArray.markers[index-1].pose.position.y)+
                        ', z:'+str(markerArray.markers[index-1].pose.orientation.z)+
                        ', w:'+str(markerArray.markers[index-1].pose.orientation.w)) 

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = markerArray.markers[index - 1].pose.position.x           
            pose.pose.position.y = markerArray.markers[index - 1].pose.position.y
            pose.pose.orientation.z = markerArray.markers[index-1].pose.orientation.z
            pose.pose.orientation.w = markerArray.markers[index-1].pose.orientation.w
            goal_pub.publish(pose)
            try_again = 0 

        
        elif index < len(markerArray.markers):      
            rospy.logwarn('try reach the target point '+str(index-1)+' failed! reach next point:'+'\r\n'+
                          'x:'+str(markerArray.markers[index-1].pose.position.x)+
                        ', y:'+str(markerArray.markers[index-1].pose.position.y)+
                        ', z:'+str(markerArray.markers[index-1].pose.orientation.z)+
                        ', w:'+str(markerArray.markers[index-1].pose.orientation.w)) 

            if index==count: index=0 
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = markerArray.markers[index].pose.position.x      
            pose.pose.position.y = markerArray.markers[index].pose.position.y
            pose.pose.orientation.z = markerArray.markers[index].pose.orientation.z
            pose.pose.orientation.w = markerArray.markers[index].pose.orientation.w
            goal_pub.publish(pose)
            index += 1 
            try_again = 1 


def navGoal_callback(msg):           
    global index, count

    print('Add a new target point '+str(count)+':')
    print('x:'+str(msg.pose.position.x)+
        ', y:'+str(msg.pose.position.y)+
        ', z:'+str(msg.pose.orientation.z)+
        ', w:'+str(msg.pose.orientation.w)) 

    marker = Marker()      
    marker.header.frame_id = 'map'
    marker.type = marker.ARROW
    marker.action = marker.ADD 
    marker.scale.x = 0.5 
    marker.scale.y = 0.05 
    marker.scale.z = 0.05 
    marker.color.a = 1 
    marker.color.r = 1 
    marker.color.g = 0 
    marker.color.b = 0 
    marker.pose.position.x = msg.pose.position.x 
    marker.pose.position.y = msg.pose.position.y 
    marker.pose.position.z = 0.1
    marker.pose.orientation.z = msg.pose.orientation.z 
    marker.pose.orientation.w = msg.pose.orientation.w 
    markerArray.markers.append(marker) 

    marker_number = Marker()      
    marker_number.header.frame_id = 'map' 
    marker_number.type = marker_number.TEXT_VIEW_FACING 
    marker_number.action = marker_number.ADD 
    marker_number.scale.x = 0.5 
    marker_number.scale.y = 0.5 
    marker_number.scale.z = 0.5 
    marker_number.color.a = 1 
    marker_number.color.r = 1 
    marker_number.color.g = 0 
    marker_number.color.b = 0 
    marker_number.pose.position.x = msg.pose.position.x 
    marker_number.pose.position.y = msg.pose.position.y 
    marker_number.pose.position.z = 0.1
    marker_number.pose.orientation.z = msg.pose.orientation.z 
    marker_number.pose.orientation.w = msg.pose.orientation.w 
    marker_number.text = str(count) 
    markerArray_number.markers.append(marker_number) 

    
    id = 0
    for m in markerArray.markers:   
        m.id = id
        id += 1

    for m in markerArray_number.markers:    
        m.id = id
        id += 1

    mark_pub.publish(markerArray) 
    mark_pub.publish(markerArray_number) 

    
    if count == 0:
        pose = PoseStamped() 
        pose.header.frame_id = 'map' 
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = msg.pose.position.x
        pose.pose.position.y = msg.pose.position.y
        pose.pose.orientation.z = marker.pose.orientation.z
        pose.pose.orientation.w = marker.pose.orientation.w
        goal_pub.publish(pose)
        index += 1 

    count += 1 


def click_callback(msg):           
    global index, count

    print('Add a new target point '+str(count)+':')
    print('x:'+str(msg.point.x)+
        ', y:'+str(msg.point.y)+
        ', z:0'+', w:1') 

    marker = Marker()      
    marker.header.frame_id = 'map' 
    marker.type = marker.ARROW 
    marker.action = marker.ADD 
    marker.scale.x = 0.2 
    marker.scale.y = 0.05 
    marker.scale.z = 0.05 
    marker.color.a = 1
    marker.color.r = 1 
    marker.color.g = 0 
    marker.color.b = 0 
    marker.pose.position.x = msg.point.x 
    marker.pose.position.y = msg.point.y 
    marker.pose.orientation.z = 0 
    marker.pose.orientation.w = 1 
    markerArray.markers.append(marker) 

    marker_number = Marker()      
    marker_number.header.frame_id = 'map' 
    marker_number.type = marker_number.TEXT_VIEW_FACING 
    marker_number.action = marker_number.ADD
    marker_number.scale.x = 0.5 
    marker_number.scale.y = 0.5 
    marker_number.scale.z = 0.5 
    marker_number.color.a = 1 
    marker_number.color.r = 1 
    marker_number.color.g = 0 
    marker_number.color.b = 0 
    marker_number.pose.position.x = msg.point.x 
    marker_number.pose.position.y = msg.point.y 
    marker_number.pose.position.z = 0.1 
    marker_number.pose.orientation.z = 0 
    marker_number.pose.orientation.w = 1 
    marker_number.text = str(count) 
    markerArray_number.markers.append(marker_number) 

    
    id = 0
    for m in markerArray.markers:    
        m.id = id
        id += 1

    for m in markerArray_number.markers:    
        m.id = id
        id += 1
    
    mark_pub.publish(markerArray) 
    mark_pub.publish(markerArray_number) 

    
    if count == 0:
        pose = PoseStamped() 
        pose.header.frame_id = 'map' 
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = msg.point.x 
        pose.pose.position.y = msg.point.y 
        pose.pose.orientation.z = 0 
        pose.pose.orientation.w = 1 
        goal_pub.publish(pose)
        index += 1 

    count += 1 


def getKey():
    fd = sys.stdin.fileno()
    new_settings = termios.tcgetattr(fd)
    new_settings[3]=new_settings[3] | termios.ECHO
    try:
        # termios.tcsetattr(fd, termios.TCSADRAIN, new_settings)
        # tty.setraw(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, new_settings)
    return key

def send_mark():
    global markerArray, markerArray_number, count, index, try_again, mark_pub, goal_pub
    markerArray = MarkerArray() 
    markerArray_number = MarkerArray() 
    count = 0
    index = 0
    try_again = 1
    sendflagPublisher = rospy.Publisher('/send_flag', Int8, queue_size =1)
    rospy.init_node('path_point_demo') 
    mark_pub    = rospy.Publisher('/path_point', MarkerArray, queue_size = 100) 
    navGoal_sub = rospy.Subscriber('/send_mark_goal', PoseStamped, navGoal_callback) 
    click_sub   = rospy.Subscriber('/clicked_point', PointStamped, click_callback) 
    goal_pub    = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 1) 
    send_flag=Int8()
    send_flag.data=1
    sendflagPublisher.publish(send_flag)
    rospy.sleep(1.)
    sendflagPublisher.publish(send_flag)
    rospy.loginfo('a=%d',send_flag.data)
    print("11111111111111111111111111")
    goal_status_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, pose_callback) 


    while not rospy.is_shutdown():
        key = getKey() 
        if(key=='c'): 
            count = 0
            index = 0
            try_again = 1

            markerArray = MarkerArray() 
            marker = Marker()
            marker.header.frame_id = 'map' 
            marker.type = marker.TEXT_VIEW_FACING 
            marker.action = marker.DELETEALL 
            marker.text = '' 
            markerArray.markers.append(marker) 

            for m in markerArray_number.markers:    
                m.action = marker.DELETEALL

            mark_pub.publish(markerArray) 
            mark_pub.publish(markerArray_number) 
            markerArray = MarkerArray() 
            markerArray_number = MarkerArray() 

        elif (key == '\x03'): 
            break
def breakkey():
    fd = sys.stdin.fileno()
    new_settings = termios.tcgetattr(fd)
    new_settings[3]=new_settings[3] | termios.ECHO
    termios.tcsetattr(fd, termios.TCSADRAIN, new_settings)

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin) 
    rospy.on_shutdown(breakkey)
    send_mark()

