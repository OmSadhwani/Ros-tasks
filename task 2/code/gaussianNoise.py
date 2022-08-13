#!/usr/bin/env python
# Software License Agreement (BSD License)



import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from numpy import random

w=Twist()



def callback(data):
    g = random.normal(size=(2, 3))
    w.linear.x= data.linear.x+ g[0][0]
    w.linear.y= data.linear.y+g[0][1]
    w.linear.z= data.linear.z+g[0][2]
    w.angular.x=data.angular.x+g[1][0]
    w.angular.y=data.angular.y+g[1][1]
    w.angular.z=data.angular.z+g[1][2]
    
    rospy.loginfo("subscriber.linear.x = %s", w.linear.x)
    
def gauss():

    
    pub = rospy.Publisher('noise_add', Twist , queue_size=10)
    rospy.init_node('gauss', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    rospy.Subscriber('/cmd_vel', Twist, callback)
    
    while not rospy.is_shutdown():
        
        
        pub.publish(w)
        rospy.loginfo("publisher.linear.x = %s" , w.linear.x)
        rate.sleep()
    

    
    rospy.spin()

if __name__ == '__main__':
	
    gauss()
