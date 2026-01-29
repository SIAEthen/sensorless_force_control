#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry

def callback(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z), 
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     '/girona1000/base_link',
                     '/world_ned')
    
def listener():
    rospy.init_node("odom_to_tf", anonymous=True)
    rospy.Subscriber("/girona1000/dynamics/odometry", Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()