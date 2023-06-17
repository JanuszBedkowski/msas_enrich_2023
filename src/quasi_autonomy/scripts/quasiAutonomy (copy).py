#!/usr/bin/env python
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
listener = None
targetPose = None
goingBackward = False
tf_prefix =""
def callback_joy(data):
    global targetPose
    if (data.buttons[0] or data.buttons[1] or data.buttons[2] or data.buttons[3]):
	print "stop!"
	targetPose = None


def callback_simple_goal(data):
    global targetPose
    global goingBackward
    data.header.stamp = rospy.Time();
    targetPose = listener.transformPose(tf_prefix+"/odom",data)
    poseIncrement = listener.transformPose(tf_prefix+"/base_link",targetPose)
    trans = poseIncrement.pose.position
    heading = math.atan2(trans.y, trans.x)
    ang=  (heading*180.0/math.pi)
    if (abs(ang)>90):
        goingBackward = True
    else:
        goingBackward = False

if __name__ == '__main__':
    global listener
    global tf_prefix
    rospy.init_node('turtle_tf_listener')

    listener = tf.TransformListener()
    tf_prefix = rospy.get_param('tf_prefix', "")
    print ("tf_prefix : "+tf_prefix)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback_simple_goal)
    rospy.Subscriber("/joy", Joy, callback_joy)
    rospy.Subscriber("/joy1", Joy, callback_joy)
    rospy.Subscriber("/joy2", Joy, callback_joy)


    vel_pub = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist,queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            global targetPose
            if targetPose is not None:
                poseIncrement = listener.transformPose(tf_prefix+"/base_link",targetPose)

                trans = poseIncrement.pose.position
                heading = math.atan2(trans.y, trans.x)
                distance = math.sqrt(trans.x ** 2 + trans.y ** 2)

                linear = 0
                linear_abs = 0
                if (distance > 1):
                    linear_abs = 0.3
                if (distance < 1 ):
                    linear_abs = 0.15




                if goingBackward:
                    heading = math.atan2(-trans.y, -trans.x)
                    angular = 0.6 * heading
                    if (heading < 0.15):
                        linear = -linear_abs
                if not goingBackward:
                    heading = math.atan2(trans.y, trans.x)
                    angular = 0.6 * heading
                    if (heading < 0.15):
                        linear = linear_abs

                if angular > 0.5:
                    angular = 0.5
                if angular < -0.5:
                    angular = -0.5
                ang=  (heading*180.0/math.pi)
                if goingBackward:
                    print ("gb", ang ,distance)
                else:
                    print ( ang ,distance)
                cmd = geometry_msgs.msg.Twist()
                cmd.linear.x = linear
                cmd.angular.z = angular
                vel_pub.publish(cmd)
                if (distance < 0.5):
                    targetPose = None
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        #print (trans, rot)

        #
        # angular = 4 * math.atan2(trans[1], trans[0])
        # linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        # cmd = geometry_msgs.msg.Twist()
        # cmd.linear.x = linear
        # cmd.angular.z = angular
        # vel_pub.publish(cmd)

        rate.sleep()
