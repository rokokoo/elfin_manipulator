#! /usr/bin/python3
import rospy
from geometry_msgs.msg import *
import tf2_geometry_msgs
import tf2_ros
import tf_conversions
from gazebo_msgs.srv import GetModelState


#pub = rospy.Publisher('model_frame', geometry_msgs.msg.TransformStamped, queue_size=10)
rospy.init_node('model_tf')
r = rospy.Rate(60) # 10hz

br = tf2_ros.TransformBroadcaster()
t = geometry_msgs.msg.TransformStamped()
def callback(msg,name):
   t.header.stamp = rospy.Time.now()
   t.header.frame_id = "world"
   t.child_frame_id = name
   t.transform.translation.x = msg.pose.position.x
   t.transform.translation.y = msg.pose.position.y
   t.transform.translation.z = msg.pose.position.z
   t.transform.rotation.x = msg.pose.orientation.x
   t.transform.rotation.y = msg.pose.orientation.y
   t.transform.rotation.z = msg.pose.orientation.z
   t.transform.rotation.w = msg.pose.orientation.w
   br.sendTransform(t)
   #pub.publish(t)


while not rospy.is_shutdown():
   try:
      model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
      response = model_coordinates('model_1','world')
      callback(response,'model_1')
      # response = model_coordinates('model_2','world')
      # callback(response,'model_2')
      r.sleep()
   except:
      pass