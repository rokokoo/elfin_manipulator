#! /usr/bin/python3
import rospy
from geometry_msgs.msg import *
import tf2_geometry_msgs
import tf2_ros
import tf_conversions
from gazebo_msgs.srv import GetModelState


#pub = rospy.Publisher('model_frame', geometry_msgs.msg.TransformStamped, queue_size=10)
rospy.init_node('model_tf')
r = rospy.Rate(60) 

br = tf2_ros.TransformBroadcaster()
t = geometry_msgs.msg.TransformStamped()
def pose_publisher(msg,name):
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
      pose_publisher(response,'model_1')
      response = model_coordinates('model_2','world')
      pose_publisher(response,'model_2')
      response = model_coordinates('model_3','world')
      pose_publisher(response,'model_3')
      response = model_coordinates('model_4','world')
      pose_publisher(response,'model_4')
      # response = model_coordinates('model_5','world')
      # pose_publisher(response,'model_5')
      # response = model_coordinates('model_6','world')
      # pose_publisher(response,'model_6')
      # response = model_coordinates('model_7','world')
      # pose_publisher(response,'model_7')
      # response = model_coordinates('model_8','world')
      # pose_publisher(response,'model_8')
      # response = model_coordinates('model_9','world')
      # pose_publisher(response,'model_9')
      # response = model_coordinates('model_10','world')
      # pose_publisher(response,'model_10')
      r.sleep()
   except:
      pass