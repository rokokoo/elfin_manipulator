#! /usr/bin/python3

import rospy, tf, random
from random import randint, uniform
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetLinkState
from geometry_msgs.msg import *
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from PIL import Image as Image2
from PIL import ImageDraw
import os
import image_geometry
from sensor_msgs.msg import CameraInfo
import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
import matplotlib.pyplot as plt
from tf.transformations import *
import tf2_msgs.msg
from object_msgs.srv import ObjectInfo
import sys, roslib, rospy, rosservice
import image_proc
import tf_conversions

bridge = CvBridge()

def model_info(model):
    rospy.wait_for_service("/gazebo_objects/get_info")
    try:
        getInfo = rospy.ServiceProxy("/gazebo_objects/get_info", ObjectInfo)
        resp = getInfo(name=model, get_geometry=True)
        if resp.success:
            primitives = resp.object.primitives
            type = primitives[0].type
            if type == 1:
                typeMsg = "Box"
            elif type == 2:
                typeMsg = "Sphere"
            elif type == 3:
                typeMsg = "Cylinder"
            elif type == 4:
                typeMsg = "Cone"
            else:
                typeMsg = "UNKNOWN"

            dimensions = primitives[0].dimensions
            dimX = dimensions[0]
            dimY = dimensions[1]
            dimZ = dimensions[2]

            print(
                "Type: %s\nDim X: %f\nDim Y: %f\nDim Z: %f"
                % (typeMsg, dimX, dimY, dimZ)
            )
            return np.array([dimX,dimY,dimZ])
        else:
            print("Model not found")
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def transform_pose(input_pose, from_frame, to_frame, model_pose=None):
    # **Assuming /tf2 topic is being broadcasted
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    # if model_pose is not None:
    #     rospy.sleep(0.15)
    #     broadcast(model_pose,from_frame)
    rospy.sleep(0.15)
    # br.sendTransform((Position.position.x,Position.position.y,Position.position.z),
    #         (Position.orientation.x,Position.orientation.y,Position.orientation.z,Position.orientation.w),
    #         rospy.Time.now(),
    #         model_name,
    #         "world")
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = "world"
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = from_frame
    t.transform.translation.x = model_pose.position.x
    t.transform.translation.y = model_pose.position.y
    t.transform.translation.z = model_pose.position.z 

    t.transform.rotation.x = model_pose.orientation.x
    t.transform.rotation.y = model_pose.orientation.y
    t.transform.rotation.z = model_pose.orientation.z
    t.transform.rotation.w = model_pose.orientation.w
    br.sendTransform(t)
    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    pose_stamped.header.stamp = rospy.Time.now()
    #pose_stamped
    #print(pose_stamped)
    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        #broadcast(t,from_frame)
        rospy.sleep(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
        #print(output_pose_stamped)
        #return output_pose_stamped.pose
        return output_pose_stamped

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise

def broadcast(msg,model_name):
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = "world"
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = model_name
    print(msg)
    t.transform.translation.x = msg.transform.translation.x #msg.position.x
    t.transform.translation.y = msg.transform.translation.y #msg.position.y
    t.transform.translation.z = msg.transform.translation.z   #msg.position.z 

    t.transform.rotation.y = msg.transform.rotation.x #msg.orientation.y
    t.transform.rotation.x = msg.transform.rotation.y #msg.orientation.x
    t.transform.rotation.z = msg.transform.rotation.z #msg.orientation.z
    t.transform.rotation.w = msg.transform.rotation.w #msg.orientation.w

    tfm = tf2_msgs.msg.TFMessage([t])
    pub_tf.publish(tfm)
    #return rospy.Time.now()

def create_bb_points(size):
    size[0] = size[0]/2
    size[1] = size[1]/2
    size[2] = size[2]/2
    cords = np.zeros((8, 3))
    cords[0, :] = np.array([size[0], size[1], -size[2]])
    cords[1, :] = np.array([-size[0], size[1], -size[2]])
    cords[2, :] = np.array([-size[0], -size[1], -size[2]])
    cords[3, :] = np.array([size[0], -size[1], -size[2]])
    cords[4, :] = np.array([size[0], size[1], size[2]])
    cords[5, :] = np.array([-size[0], size[1], size[2]])
    cords[6, :] = np.array([-size[0], -size[1], size[2]])
    cords[7, :] = np.array([size[0], -size[1], size[2]])
    return cords


def save_output(img, bboxes,count):
    path = 'dataset/'
    im = Image2.open("dataset/"+img)
    im_data = np.asarray(im)
    image = Image2.fromarray(im_data, 'RGB')
    img_draw = ImageDraw.Draw(image)  
    for x in bboxes:
        u1 = int(x[0])
        v1 = int(x[1])
        u2 = int(x[0])
        v2 = int(x[1])
        img_draw.point([u1,v1],fill=0)
    filename = path + '%06d.png' % count
    if not os.path.exists(os.path.dirname(filename)):
        os.makedirs(os.path.dirname(filename))
    image.save(filename)

def capture():
    # msg = rospy.wait_for_message("/camera_ir/color/image_raw", Image)
    msg = rospy.wait_for_message("/camera_ir/color/image_raw", Image)
    print("Received an image!")
    try:
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")

    except CvBridgeError as e:
        print(e)
    else:
        if not os.path.exists(os.path.dirname('dataset/')):
            os.makedirs(os.path.dirname('dataset/'))
        cv2.imwrite('dataset/camera_image'+ str(count)+'.jpeg', cv2_img)
    print("Capture() finished")

def handle_turtle_pose(msg, turtlename):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = turtlename
    t.transform.translation.x = 2.5
    t.transform.translation.y = 0
    t.transform.translation.z = 2.5
    q = tf_conversions.transformations.quaternion_from_euler(0, 1, 3.14)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)


if __name__ == '__main__':
    rospy.init_node('model_spawner')
    print("Waiting for gazebo services...")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    print("Got it.")

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    srv_delete_model = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    rospy.wait_for_service("gazebo/get_link_state")
    model_info_prox = rospy.ServiceProxy('gazebo/get_link_state', GetLinkState) 
    camera_model = image_geometry.PinholeCameraModel()
    # camera_model.fromCameraInfo(rospy.wait_for_message('/camera_ir/camera_info', CameraInfo))
    camera_model.fromCameraInfo(rospy.wait_for_message('/camera_ir/camera_info', CameraInfo))

    br = tf2_ros.TransformBroadcaster()

    pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

    with open("src/elfin_manipulator/samk/automatic_annotator/models/box.urdf", "r") as f:
        box = f.read()
    count = 0
    while count < 100:
        count = count + 1
        Position = Pose()
        orient = tf.transformations.quaternion_from_euler(0,0,0) #+ random.uniform(-1,1) )
        Position.position.x = 0.15 + count/10#+ random.uniform(0.1,1)
        Position.position.y = 0 - count/10#+ random.uniform(-1,1)
        Position.position.z = 0 #+count/10
        Position.orientation.x = orient[0]
        Position.orientation.y = orient[1]
        Position.orientation.z = orient[2]
        Position.orientation.w = orient[3]
        pose_0 = Pose()

        model_name = "model_" + str(count)
        print("Spawning model:", model_name)
        spawn_model(model_name, box, '', Position, "world")
        #print(model_info_prox(model_name+"::link",''))
        rospy.sleep(1)
        model_sizes = model_info(model_name)#np.array([0.05,0.05,0.05])
        bbox_points = create_bb_points(model_sizes)
        x = []
        for bbox_corner in bbox_points:
            #rospy.Subscriber('/%s/pose' % turtlename, geometry_msgs.msg.TransformStamped(),handle_turtle_pose,model_name)
            pose_0.position.x = 0 + bbox_corner[0]
            pose_0.position.y = 0 + bbox_corner[1]
            pose_0.position.z = 0 + bbox_corner[2]
            pose_0.orientation.x = Position.orientation.x
            pose_0.orientation.y = Position.orientation.y
            pose_0.orientation.z = Position.orientation.z
            pose_0.orientation.w = Position.orientation.w

            # br.sendTransform((Position.position.x,Position.position.y,Position.position.z),
            #             (Position.orientation.x,Position.orientation.y,Position.orientation.z,Position.orientation.w),
            #             rospy.Time.now(),
            #             model_name,
            #             "world")
            # world_to_sensor = transform_pose(pose_0 ,model_name,"camera_depth_optical_frame")
            #broadcast(Position,model_name)
            world_to_sensor = transform_pose(pose_0 ,model_name,"camera_color_optical_frame",Position)

            x.append(camera_model.project3dToPixel((-world_to_sensor.pose.position.y,-world_to_sensor.pose.position.z,world_to_sensor.pose.position.x)))
        capture()
        save_output('camera_image'+str(count)+'.jpeg',x,count)
        print("Image saved")
        srv_delete_model(model_name)
        rospy.sleep(0.01)

