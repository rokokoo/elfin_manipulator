#! /usr/bin/python3

from geometry_msgs import msg
import rospy, tf
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetLinkState
from geometry_msgs.msg import *
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
import tf2_geometry_msgs
from tf.transformations import *
import tf2_msgs.msg
from object_msgs.srv import ObjectInfo
import image_proc
import tf_conversions
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

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


def create_bb_points(sizes):
    size = np.zeros((3))
    size[0] = sizes[0]/2
    size[1] = sizes[1]/2
    size[2] = sizes[2]/2
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

def p3d_to_p2d_bb(p3d_bb):
    min_x = np.amin(p3d_bb[:,0])
    min_y = np.amin(p3d_bb[:,1])
    max_x = np.amax(p3d_bb[:,0])
    max_y = np.amax(p3d_bb[:,1])
    p2d_bb = np.array([[min_x,min_y] , [max_x,max_y]])
    return p2d_bb

def capture(point,count):
    msg = rospy.wait_for_message("/realsense_plugin/camera_depth/image_raw", Image)
    try:
        cv2_img = bridge.imgmsg_to_cv2(msg, "passthrough")
    except CvBridgeError as e:
        print(e)
    else:
        cv2.normalize(cv2_img, cv2_img, 0, 65534, cv2.NORM_MINMAX)
        cv2.imwrite('dataset/camera_image'+ str(count)+'.png', cv2_img)
        path = 'dataset/'
        im = Image2.open("dataset/camera_image"+str(count)+".png")
        im_data = np.asarray(im)
        image = Image2.fromarray(im_data, 'I')
        img_draw = ImageDraw.Draw(image)  
        for uv in point:
            img_draw.point(uv,fill=0)
        filename = path + '%06d.png' % count
        if not os.path.exists(os.path.dirname(filename)):
            os.makedirs(os.path.dirname(filename))
        uv_array = np.array(point)
        box = p3d_to_p2d_bb(uv_array)
        datastr = ''
        uc = ((box[0,0] + box[1,0])/2) / camera_model.width
        vc = ((box[0,1] + box[1,1])/2) / camera_model.height
        w = (box[1,0] - box[0,0]) / camera_model.width
        h = (box[1,1] - box[0,1]) / camera_model.height
        datastr = datastr + f"0 {uc} {vc} {w} {h} \n"
        with open(path + '%06d.txt' % count, 'w') as txt_file:
            txt_file.write(datastr)
        image.save(filename)

def transform_pose(input_pose, from_frame, to_frame):
    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose.pose
    pose_stamped.header.frame_id = from_frame
    pose_stamped.header.stamp = rospy.Time(0)
    try:
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))  
        return output_pose_stamped.pose
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise

def point_distance(p1_coords,p2_coords):
    p1 = np.array([p1_coords[0], p1_coords[1], p1_coords[2]])
    p2 = np.array([p2_coords[0], p2_coords[1], p2_coords[2]])
    squared_dist = np.sum((p1-p2)**2, axis=0)
    return np.sqrt(squared_dist)

def create_posestamped(xyz, rpy):
    msg = tf2_geometry_msgs.PoseStamped()
    orient = tf.transformations.quaternion_from_euler(rpy[0],rpy[1],rpy[2])
    msg.pose.orientation.x = orient[0]
    msg.pose.orientation.y = orient[1]
    msg.pose.orientation.z = orient[2]
    msg.pose.orientation.w = orient[3]
    msg.pose.position.x = xyz[0]
    msg.pose.position.y = xyz[1]
    msg.pose.position.z = xyz[2]
    return msg

def side_points(sizes, from_frame, to_frame, r_p_y, camera_coords):
    size = np.zeros((3))
    size[0] = sizes[0]/2
    size[1] = sizes[1]/2
    size[2] = sizes[2]/2
    cords = np.zeros((6, 3))
    cords[0, :] = np.array([size[0], 0, 0])
    cords[1, :] = np.array([-size[0], 0, 0])
    cords[2, :] = np.array([0, size[1], 0])
    cords[3, :] = np.array([0, -size[1], 0])
    cords[4, :] = np.array([0, 0, size[2]])
    cords[5, :] = np.array([0, 0, -size[2]])
    cords_offset = np.zeros((6, 3))
    cords_offset[0, :] = np.array([size[0]+0.0001, 0, 0])
    cords_offset[1, :] = np.array([-size[0]-0.0001, 0, 0])
    cords_offset[2, :] = np.array([0, size[1]+0.0001, 0])
    cords_offset[3, :] = np.array([0, -size[1]-0.0001, 0])
    cords_offset[4, :] = np.array([0, 0, size[2]+0.0001])
    cords_offset[5, :] = np.array([0, 0, -size[2]-0.0001])
    used_sides = [False,False,False,False,False,False] #0=+x 1=-x 2=+y 3=-y 4=+z 5=-z
    count = 0
    for real, offset in zip(cords, cords_offset):
        center_pose = create_posestamped(real,r_p_y)
        offset_pose = create_posestamped(offset,r_p_y)
        real_pose_tf = transform_pose(center_pose, from_frame, to_frame)
        offset_pose_tf = transform_pose(offset_pose, from_frame, to_frame)
        real_dist = point_distance(camera_coords,[real_pose_tf.position.x,real_pose_tf.position.y,real_pose_tf.position.z])
        offset_dist = point_distance(camera_coords,[offset_pose_tf.position.x,offset_pose_tf.position.y,offset_pose_tf.position.z])
        if real_dist > offset_dist:
            used_sides[count] = True
            print(count)
        count += 1
    sidecheck = np.zeros((3,9,3))
    if used_sides[0]:
        sidecheck[0,0:9,0] = size[0]
        sidecheck[0,[0,1,2],2] = size[2]/2
        sidecheck[0,[6,7,8],2] = -size[2]/2 
        sidecheck[0,[0,3,6],1] = size[1]/2
        sidecheck[0,[2,5,8],1] = -size[1]/2 
    if used_sides[1]:
        sidecheck[0,0:9,0] = -size[0]
        sidecheck[0,[0,1,2],2] = size[2]/2
        sidecheck[0,[6,7,8],2] = -size[2]/2 
        sidecheck[0,[0,3,6],1] = size[1]/2
        sidecheck[0,[2,5,8],1] = -size[1]/2 
    if used_sides[2]:
        sidecheck[1,0:9,1] = size[1]
        sidecheck[1,[0,1,2],2] = size[2]/2
        sidecheck[1,[6,7,8],2] = -size[2]/2 
        sidecheck[1,[0,3,6],0] = size[0]/2
        sidecheck[1,[2,5,8],0] = -size[0]/2
    if used_sides[3]:
        sidecheck[1,0:9,1] = -size[1]
        sidecheck[1,[0,1,2],2] = size[2]/2
        sidecheck[1,[6,7,8],2] = -size[2]/2 
        sidecheck[1,[0,3,6],0] = size[0]/2
        sidecheck[1,[2,5,8],0] = -size[0]/2
    if used_sides[4]:
        sidecheck[2,0:9,2] = size[2]
        sidecheck[2,[0,1,2],0] = size[0]/2
        sidecheck[2,[6,7,8],0] = -size[0]/2 
        sidecheck[2,[0,3,6],1] = size[1]/2
        sidecheck[2,[2,5,8],1] = -size[1]/2
    if used_sides[5]:
        sidecheck[2,0:9,2] = -size[2]
        sidecheck[2,[0,1,2],0] = size[0]/2
        sidecheck[2,[6,7,8],0] = -size[0]/2 
        sidecheck[2,[0,3,6],1] = size[1]/2
        sidecheck[2,[2,5,8],1] = -size[1]/2

    return sidecheck

bridge = CvBridge()

if __name__ == '__main__':
    rospy.init_node('model_spawner')
    print("Waiting for gazebo services...")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    print("Got it.")
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    srv_delete_model = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    state_msg = ModelState()
    camera_model = image_geometry.PinholeCameraModel()
    camera_model.fromCameraInfo(rospy.wait_for_message('/realsense_plugin/camera_depth/camera_info', CameraInfo))

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    with open("src/elfin_manipulator/samk/automatic_annotator/models/box.urdf", "r") as f:
        model = f.read()

    model_name = "model_1"
    state_msg.model_name = model_name
    model_pose = Pose()
    model_pose.position.x = 0
    model_pose.position.y = 0
    model_pose.position.z = 0
    orient = tf.transformations.quaternion_from_euler(0,0,0)
    model_pose.orientation.x = orient[0]
    model_pose.orientation.y = orient[1]
    model_pose.orientation.z = orient[2]
    model_pose.orientation.w = orient[3]
    spawn_model(model_name, model, '', model_pose, "world")
    print("Model spawned")
    count = 0
    while count < 100:
        count = count + 1
        state_msg.pose.position.x = 0.2 + count/10
        state_msg.pose.position.y = 0.3
        state_msg.pose.position.z = 0
        r_p_y = [0,0,0]
        orient2 = tf.transformations.quaternion_from_euler(r_p_y[0],r_p_y[1],r_p_y[2],)
        state_msg.pose.orientation.x = orient2[0]
        state_msg.pose.orientation.y = orient2[1]
        state_msg.pose.orientation.z = orient2[2]
        state_msg.pose.orientation.w = orient2[3]
        set_state(state_msg)
        rospy.sleep(0.3)
        model_sizes = model_info(model_name)
        bbox_points = create_bb_points(model_sizes)
        point_list = []
        side_points(model_sizes,model_name,'world',r_p_y,[1.5,0,1.5])
        
        for bbox_corner in bbox_points:
            corner_pose = create_posestamped(bbox_corner,r_p_y)

            model_to_sensor = transform_pose(corner_pose, model_name, 'camera_frame')
            # model_to_world = transform_pose(corner_pose, model_name, 'world')
            point_list.append(camera_model.project3dToPixel((-model_to_sensor.position.y,-model_to_sensor.position.z,model_to_sensor.position.x)))
        capture(point_list,count)
        print(count)
        rospy.sleep(0.3)