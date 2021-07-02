#! /usr/bin/python3
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
from object_msgs.srv import ObjectInfo
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
            # print(
            #     "Type: %s\nDim X: %f\nDim Y: %f\nDim Z: %f"
            #     % (typeMsg, dimX, dimY, dimZ)
            # )
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

def model_pose_msg(xyz,rpy):
    model_pose = Pose()
    model_pose.position.x = xyz[0]
    model_pose.position.y = xyz[1]
    model_pose.position.z = xyz[2]
    orient = tf.transformations.quaternion_from_euler(rpy[0],rpy[1],rpy[2])
    model_pose.orientation.x = orient[0]
    model_pose.orientation.y = orient[1]
    model_pose.orientation.z = orient[2]
    model_pose.orientation.w = orient[3]
    return model_pose

def make_state_msg(xyz,rpy,name):
    state_msg = ModelState()
    state_msg.model_name = name
    state_msg.pose.position.x = xyz[0]
    state_msg.pose.position.y = xyz[1]
    state_msg.pose.position.z = xyz[2]
    orient2 = tf.transformations.quaternion_from_euler(rpy[0],rpy[1],rpy[2])
    state_msg.pose.orientation.x = orient2[0]
    state_msg.pose.orientation.y = orient2[1]
    state_msg.pose.orientation.z = orient2[2]
    state_msg.pose.orientation.w = orient2[3]
    return state_msg

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




class AnnotatorCamera():
    def __init__(self):
        self.spawn_model = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.model_name = []
        self.model_sizes = []
        self.model_rpy = [] #model_rpy and xyz don't have real use atm
        # self.model_xyz = []
        self.model_class_num = []
        self.initial_pose = [0,0,0]
        self.model_spawn_pose = model_pose_msg(self.initial_pose,self.initial_pose)
        self.cameras = []

    def spawn_new_model(self, model_path, model_name,model_class_num):
        self.model_name.append(model_name)
        with open(model_path, "r") as f:
            model = f.read()
        self.spawn_model(model_name, model, '', self.model_spawn_pose, 'world')
        self.model_sizes.append(model_info(model_name))
        self.model_rpy.append(self.initial_pose)
        # self.model_xyz.append(self.initial_pose)
        self.model_class_num.append(model_class_num)

    def spawn_object(self, model_path, model_name):
        with open(model_path, "r") as f:
            model = f.read()
        self.spawn_model(model_name, model, '', model_pose_msg([25,25,25],[0,0,0]), 'world')


    def new_state(self,xyz,rpy,model_name):
        state_msg = make_state_msg(xyz,rpy,model_name)
        self.set_state(state_msg)

    def add_camera(self, camera_name, camera_coords):
        camera = NewCamera(camera_name,camera_coords,self)
        self.cameras.append(camera)
        return camera

class NewCamera(AnnotatorCamera):
    def __init__(self, camera_name, camera_coords, parent):
        super().__init__()
        self.camera_name = camera_name
        self.camera_coords = camera_coords
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(rospy.wait_for_message('/realsense_plugin/'+self.camera_name+'_depth/camera_info', CameraInfo))
        self.model_name = parent.model_name
        self.model_sizes = parent.model_sizes
        self.model_rpy = parent.model_rpy #model_rpy and xyz don't have real use atm
        # self.model_xyz = parent.model_xyz
        self.model_class_num = parent.model_class_num

    def distance_to_camera(self,pixel,cv2_img):
        depth = cv2_img[int(pixel[1])][int(pixel[0])]/1000
        ray = self.camera_model.projectPixelTo3dRay((pixel[0],pixel[1]))  # get 3d ray of unit length through desired pixel
        ray_z = [el / ray[2] for el in ray]
        pt = [el * depth for el in ray_z]
        return point_distance([0,0,0],pt)

    def model_corner_points(self, model_name,model_sizes,camera_name,rpy):
        bbox_points = create_bb_points(model_sizes)
        point_list = []
        for bbox_corner in bbox_points:
            corner_pose = create_posestamped(bbox_corner,rpy)
            model_to_sensor = transform_pose(corner_pose, model_name, camera_name+'_frame')
            point_list.append(self.camera_model.project3dToPixel((-model_to_sensor.position.y,-model_to_sensor.position.z,model_to_sensor.position.x)))
        return point_list

    def get_points(self,model_name):
        model_points = []
        name = self.model_name.index(model_name)
        occlusion = []
        occlusion_list = []
        for model_name in self.model_name:
            helper_points = side_points(self.model_sizes[name],model_name,'world',self.model_rpy[name],self.camera_coords)
            for sides in helper_points:
                occlusion_side = []
                for occ_points in sides:
                    if np.all(occ_points == 0): continue
                    corner_pose = create_posestamped(occ_points,self.model_rpy[name])
                    model_to_sensor = transform_pose(corner_pose, model_name, self.camera_name+'_frame')
                    occlusion_side.append([self.camera_model.project3dToPixel((-model_to_sensor.position.y,-model_to_sensor.position.z,model_to_sensor.position.x)),point_distance([0,0,0],[model_to_sensor.position.x,model_to_sensor.position.y,model_to_sensor.position.z,])])
                occlusion.append(occlusion_side)
            for uv_dist in occlusion:
                for uv in uv_dist:
                    # try:
                    #     print(self.cv2_img[int(uv[0][1]),int(uv[0][0])],math.trunc( uv[1]*1000))
                    #     print(self.distance_to_camera(uv[0],self.cv2_img))
                    #     print("___")
                    # except: pass
                    if uv[0][0] < 0 or uv[0][1] < 0 or uv[0][0] > self.cv2_img.shape[1] or uv[0][1] > self.cv2_img.shape[0]: continue
                    if self.distance_to_camera(uv[0],self.cv2_img)+0.005 > uv[1]: occlusion_list.append(1)

            if sum(occlusion_list) >= 3: model_points.append([model_name,self.model_corner_points(model_name,self.model_sizes[name],self.camera_name,self.model_rpy[name])])
            # print(occlusion)
            occlusion = []
            # print(occlusion_list)
            occlusion_list = []
        return model_points

    def get_image(self):
        msg = rospy.wait_for_message('/realsense_plugin/'+self.camera_name+'_depth/image_raw', Image)
        # pc_msg = rospy.wait_for_message('/realsense_plugin/'+self.camera_name+'_depth/points', PointCloud2)
        try:
            self.cv2_img = bridge.imgmsg_to_cv2(msg, "passthrough")
        except CvBridgeError as e:
            print(e)
        else:
            count = 0
            new_image = cv2.normalize(self.cv2_img, None, 0, 65535, cv2.NORM_MINMAX)
            path = 'dataset/'+self.camera_name
            while os.path.exists(path+'_image'+ str(count)+'.png'):
                count += 1
            cv2.imwrite('dataset/'+self.camera_name+'_image'+ str(count)+'.png', new_image)
            model_points = []

            # im = Image2.open("dataset/"+self.camera_name+"_image"+str(count)+".png")
            # im_data = np.asarray(im)
            # image = Image2.fromarray(im_data, 'I')
            # img_draw = ImageDraw.Draw(image)

            for model_name in self.model_name:
                model_points = self.get_points(model_name)

            datastr = ''
            for points in model_points:
                name = self.model_name.index(points[0])
                if self.model_class_num[name] == 20: continue
                uv_array = np.array(points[1])
                box = p3d_to_p2d_bb(uv_array)
                uc = ((box[0,0] + box[1,0])/2) / self.camera_model.width
                vc = ((box[0,1] + box[1,1])/2) / self.camera_model.height
                w = (box[1,0] - box[0,0]) / self.camera_model.width
                h = (box[1,1] - box[0,1]) / self.camera_model.height
                datastr = datastr + f"{self.model_class_num[name]} {uc} {vc} {w} {h} \n"
            with open(path+'_image'+str(count)+'.txt', 'a') as txt_file:
                txt_file.write(datastr)


bridge = CvBridge()

if __name__ == '__main__':
    rospy.init_node('model_spawner')
    print("Waiting for gazebo services...")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    print("Got it.")

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    z = AnnotatorCamera()
    z.spawn_new_model("src/elfin_manipulator/samk/automatic_annotator/models/box.urdf","model_1",0)
    z.spawn_new_model("src/elfin_manipulator/samk/automatic_annotator/models/box.urdf","model_2",0)
    z.spawn_new_model("src/elfin_manipulator/samk/automatic_annotator/models/box.urdf","model_3",0)
    z.spawn_new_model("src/elfin_manipulator/samk/automatic_annotator/models/box.urdf","model_4",0)
    z.spawn_object("src/elfin_manipulator/samk/automatic_annotator/models/cylinder.urdf","model_5")


    a = z.add_camera('camera',[1.5,0,1.5])
    b=z.add_camera('camera2',[0.8, 0, 1.5])
    c=z.add_camera('camera3',[0.6, 1.5, 0.3])
    d=z.add_camera('camera4',[0.6, -1.5, 0.3])
    e=z.add_camera('camera5',[1.4, 1.4, 1.5])
    f=z.add_camera('camera6',[1.5, -1.3, 1])
    g=z.add_camera('camera7',[-1, -1, 1])
    h=z.add_camera('camera8',[2.5, 0, 0.5])
    i=z.add_camera('camera9',[0, -0.3, 1.7])
    j=z.add_camera('camera10',[-1, 1, 0.7])
    z.new_state([0.1,0.8,0],[2,0,2],"model_1")
    z.new_state([0.1,-0.8,0],[3,0,1],"model_2")
    z.new_state([1,0.8,0],[1,0,3],"model_3")
    z.new_state([1,-0.8,0],[0,2,0],"model_4")
    z.new_state([1,0.4,0],[0,2,0],"model_5")

    rospy.sleep(0.3)
    a.get_image()
    b.get_image()
    c.get_image()
    d.get_image()
    e.get_image()
    f.get_image()
    g.get_image()
    h.get_image()
    i.get_image()
    j.get_image()
