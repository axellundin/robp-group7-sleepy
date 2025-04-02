import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped,Point,PointStamped
import rclpy.time
from sensor_msgs.msg import Image 
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from builtin_interfaces.msg import Time
from arm_control.arm_camera_processing import BoxPositionConverter
from arm_control.box_position_pub import quaternion_to_matrix

import time
from cv_bridge import CvBridge  
import cv2  
import torch
import numpy as np
from ultralytics import YOLO

from core_interfaces.srv import YoloImageDetect
from core_interfaces.msg import YoloClassifiedObject
from std_msgs.msg import Int32
from perception.cam_image_point_convert import from_2d_to_3d


time_it_enable = True

def time_it(func):
    """ 
    calculate the time used for executing the function
    """
    def wrapper(*args, **kwargs):
        if time_it_enable:  
            start_time = time.time() 
            result = func(*args, **kwargs) 
            end_time = time.time()  
            elapsed_time = end_time - start_time 
            print(f"Function '{func.__name__}' executed in {elapsed_time:.4f} seconds")
        else:
            result = func(*args, **kwargs)  
        return result  
    return wrapper

class YoloDetector(Node):
    def __init__(self):
        super().__init__('topic_listener_node')
        self.get_logger().info("inited, if i am stucked, check if u turned on the camera")

        # self.model = YOLO("/home/sleepy/robp-group7-sleepy/yolo/models/l_300e_170jpg.pt")
        self.get_logger().info("using 3h trained yolo large model, dataset: 270images from rgbd and arm raw")
        self.model = YOLO("/home/sleepy/robp-group7-sleepy/yolo/models/l_300e_3h_270arm&rgbd.pt")

        self.bridge = CvBridge()

        self.latest_rgbd_image=None
        self.latest_depthimage=None
        self.latest_arm_image=None
        self.box_position_converter = BoxPositionConverter()

        self.subscription_rgbd_image = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.rgbd_image_listener_callback,
            10
        )

        self.subscription_arm_image = self.create_subscription(
            Image,
            "/arm_camera/filtered/image_raw",
            self.arm_image_listener_callback,
            10
        )

        self.subscription_depthimage = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depthimage_listener_callback,
            10
        )   

        self.yolo_result_pub = self.create_publisher(
            Image, '/yolo_detection_result', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.srv = self.create_service(YoloImageDetect, 'yolo_image_detect', self.yolo_image_detect_callback)

    def rgbd_image_listener_callback(self, msg):
        self.latest_rgbd_image = msg

    def arm_image_listener_callback(self, msg):
        self.latest_arm_image = msg

    def depthimage_listener_callback(self, msg):
        self.latest_depthimage = msg

    def yolo_image_detect_callback(self, request, response):
        def validate_service_request(request):
            assert request.camera_name in ['arm_camera', 'rgbd_camera'], \
                f"Invalid camera_name: {request.camera_name}. It should be 'arm_camera' or 'rgbd_camera'."
            assert isinstance(request.target_frame, str) and len(request.target_frame) > 0, \
                f"Invalid target_frame: {request.target_frame}. It should be a non-empty string."

        def wait_for_image(camera_name, timeout_duration=3):
            """wait until having image"""
            start_time = self.get_clock().now()
            latest_image = None

            if camera_name == "rgbd_camera":
                latest_image = self.latest_rgbd_image
            elif camera_name == "arm_camera":
                latest_image = self.latest_arm_image
            else:
                self.get_logger().error(f"Unknown camera_name: {camera_name}")
                return None

            while latest_image is None:
                if self.get_clock().now() - start_time > Duration(seconds=timeout_duration):
                    self.get_logger().error(f"Timeout waiting for {camera_name} image message!")
                    raise TimeoutError(f"Timeout waiting for {camera_name} image message.")
                rclpy.spin_once(self)

        @time_it
        def yolo_image_detect(image,camera_info,hardcode_depth=-1):
            def response_send(x,y,z,x_tl,y_tl,z_tl,x_br,y_br,z_br,stamp,cls,cls_num, cropped_image,frame=request.target_frame):
                pose_c = PoseStamped()
                pose_c.header.stamp = stamp
                pose_c.header.frame_id = frame  #frame？
                pose_c.pose.position.x = float(x)
                pose_c.pose.position.y = float(y)
                pose_c.pose.position.z = float(z)

                pose_tl = PoseStamped()
                pose_tl.header.stamp = stamp
                pose_tl.header.frame_id = frame  #frame？
                pose_tl.pose.position.x = float(x_tl)
                pose_tl.pose.position.y = float(y_tl)
                pose_tl.pose.position.z = float(z_tl)

                pose_br = PoseStamped()
                pose_br.header.stamp = stamp
                pose_br.header.frame_id = frame  #frame？
                pose_br.pose.position.x = float(x_br)
                pose_br.pose.position.y = float(y_br)
                pose_br.pose.position.z = float(z_br)

                assert isinstance(cls, str), "Expected class name to be an instance of String, but got {}".format(type(cls))

                detected_object = YoloClassifiedObject()
                detected_object.center_point=pose_c
                detected_object.topleft_point=pose_tl
                detected_object.bottomright_point=pose_br
                detected_object.category=cls 
                detected_object.category_num = Int32()
                detected_object.category_num = int(cls_num)

                if cropped_image is None:
                    detected_object.image =Image()
                else:
                    detected_object.image = self.bridge.cv2_to_imgmsg(cropped_image, encoding="bgr8")
                
                return detected_object

            def deal_with_one_obj(box):
                def transform_point(x,y,z,from_frame=None,to_frame=None):
                    assert from_frame!=None, "transform_point error! frame before transform doesn't exist"
                    assert to_frame!=None, "transform_point error! target frame doesn't exist"
                    if from_frame==to_frame:
                        return x,y,z
                    position_before = PointStamped()
                    position_before.header.stamp = time_request 
                    position_before.header.frame_id = from_frame
                    position_before.point.x = x
                    position_before.point.y = y
                    position_before.point.z = z
                    for attempt in range(3):
                        tf_future = self.tf_buffer.wait_for_transform_async(
                            target_frame=to_frame,
                            source_frame=from_frame,
                            time=time_request,
                        )
                        rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)

                        try:
                            transform = self.tf_buffer.lookup_transform(to_frame,  
                                                                        from_frame,
                                                                        time_request)  
                            transformed_point = tf2_geometry_msgs.do_transform_point(position_before, transform)
                            return transformed_point.point.x, transformed_point.point.y, transformed_point.point.z
                        except:
                            self.get_logger().error(f"cannot transform to {to_frame} from {from_frame} with lookup time: {time_request}")
                            continue
                        return None, None, None
                
                def get_depth_value(depth_msg, x, y):
                        """
                        x normal y from up to down 
                        """
                        x = int(round(x))
                        y = int(round(y))

                        width = depth_msg.width
                        height = depth_msg.height

                        assert 0 <= x < width, f"x={x} is out of range. It should be between 0 and {width-1}."
                        assert 0 <= y < height, f"y={y} is out of range. It should be between 0 and {height-1}."

                        data = np.frombuffer(depth_msg.data, dtype=np.uint16)
                        index = y * width + x

                        depth_value = data[index]
                        return depth_value

                def crop_and_pad_image(image):
                    padding_value=50
                    top_y = max(0,y_topleft-padding_value)
                    bottom_y = min(image.shape[0],y_bottomright+padding_value)
                    left_x = max(0,x_topleft-padding_value)
                    right_x = min(image.shape[1],x_bottomright+padding_value)

                    assert int(left_x) < int(right_x), "x_topleft must be less than x_bottomright"
                    assert int(top_y) < int(bottom_y), "y_topleft must be less than y_bottomright"
                    cropped_image = image[int(top_y):int(bottom_y), int(left_x):int(right_x)]

                    return cropped_image

                # here box xy have x and y counted from boundry up to down 
                x=box.xywh[0,0]
                y=box.xywh[0,1]
                w=box.xywh[0,2]
                h=box.xywh[0,3]
                x_topleft=box.xyxy[0,0]
                y_topleft=box.xyxy[0,1]
                x_bottomright=box.xyxy[0,2]
                y_bottomright=box.xyxy[0,3]

                if hardcode_depth==-1:
                    z=get_depth_value(self.latest_depthimage,x,y)
                elif hardcode_depth>0:
                    z=hardcode_depth
                else:
                    self.get_logger().error(f"yolo detection outputing error, hardcoded object depth has strange value{hardcode_depth}")

                X_c,Y_c,Z_c=None,None,None
                X_topleft,Y_topleft,Z_topleft=None,None,None
                X_bottomright,Y_bottomright,Z_bottomright=None,None,None
                X_c2,Y_c2,Z_c2=None,None,None
                X_topleft2,Y_topleft2,Z_topleft2=None,None,None
                X_bottomright2,Y_bottomright2,Z_bottomright2=None,None,None
                # Convert to world coordinates
                if camera_info['frame'] == 'arm_camera':
                    try:
                        transform_arm_base_link_to_arm_camera = self.tf_buffer.lookup_transform('arm_base_link', 'arm_camera', rclpy.time.Time())
                    except Exception as e:
                        self.get_logger().error(f"cannot transform to arm_base_link from {camera_info['frame']}: {e}")
                        return None, -1
                    # Get matrix of whole transform
                    # Extract translation and rotation from transform
                    translation = transform_arm_base_link_to_arm_camera.transform.translation
                    rotation = transform_arm_base_link_to_arm_camera.transform.rotation
                    
                    # Convert quaternion to rotation matrix
                    quat = [rotation.x, rotation.y, rotation.z, rotation.w]
                    transform_matrix = quaternion_to_matrix(quat)
                    
                    # Add translation to transformation matrix
                    transform_matrix[0, 3] = translation.x
                    transform_matrix[1, 3] = translation.y
                    transform_matrix[2, 3] = translation.z

                    X_c, Y_c, Z_c = self.box_position_converter.convert_image_to_world_coordinates(transform_matrix, x, y, -0.16)
                    X_topleft,Y_topleft,Z_topleft=self.box_position_converter.convert_image_to_world_coordinates(transform_matrix, x_topleft, y_topleft, z)
                    X_bottomright,Y_bottomright,Z_bottomright=self.box_position_converter.convert_image_to_world_coordinates(transform_matrix, x_bottomright, y_bottomright, z)

                    X_c2,Y_c2,Z_c2=transform_point(X_c,Y_c,Z_c,from_frame='arm_base_link',to_frame=target_frame)# take meter unit
                    X_topleft2,Y_topleft2,Z_topleft2=transform_point(X_topleft,Y_topleft,Z_topleft,from_frame='arm_base_link',to_frame=target_frame)
                    X_bottomright2,Y_bottomright2,Z_bottomright2=transform_point(X_bottomright,Y_bottomright,Z_bottomright,from_frame='arm_base_link',to_frame=target_frame)
                else:
                    X_c,Y_c,Z_c=from_2d_to_3d(x,y,z,camera_info["focal"],camera_info["c_x"],camera_info["c_y"])# the point in the robot camera frame，mm unit
                    X_topleft,Y_topleft,Z_topleft=from_2d_to_3d(x_topleft,y_topleft,z,camera_info["focal"],camera_info["c_x"],camera_info["c_y"])
                    X_bottomright,Y_bottomright,Z_bottomright=from_2d_to_3d(x_bottomright,y_bottomright,z,camera_info["focal"],camera_info["c_x"],camera_info["c_y"])

                    X_c2,Y_c2,Z_c2=transform_point(X_c/1000,Y_c/1000,Z_c/1000,from_frame=camera_frame,to_frame=target_frame)# take meter unit
                    X_topleft2,Y_topleft2,Z_topleft2=transform_point(X_topleft/1000,Y_topleft/1000,Z_topleft/1000,from_frame=camera_frame,to_frame=target_frame)
                    X_bottomright2,Y_bottomright2,Z_bottomright2=transform_point(X_bottomright/1000,Y_bottomright/1000,Z_bottomright/1000,from_frame=camera_frame,to_frame=target_frame)


                if X_c2 is None or Y_c2 is None or Z_c2 is None:
                    self.get_logger().error(f"cannot transform to {target_frame} from {camera_frame} with lookup time: {time_request}")
                    return None, -1
                
                cls=classname[box.cls[0]]
                cls_num = box.cls[0]

                padded_image=crop_and_pad_image(cv_image)
                detected_object = response_send(X_c2,Y_c2,Z_c2,X_topleft2,Y_topleft2,Z_topleft2,X_bottomright2,Y_bottomright2,Z_bottomright2,stamp,cls, cls_num,padded_image,)
                return detected_object

            cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
            
            debug_time=time.time()
            r = self.model(cv_image,
                           device=0,
                           )  
            print(f"yolo calculation took {time.time()-debug_time} seconds")

            detected_objects = []
            classname=r[0].names
            classname_list = list(classname.values())
            "result is a list, result[0] is a tensor, result[0].cpu().numpy() is a matrix "
            
            if(r[0].cpu().numpy().boxes.conf.size==0):#no empty filter, should be improved
                self.get_logger().info("nothing here")
                failed_result= response_send(0,0,0,0,0,0,0,0,0,stamp,"no_detection",0,None)
                return [failed_result], ["no_category_list"]
            
            r[0].save(filename="/home/sleepy/robp-group7-sleepy/yolo/result.jpg")
            cv_image_result = cv2.imread("/home/sleepy/robp-group7-sleepy/yolo/result.jpg")
            yolo_result_pub(cv_image_result)

            r=r[0].cpu().numpy()

            for i, box in enumerate(r.boxes):
                self.get_logger().info(f"detected object {i}")
                detected_object = deal_with_one_obj(box)
                detected_objects.append(detected_object)

            return detected_objects, classname_list

        def yolo_result_pub(cv_image_result):
            ros_image = self.bridge.cv2_to_imgmsg(cv_image_result, encoding="bgr8")
            ros_image.header.stamp = stamp
            ros_image.header.frame_id = camera_frame
            self.yolo_result_pub.publish(ros_image)

        validate_service_request(request) #make sure request is valid
        time_request=self.get_clock().now()
        stamp=time_request.to_msg()

        camera_name=request.camera_name
        target_frame=request.target_frame
        wait_for_image(camera_name,timeout_duration=3)
        self.get_logger().info("Processing image message...")
        if camera_name=="arm_camera":
            camera_frame = "arm_camera"     
            camera_info={'focal':540,'c_x':320,'c_y':240, 'frame': 'arm_camera'}######this should be tested
            detected_objects, classname_list = yolo_image_detect(self.latest_arm_image, camera_info, hardcode_depth=230)
        elif camera_name=="rgbd_camera":
            camera_frame = "camera_depth_optical_frame"
            camera_info={'focal':606.22,'c_x':330.02,'c_y':247.6, 'frame': 'camera_depth_optical_frame'}
            detected_objects, classname_list = yolo_image_detect(self.latest_rgbd_image,camera_info,hardcode_depth=-1)
        else:
            self.get_logger().error("yolo detection error, camera name doesn't match")

        response.objects = detected_objects
        response.category_list = classname_list
        
        return response



def main():
    rclpy.init()

    node = YoloDetector()

    try:
        rclpy.spin(node)  
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()


    

if __name__ == "__main__":
    main()


