#!/usr/bin/env python3
import rospy
import rospkg
import numpy as np
import torch
import torch.backends.cudnn as cudnn
from pathlib import Path
from sensor_msgs.msg import Image
from yolov5_ros.msg import BoundingBox, BoundingBoxes
from robot_ctrl.msg import CenterAndArray, DepthandDeg
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber
import os
import sys

# YOLOv5 Imports
rospack = rospkg.RosPack()
yolov5_path = rospack.get_path('yolov5_ros') + '/src/yolov5'
if yolov5_path not in sys.path:
    sys.path.append(yolov5_path)  # add yolov5 to PATH
sys.path.append(yolov5_path + "/models")
sys.path.append(yolov5_path + "/utils")

from models.common import DetectMultiBackend
from utils.general import check_img_size, non_max_suppression, scale_boxes
from utils.torch_utils import select_device
from utils.augmentations import letterbox

class DepthCorrection:
    def __init__(self, horizontal_fov=56, min_depth=0.1, max_depth=10.0):
        self.horizontal_fov = np.radians(horizontal_fov)
        self.min_depth = min_depth
        self.max_depth = max_depth 

    def correct_depth(self, depth, pixel_x, image_width):
        angle = self.calculate_angle(pixel_x, image_width)
        correction_factor = self.get_correction_factor(angle)
        corrected_depth = depth * correction_factor #카메라 가장자리의 거리왜곡 보정 점수
        corrected_depth = np.clip(corrected_depth, self.min_depth, self.max_depth)
        return corrected_depth

    def calculate_angle(self, pixel_x, image_width):
        x_offset_from_center = pixel_x - (image_width / 2)
        angle = (x_offset_from_center / (image_width / 2)) * 28  
        return angle

    def get_correction_factor(self, angle):
        angle_radians = np.radians(angle)
        correction_factor = 1 / np.cos(angle_radians)
        correction_factor = np.clip(correction_factor, 1, 1.2)
        return correction_factor

class HumanLocationProcessor:
    def __init__(self):
        rospy.init_node('human_location_processor', anonymous=True)
        
        # YOLOv5 Parameters
        self.conf_thres = rospy.get_param("~confidence_threshold", 0.75)
        self.iou_thres = rospy.get_param("~iou_threshold", 0.45)
        self.agnostic_nms = rospy.get_param("~agnostic_nms", True)
        self.max_det = rospy.get_param("~maximum_detections", 1000)
        weights = rospy.get_param("~weights", yolov5_path + '/weights/yolov5s.pt')
        self.device = select_device(str(rospy.get_param("~device", "cpu")))
        self.model = DetectMultiBackend(weights, device=self.device, dnn=rospy.get_param("~dnn", True), data=rospy.get_param("~data", ""))
        self.stride, self.names, self.pt, self.jit, self.onnx, self.engine = (
            self.model.stride,
            self.model.names,
            self.model.pt,
            self.model.jit,
            self.model.onnx,
            self.model.engine,
        )
        self.img_size = [rospy.get_param("~inference_size_w", 640), rospy.get_param("~inference_size_h", 480)]
        self.img_size = check_img_size(self.img_size, s=self.stride)
        self.half = rospy.get_param("~half", False)
        self.half &= (
            self.pt or self.jit or self.onnx or self.engine
        ) and self.device.type != "cpu"
        if self.pt or self.jit:
            self.model.model.half() if self.half else self.model.model.float()
        bs = 1
        cudnn.benchmark = True
        self.model.warmup(imgsz=(1 if self.pt else bs, 3, *self.img_size))

        self.depth_correction = DepthCorrection(horizontal_fov=56) 

        self.image_sub = Subscriber('/camera/color/image_raw', Image)
        self.depth_sub = Subscriber('/camera/depth/image_raw', Image) # gazebo 시뮬레이션시 사용
        #self.depth_sub = Subscriber('/camera/aligned_depth_to_color/image_raw', Image)

        self.ts = ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], queue_size=10, slop=0.1) # 
        self.ts.registerCallback(self.sync_callback)

        self.pub = rospy.Publisher('/depth_and_deg', DepthandDeg, queue_size=10)

        self.bridge = CvBridge()

    def sync_callback(self, img_msg, depth_msg):
        detected_persons = self.detect_person(img_msg) #사람 인식
        if not detected_persons:
            rospy.logwarn("No persons detected.")
            return
            
        closest_depth = None
        closest_degree = None
        #인식한 사람중 가장 가까운 사람 선택
        for person in detected_persons:
            xc, yc, x = person
            center_depth, avg_x = self.get_min_depth(depth_msg, x, yc, xc)
            if center_depth is not None and avg_x is not None:
                if closest_depth is None or center_depth < closest_depth:
                    degree = self.depth_correction.calculate_angle(xc, self.img_size[0])
                    corrected_depth = self.depth_correction.correct_depth(center_depth, xc, self.img_size[0])
                    closest_depth = corrected_depth
                    closest_degree = degree

        if closest_depth is not None and closest_degree is not None:
            self.publish_depth_and_deg(closest_depth, closest_degree)
        else:
            rospy.logwarn("No valid depth found for any detected person.")

    def detect_person(self, img_msg):
        try:
            img_cv = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return []
        
        im, im0 = self.preprocess(img_cv)
        im = torch.from_numpy(im).to(self.device)
        im = im.half() if self.half else im.float()
        im /= 255
        if len(im.shape) == 3:
            im = im[None]
        pred = self.model(im, augment=False, visualize=False)
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, classes=[0], agnostic=self.agnostic_nms, max_det=self.max_det)       
        det = pred[0].cpu().numpy()
        detected_persons = []
        if len(det):
            det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()
            for *xyxy, conf, cls in reversed(det):
                if self.names[int(cls)] == 'person':
                    xc = int((xyxy[0] + xyxy[2]) / 2)
                    yc = int((xyxy[1] + xyxy[3]) / 2)
                    x = [i for i in range(int(xyxy[0]) + 1, int(xyxy[2]))]
                    detected_persons.append((xc, yc, x))
        return detected_persons
    #이미지 전처리
    def preprocess(self, img):
        img0 = img.copy()
        img = np.array([letterbox(img, self.img_size, stride=self.stride, auto=self.pt)[0]])
        img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
        img = np.ascontiguousarray(img)
        return img, img0 

    def get_min_depth(self, depth_msg, x, yc,xc):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return None, None

        depth_array = np.array(depth_image, dtype=np.float32)

        if yc >= depth_array.shape[0]:
            rospy.logwarn(f"yc coordinate {yc} is out of bounds of the depth image.")
            return None, None

        valid_depths_and_x = [(depth_array[yc, xi], xi) for xi in x if xi < depth_array.shape[1] and depth_array[yc, xi] > 0]

        if valid_depths_and_x:
            _min_depth, min_x = min(valid_depths_and_x, key=lambda t: t[0])
            avg_x =xc;
            min_depth = round(_min_depth / 1000, 1) 
            return min_depth, avg_x
        else:
            rospy.logwarn(f"No valid depths found in x array at yc={yc}.")
            return None, None

    def publish_depth_and_deg(self, center_depth, degree):
        msg = DepthandDeg()
        msg.center_depth = center_depth
        msg.deg = degree
        self.pub.publish(msg)
        rospy.loginfo(f"Published Depth: {center_depth} m, Degree: {degree}°")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        processor = HumanLocationProcessor()
        processor.run()
    except rospy.ROSInterruptException:
        pass
