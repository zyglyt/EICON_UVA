"""

* 2025无人系统具身智能算法挑战赛 专用代码
* 版权所有 (c) 2025 无人系统具身智能算法挑战赛组委会
* 
* 本源码仅限本赛事参赛团队在比赛过程中使用，
* 禁止任何形式的商业用途、非授权传播或用于其他非比赛场景。
* 
* 依照 GNU 通用公共许可证（GPL）条款授权：
* 参赛者可基于赛事目的对源码进行修改和扩展，
* 但修改后的代码仍受限于本声明的约束条款。
* 
* 本源码按"现状"提供，组委会不承担任何明示或暗示的担保责任，
* 包括但不限于适销性或特定用途适用性的保证。

"""
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
from go_arm.msg import Mycaryolo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from ultralytics import YOLO

class YoloPublisher(object):
    def __init__(self):
        rospy.init_node('yolo_publisher', anonymous=True)

        self.publisher_ = rospy.Publisher('mycaryolo', Mycaryolo, queue_size=10)
        self.annotated_pub = rospy.Publisher('yolo_annotated_image', Image, queue_size=10)

        self.camera_info_sub = rospy.Subscriber('/camera/wrist/info', CameraInfo, self.camera_info_callback)
        self.rgb_sub = rospy.Subscriber('/camera/wrist/rgb',  Image, self.rgb_image_callback)
        self.depth_sub  = rospy.Subscriber('/camera/wrist/depth',Image, self.depth_image_callback)

        self.bridge = CvBridge()
        self.model  = YOLO("/home/q/Go2/best.pt")

        self.fx = self.fy = self.cx = self.cy = None
        self.rgb_image   = None
        self.depth_image = None

    def camera_info_callback(self, msg: CameraInfo):
        self.fx, self.fy = msg.K[0], msg.K[4]
        self.cx, self.cy = msg.K[2], msg.K[5]
        self.camera_info_sub.unregister()
        rospy.loginfo(f"Got intrinsics fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

    def rgb_image_callback(self, msg: Image):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge RGB Error: {e}")

    def depth_image_callback(self, msg: Image):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            if self.rgb_image is not None:
                self.process_yolo_results()
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Depth Error: {e}")

    def process_yolo_results(self):
        results = self.model.predict(source=self.rgb_image)
        for r in results:
            boxes = r.boxes.xyxy.cpu().numpy().astype(int)
            for idx, (x1, y1, x2, y2) in enumerate(boxes):
                cl   = int(r.boxes.cls[idx].item())
                conf = float(r.boxes.conf[idx])
                name = r.names[cl]

                cx_pix = (x1 + x2) / 2.0
                cy_pix = (y1 + y2) / 2.0
                cx_i, cy_i = int(cx_pix), int(cy_pix)

                depth = float('nan')
                if (self.depth_image is not None and
                    0 <= cy_i < self.depth_image.shape[0] and
                    0 <= cx_i < self.depth_image.shape[1]):
                    depth = float(self.depth_image[cy_i, cx_i])

                roi = self.rgb_image[y1:y2, x1:x2]
                bottle_angle = None
                if roi.size > 0:
                    bottle_angle = self.compute_2d_tilt(roi)
                angle_str = f"{bottle_angle:.2f}" if bottle_angle is not None else "N/A"

                label = f"({cx_i},{cy_i},{depth:.2f}m,{angle_str})"
                cv2.rectangle(self.rgb_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    self.rgb_image,
                    label,
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, 
                    (0, 0, 255),
                    2 
                )

                msg = Mycaryolo()
                msg.conf = conf
                msg.name = name
                msg.pose = Pose()
                if not np.isnan(depth):
                    cam_x, cam_y, cam_z = self.convert_pixel_to_camera_coordinates(
                        cx_pix, cy_pix, depth
                    )
                    msg.pose.position.x = cam_x
                    msg.pose.position.y = cam_y
                    msg.pose.position.z = cam_z
                    self.publisher_.publish(msg)
                else:
                    rospy.logwarn("Invalid depth, skipping pose publish")

        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(self.rgb_image, encoding="bgr8")
            self.annotated_pub.publish(annotated_msg)
        except CvBridgeError as e:
            rospy.logerr(f"Annotated image conversion failed: {e}")

        cv2.namedWindow("YOLO Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("YOLO Detection", 1280, 720)
        cv2.imshow("YOLO Detection", self.rgb_image)
        cv2.waitKey(1)

    def compute_2d_tilt(self, roi):
        gray  = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur  = cv2.GaussianBlur(gray, (5,5), 0)
        edges = cv2.Canny(blur, 50, 150)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        cnt = max(contours, key=cv2.contourArea)
        rect = cv2.minAreaRect(cnt) 
        w, h = rect[1]  
        angle = rect[2] 

        if w < h:
            return angle  
        else:
        
            if angle > 0:
                return angle - 90  
            else:
                return angle + 90  

    def convert_pixel_to_camera_coordinates(self, u, v, depth):
        x = (u - self.cx) * depth / self.fx
        y = (v - self.cy) * depth / self.fy
        return x, y, depth

if __name__ == "__main__":
    try:
        node = YoloPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
