#!/home/kobe/workspace/officeBase/amr_percept_venv/.venv/bin/python
import sys
sys.path.insert(0, "/home/kobe/workspace/officeBase/amr_percept_venv/.venv/lib/python3.12/site-packages")

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    BoundingBox2D,
    ObjectHypothesisWithPose,
)
from cv_bridge import CvBridge

import cv2
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import os


class YoloDetector(Node):
    def __init__(self):
        super().__init__("yolo_detector")

        # --- parameters ---
        self.declare_parameter("visualize", True)
        self.declare_parameter("conf_threshold", 0.45)
        default_model = os.path.join(
            get_package_share_directory('servera_percept'),
            'models', 'yolov8n_openvino_model'
        )
        self.declare_parameter("model_path", default_model)

        self.visualize     = self.get_parameter("visualize").value
        self.conf_thresh   = self.get_parameter("conf_threshold").value
        model_path         = self.get_parameter("model_path").value

        # --- model (openvino format, already exported) ---
        self.model = YOLO(model_path, task="detect")
        self.get_logger().info(f"Loaded model from: {model_path}")

        # --- cv bridge ---
        self.bridge = CvBridge()

        # --- subscribers ---
        self.image_sub = self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.image_callback,
            10,
        )

        # --- publishers ---
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            "/detections",
            10,
        )
        self.image_pub = self.create_publisher(
            Image,
            "/detections_image",
            10,
        )

        # --- fps tracking ---
        self._last_time = self.get_clock().now()
        self._fps = 0.0

        self.get_logger().info("YOLOv8n detector node started ✅")

    def image_callback(self, msg: Image):
        # --- fps calculation ---
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds / 1e9
        self._fps = 1.0 / dt if dt > 0 else 0.0
        self._last_time = now

        # convert ROS Image → OpenCV BGR
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # run inference — device is auto-resolved from openvino export format
        results = self.model(
            frame,
            conf=self.conf_thresh,
            verbose=False,
        )

        # --- build Detection2DArray ---
        detections_msg = Detection2DArray()
        detections_msg.header = msg.header

        for r in results:
            for box in r.boxes:
                cls  = int(box.cls[0])
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].tolist()

                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                w  = x2 - x1
                h  = y2 - y1

                # bounding box
                bbox = BoundingBox2D()
                bbox.center.position.x = cx
                bbox.center.position.y = cy
                bbox.size_x = w
                bbox.size_y = h

                # hypothesis — id MUST be str in Jazzy's vision_msgs
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(cls)   # fix: was hypothesis.id
                hyp.hypothesis.score    = conf        # fix: was hypothesis.score directly

                det = Detection2D()
                det.header = msg.header
                det.bbox   = bbox
                det.results.append(hyp)

                detections_msg.detections.append(det)

                # --- optional visualization ---
                if self.visualize:
                    label = f"{self.class_name(cls)} {conf:.2f}"
                    cv2.rectangle(
                        frame,
                        (int(x1), int(y1)),
                        (int(x2), int(y2)),
                        (0, 255, 0), 2,
                    )
                    cv2.putText(
                        frame, label,
                        (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 2,
                    )

        # publish detections
        self.detection_pub.publish(detections_msg)

        # publish annotated image (only if someone is listening — saves CPU)
        if self.visualize and self.image_pub.get_subscription_count() > 0:
            # draw fps top-left
            cv2.putText(
                frame, f"FPS: {self._fps:.1f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0, (0, 255, 255), 2, cv2.LINE_AA,
            )
            vis_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            vis_msg.header = msg.header
            self.image_pub.publish(vis_msg)

    def class_name(self, cls_id: int) -> str:
        """Return COCO class name if available, else fallback to id string."""
        names = self.model.names  # dict {int: str} from ultralytics
        return names.get(cls_id, str(cls_id))


def main():
    rclpy.init()
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()