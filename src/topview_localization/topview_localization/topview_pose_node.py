#!/usr/bin/env python3
import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R


import sys
print("PYTHON:", sys.executable)
import cv2
print("CV2:", cv2.__file__)
print("VERSION:", cv2.__version__)


# =========================
# User settings
# =========================
CALIB_PATH = "/home/addinedu/topview_ws/src/topview_localization/topview_localization/calib_data_topview_v3.npz"
CAM_INDEX = 3

ARUCO_DICT = cv2.aruco.DICT_4X4_50
TARGET_ID = 2

MARKER_LENGTH_M = 0.06
DO_SUBPIX = True

PRINT_PERIOD_SEC = 0.4
WINDOW_SEC = 0.4

# =========================
# Fixed transform (constant): origin -> cam
# =========================
T_ORIGIN_CAM = np.array([
    [ 0.989111,  0.036269, -0.142629,  0.20409541],
    [ 0.01594 , -0.989857, -0.141171,  0.20045913],
    [-0.146302,  0.13736 , -0.979657,  1.34699311],
    [ 0.      ,  0.      ,  0.      ,  1.        ],
], dtype=float)

H_corr = np.array([
    [ 0.89847951,  0.10161037,  1.09134279],
    [-0.00931265,  1.00917373, -0.03433492],
    [-0.10240235,  0.09682792,  1.        ]
], dtype=float)

# =========================
# Helpers (원본 그대로 유지)
# =========================

def apply_homography(H: np.ndarray, x: float, y: float):
    p = np.array([x, y, 1.0])
    q = H @ p
    q = q / q[2]
    return float(q[0]), float(q[1])

def refine_corners_subpix(gray, corners, do_subpix=True):
    if not do_subpix:
        return
    for c in corners:
        cv2.cornerSubPix(
            gray, c,
            winSize=(5, 5),
            zeroZone=(-1, -1),
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )

def circular_mean(angles_rad: np.ndarray) -> float:
    s = float(np.mean(np.sin(angles_rad)))
    c = float(np.mean(np.cos(angles_rad)))
    return float(np.arctan2(s, c))

def compute_H_img_to_origin(K: np.ndarray, T_origin_cam: np.ndarray) -> np.ndarray:
    R_oc = T_origin_cam[:3, :3]
    t_oc = T_origin_cam[:3, 3:4]
    r1 = R_oc[:, 0:1]
    r2 = R_oc[:, 1:2]
    H_o2i = K @ np.hstack([r1, r2, t_oc])
    return np.linalg.inv(H_o2i)

def img_pts_to_origin_xy(H_i2o: np.ndarray, pts_uv: np.ndarray) -> np.ndarray:
    pts = np.asarray(pts_uv, dtype=float).reshape(-1, 2)
    ones = np.ones((pts.shape[0], 1), dtype=float)
    p = np.hstack([pts, ones])
    q = (H_i2o @ p.T).T
    q = q / q[:, 2:3]
    return q[:, :2]

def yaw_from_origin_corners(c_xy: np.ndarray) -> float:
    v = c_xy[1] - c_xy[0]
    return float(np.arctan2(v[1], v[0]))

# =========================
# ROS2 Node
# =========================

class TopViewPoseNode(Node):

    def __init__(self):
        super().__init__('topview_pose_node')

        self.publisher_ = self.create_publisher(
            PoseStamped,
            '/amr_pose',
            10
        )

        self.declare_parameter('show_debug_view', True)
        self.declare_parameter('debug_window_name', 'topview_pose_debug')
        self.show_debug_view = bool(self.get_parameter('show_debug_view').value)
        self.debug_window_name = str(self.get_parameter('debug_window_name').value)

        # ---- load calibration
        data = np.load(CALIB_PATH)
        self.K = data["mtx"]
        self.dist = data["dist"]

        self.H_i2o = compute_H_img_to_origin(self.K, T_ORIGIN_CAM)

        # ---- aruco detector
        aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        aruco_params = cv2.aruco.DetectorParameters()

        aruco_params.adaptiveThreshConstant = 9
        aruco_params.minMarkerPerimeterRate = 0.02
        aruco_params.maxMarkerPerimeterRate = 2.5
        aruco_params.polygonalApproxAccuracyRate = 0.03
        aruco_params.minCornerDistanceRate = 0.05
        aruco_params.minMarkerDistanceRate = 0.02
        aruco_params.minOtsuStdDev = 5.0
        aruco_params.perspectiveRemoveIgnoredMarginPerCell = 0.13

        self.detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        # # ---- camera
        # # self.cap = cv2.VideoCapture(CAM_INDEX)
        # self.cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
        # if not self.cap.isOpened():
        #     raise RuntimeError(f"❌ Camera open failed (index={CAM_INDEX})")

        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        # self.cap.set(cv2.CAP_PROP_FPS, 30)


        ####################################################################
        # YUYV -> MJPG 방식으로 변경 ; 10Hz -> 30Hz
        # ---- camera
        # self.cap = cv2.VideoCapture(CAM_INDEX)
        self.cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"❌ Camera open failed (index={CAM_INDEX})")

        # 🔥 MJPG 포맷 강제 (핵심)
        self.cap.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*'MJPG')
        )

        # 해상도 유지
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # FPS 30 요청
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        # ---- 실제 적용값 확인 출력
        fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        fourcc_str = "".join([chr((fourcc >> 8*i) & 0xFF) for i in range(4)])

        print("camera resolution:",
              self.cap.get(cv2.CAP_PROP_FRAME_WIDTH),
              self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
              "FPS:",
              self.cap.get(cv2.CAP_PROP_FPS),
              "FOURCC:",
              fourcc_str)
        ####################################################################


        print("camera resolution:",
              self.cap.get(cv2.CAP_PROP_FRAME_WIDTH),
              self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
              "FPS:",
              self.cap.get(cv2.CAP_PROP_FPS))

        print("========================================")
        print(f"Tracking TARGET_ID={TARGET_ID}")
        print("MAP pose = homography 기반")
        print("========================================")

        self.last_print_t = 0.0
        self.buf = []

        self.timer = self.create_timer(1/30, self.timer_callback)

    def _handle_debug_view(self, frame_u):
        if not self.show_debug_view:
            return

        cv2.imshow(self.debug_window_name, frame_u)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord('q')):
            self.get_logger().info("Debug view exit key pressed. Shutting down node.")
            rclpy.shutdown()

    def timer_callback(self):

        ret, frame = self.cap.read()
        if not ret:
            return

        frame_u = cv2.undistort(frame, self.K, self.dist)
        gray = cv2.cvtColor(frame_u, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is None:
            self._handle_debug_view(frame_u)
            return

        refine_corners_subpix(gray, corners, DO_SUBPIX)

        for i, marker_id in enumerate(ids.flatten().astype(int)):

            cv2.aruco.drawDetectedMarkers(
                frame_u,
                [corners[i]],
                np.array([[marker_id]])
            )

            if marker_id != TARGET_ID:
                continue

            c_uv = corners[i][0].astype(float)

            # #######################################################
            # # 🔥 마커 한 변의 픽셀 길이 측정
            # edge_length_px = np.linalg.norm(c_uv[1] - c_uv[0])
            # print(f"Marker pixel size: {edge_length_px:.2f} px")
            # #######################################################


            c_xy = img_pts_to_origin_xy(self.H_i2o, c_uv)

            c_xy_corr = np.array([
                apply_homography(H_corr, pt[0], pt[1])
                for pt in c_xy
            ])

            center = np.mean(c_xy_corr, axis=0)
            x_raw = float(center[0])
            y_raw = float(center[1])
            yaw_raw = yaw_from_origin_corners(c_xy_corr)

            now = time.time()
            self.buf.append((now, x_raw, y_raw, yaw_raw))
            self.buf = [s for s in self.buf if s[0] >= now - WINDOW_SEC]

            xs = np.array([s[1] for s in self.buf])
            ys = np.array([s[2] for s in self.buf])
            yaws = np.array([s[3] for s in self.buf])

            x_f = float(np.median(xs))
            y_f = float(np.median(ys))
            yaw_f = circular_mean(yaws)

            self.publish_pose(x_f, y_f, yaw_f)

            if now - self.last_print_t > PRINT_PERIOD_SEC:
                print(f"[MAP] x={x_f:+.3f}, y={y_f:+.3f}, yaw={np.degrees(yaw_f):+.2f} deg")
                self.last_print_t = now

            break

        self._handle_debug_view(frame_u)

    def publish_pose(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0

        q = R.from_euler('z', yaw).as_quat()
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self.publisher_.publish(msg)

    def destroy_node(self):
        self.cap.release()
        if self.show_debug_view:
            cv2.destroyAllWindows()
        super().destroy_node()

# =========================
# main
# =========================

def main(args=None):
    rclpy.init(args=args)
    node = TopViewPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
