import argparse
import os
import time
import yaml
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R
import scipy.spatial.transform as st
from PyQt6.QtWidgets import *
from PyQt6.QtGui import *
from PyQt6.QtCore import *

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from moveit.planning import MoveItPy, PlanRequestParameters, MultiPipelinePlanRequestParameters
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped, Pose
from control_msgs.action import GripperCommand

from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo

import cv2
print(cv2.aruco)
from cv2 import aruco

np.set_printoptions(precision=3, suppress=True)

class Worker(QRunnable):
    '''
    Worker thread

    Inherits from QRunnable to handler worker thread setup, signals and wrap-up.

    :param callback: The function callback to run on this worker thread. Supplied args and
                     kwargs will be passed through to the runner.
    :type callback: function
    :param args: Arguments to pass to the callback function
    :param kwargs: Keywords to pass to the callback function

    '''

    def __init__(self, fn, *args, **kwargs):
        super(Worker, self).__init__()
        # Store constructor arguments (re-used for processing)
        self.fn = fn
        self.args = args
        self.kwargs = kwargs

    @pyqtSlot()
    def run(self):
        '''
        Initialise the runner function with passed args, kwargs.
        '''
        self.fn(*self.args, **self.kwargs)

class ImageSubscriber(QThread):
    new_image = pyqtSignal(object)

    def __init__(self, camera_topic):
        super().__init__()
        self.cv_bridge = CvBridge()
        self.camera_topic = camera_topic
        self.camera_callback_group = ReentrantCallbackGroup()
        self.camera_qos_profile = QoSProfile(
                depth=1,
                history=QoSHistoryPolicy(rclpy.qos.HistoryPolicy.KEEP_LAST),
                reliability=QoSReliabilityPolicy(rclpy.qos.ReliabilityPolicy.RELIABLE),
            )

    def run(self):
        self.node = rclpy.create_node('image_subscriber')
        self.subscription = self.node.create_subscription(
            Image, 
            self.camera_topic, 
            self.image_callback, 
            self.camera_qos_profile,
            callback_group=self.camera_callback_group,
            )
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.spin()

    def update_topic(self, camera_topic):
        self.camera_topic = camera_topic
        self.executor.remove_node(self.node)
        self.node.destroy_subscription(self.subscription)
        self.subscription = self.node.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            self.camera_qos_profile,
            callback_group=self.camera_callback_group,
            )
        self.executor.add_node(self.node)
        self.executor.wake()

    def image_callback(self, msg):
        rgb_img = self.cv_bridge.imgmsg_to_cv2(msg, "rgb8")
        self.new_image.emit(rgb_img)

class CameraInfoSubscriber(QThread):
    new_camera_info = pyqtSignal(object)

    def __init__(self, camera_info_topic):
        super().__init__()
        self.camera_info_topic = camera_info_topic
        self.camera_callback_group = ReentrantCallbackGroup()
        self.camera_qos_profile = QoSProfile(
                depth=1,
                history=QoSHistoryPolicy(rclpy.qos.HistoryPolicy.KEEP_LAST),
                reliability=QoSReliabilityPolicy(rclpy.qos.ReliabilityPolicy.RELIABLE),
            )

    def run(self):
        self.node = rclpy.create_node('camera_info_subscriber')
        self.subscription = self.node.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            self.camera_qos_profile,
            callback_group=self.camera_callback_group,
            )
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.spin()

    def update_topic(self, camera_info_topic):
        self.camera_info_topic = camera_info_topic
        self.executor.remove_node(self.node)
        self.node.destroy_subscription(self.subscription)
        self.subscription = self.node.create_subscription(
            CameraInfo, 
            self.camera_info_topic, 
            self.camera_info_callback, 
            self.camera_qos_profile,
            callback_group=self.camera_callback_group,
            )
        self.executor.add_node(self.node)
        self.executor.wake()

    def camera_info_callback(self, msg):
        self.new_camera_info.emit(msg)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        # robot_ip = args.robot_ip
        # aubo_type = args.aubo_type
        # use_fake_hardware = args.use_fake_hardware
        # moveit_config = (
        #     MoveItConfigsBuilder(
        #         robot_name=aubo_type, package_name="aubo_description"
        #     )
        #     .robot_description_semantic(file_path=get_package_share_directory("aubo_description") 
        #         + "/srdf/aubo.srdf.xacro")
        #     .robot_description(file_path=get_package_share_directory("aubo_description") 
        #         + "/urdf/aubo.urdf.xacro"
        #         ,mappings={
        #                 "robot_ip": robot_ip,
        #                 "aubo_type": aubo_type,
        #                 "use_fake_hardware": use_fake_hardware,
        #                 }
        #         )
        #     .planning_pipelines("ompl", ["ompl",  "pilz_industrial_motion_planner"])
        #     .moveit_cpp(
        #         file_path=get_package_share_directory("aubo_description")
        #         + "/config/moveit_cpp.yaml"
        #     )
        #     .planning_scene_monitor(publish_robot_description=True,publish_robot_description_semantic=True)
        #     .to_moveit_configs()
        # ).to_dict()

        moveit_config = (MoveItConfigsBuilder(robot_name="sgr532", package_name="sagittarius_moveit")
            .robot_description_semantic(file_path=get_package_share_directory("sagittarius_moveit")
                + "/config/sgr532.srdf.xacro")
            .robot_description(file_path=get_package_share_directory("sagittarius_moveit")
                + "/config/sgr532.urdf.xacro"
                )
            .planning_pipelines("ompl", ["ompl",  "pilz_industrial_motion_planner"])
            .moveit_cpp(
                file_path=get_package_share_directory("sagittarius_moveit")
                + "/config/moveit_cpp.yaml"
            )
            .planning_scene_monitor(publish_robot_description=True,publish_robot_description_semantic=True)
            .to_moveit_configs()
            ).to_dict()

        self.sgr532 = MoveItPy(config_dict=moveit_config)
        self.planning_scene_monitor = self.sgr532.get_planning_scene_monitor()
        self.sgr532_arm = self.sgr532.get_planning_component("sagittarius_arm") 

        # add ground plane
        # TODO: add calibration board geometry
        with self.planning_scene_monitor.read_write() as scene:
            collision_object = CollisionObject()
            collision_object.header.frame_id = "sgr532/base_link"
            collision_object.id = "ground_plane"

            box_pose = Pose()
            box_pose.position.x = 0.0
            box_pose.position.y = 0.0
            box_pose.position.z = 0.0

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [2.0, 2.0, 0.001]

            collision_object.primitives.append(box)
            collision_object.primitive_poses.append(box_pose)
            collision_object.operation = CollisionObject.ADD

            scene.apply_collision_object(collision_object)

            # finally handle the allowed collisions for the object
            scene.allowed_collision_matrix.set_entry("ground_plane", "sgr532/base_link", True)
            scene.allowed_collision_matrix.set_entry("ground_plane", "sgr532/link1", True)

            scene.current_state.update()
        self.threadpool = QThreadPool()

        # GUI application parameters
        self.calibration_status = "None"

        self.image_subscriber = None
        self.current_image = None

        self.camera_info_subscriber = None
        self.camera_info = None

        # results
        os.makedirs("./results", exist_ok=True)
        self.rmat = None
        self.pos = None
        self.samples_img = []
        self.samples_pose = []
        self.samples_gripper2base = []
        os.makedirs('CamCali/origin_img', exist_ok=True)
        self.pose_file = 'CamCali/gripper_pose.txt'
        self.gripper2base_file = 'CamCali/gripper2base.txt'

        # initialize the GUI
        self.initUI()

    def gripper2base(self):
        """Get the transform from the gripper coordinate frame to the base coordinate frame"""
        self.sgr532_arm.set_start_state_to_current_state()
        robot_state = self.sgr532_arm.get_start_state()
        return robot_state.get_frame_transform("sgr532/ar_tag_link")

    def gripper_pose(self):
        """Get the pose of the gripper"""
        self.sgr532_arm.set_start_state_to_current_state()
        robot_state = self.sgr532_arm.get_start_state()

        pose = robot_state.get_pose("sgr532/ar_tag_link")

        pose_pos = np.array([
            pose.position.x,
            pose.position.y,
            pose.position.z,
        ])

        pos_euler = R.from_quat([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]).as_euler("xyz", degrees=True)

        gripper_pose = np.concatenate([pose_pos, pos_euler])
        np.set_printoptions(precision=3, suppress=True)
        print(gripper_pose)
        return gripper_pose
    def initUI(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_screen = QVBoxLayout(central_widget)
        horizontal_panes = QHBoxLayout()
        left_pane = QVBoxLayout()
        right_pane = QVBoxLayout()
        right_pane.setAlignment(Qt.AlignmentFlag.AlignTop)
        right_pane.setSpacing(20)

        # Left Pane
        self.label_image = QLabel()
        left_pane.addWidget(self.label_image)
        horizontal_panes.addLayout(left_pane)

        # Right Pane
        self.camera_topic_label = QLabel("Camera Topics:")
        self.camera_topic_label.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.camera_topic_label.setFixedHeight(20)

        self.camera_topic_name = QLineEdit()
        self.camera_topic_name.setPlaceholderText("Enter image topic")
        self.camera_topic_name.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.camera_topic_name.setFixedHeight(20)
        self.camera_topic_name.returnPressed.connect(self.update_camera_topic)

        self.camera_info_topic_name = QLineEdit()
        self.camera_info_topic_name.setPlaceholderText("Enter camera info topic")
        self.camera_info_topic_name.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.camera_info_topic_name.setFixedHeight(20)
        self.camera_info_topic_name.returnPressed.connect(self.update_camera_info_topic)

        self.upload_application_parameters_button = QPushButton("Upload Application Config")
        self.upload_application_parameters_button.setFixedHeight(20)
        self.upload_application_parameters_button.clicked.connect(self.upload_application_parameters)

        # checkbox for calibration type
        self.calibration_type_label = QLabel("Calibration Type:")
        self.calibration_type_label.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.calibration_type_label.setFixedHeight(20)

        self.calibration_type_button_group = QButtonGroup()
        self.calibration_type_eye_in_hand = QRadioButton("Eye-in-Hand")
        self.calibration_type_eye_in_hand.setChecked(True)
        self.calibration_type_hand_eye = QRadioButton("Hand-eye")
        self.calibration_type_button_group.addButton(self.calibration_type_eye_in_hand)
        self.calibration_type_button_group.addButton(self.calibration_type_hand_eye)

        self.start_take_sample_button = QPushButton("Take Samples")
        self.start_take_sample_button.setFixedHeight(20)
        self.start_take_sample_button.clicked.connect(self.start_take_samples)

        self.start_compute_button = QPushButton("Compute Result")
        self.start_compute_button.setFixedHeight(20)
        self.start_compute_button.clicked.connect(self.start_compute)


        # add all widgets to right pane
        right_pane.addWidget(self.camera_topic_label)
        right_pane.addWidget(self.camera_topic_name)
        right_pane.addWidget(self.camera_info_topic_name)
        right_pane.addWidget(self.upload_application_parameters_button)
        right_pane.addWidget(self.calibration_type_label)
        right_pane.addWidget(self.calibration_type_eye_in_hand)
        right_pane.addWidget(self.calibration_type_hand_eye)
        right_pane.addWidget(self.start_take_sample_button)
        right_pane.addWidget(self.start_compute_button)

        # add to main screen
        horizontal_panes.addLayout(right_pane)
        main_screen.addLayout(horizontal_panes)
        central_widget.setLayout(main_screen)

        self.setWindowTitle('Camera Calibration')
        self.show()

    def update_camera_topic(self, text=None):
        self.camera_topic_name.setText(text)
        if text is None:
            topic_name = self.camera_topic_name.text()
        else:
            topic_name = text

        if self.image_subscriber is not None:
            self.image_subscriber.update_topic(topic_name)
        else:
            self.image_subscriber = ImageSubscriber(topic_name)
            self.image_subscriber.new_image.connect(self.update_image)
            self.image_subscriber.start()

    def update_camera_info_topic(self, text=None):
        self.camera_info_topic_name.setText(text)
        if text is None:
            topic_name = self.camera_info_topic_name.text()
        else:
            topic_name = text

        if self.camera_info_subscriber is not None:
            self.camera_info_subscriber.update_topic(topic_name)
        else:
            self.camera_info_subscriber = CameraInfoSubscriber(topic_name)
            self.camera_info_subscriber.new_camera_info.connect(self.update_camera_info)
            self.camera_info_subscriber.start()

    def update_image(self, rgb_img):
        # store the current image
        self.current_image = rgb_img.copy() 

        # display the image
        height, width, channel = rgb_img.shape
        bytes_per_line = channel * width
        qimg = QImage(rgb_img.data, width, height, QImage.Format(13))
        pixmap = QPixmap.fromImage(qimg)
        self.label_image.setPixmap(pixmap)
    
    def update_camera_info(self, camera_info):
        self.camera_info = camera_info
    
    def upload_application_parameters(self):
        # get the path to the application parameters file
        file_path, _ = QFileDialog.getOpenFileName(self, "Open Application Parameters File", "", "YAML Files (*.yaml)")
        if file_path:
            # read the yaml file
            with open(file_path, "r") as file:
                self.config = yaml.load(file, Loader=yaml.FullLoader)

            # camera topic
            if self.config["camera_image_topic"]!="":
                self.update_camera_topic(self.config["camera_image_topic"])

            if self.config["camera_info_topic"]!="":
                self.update_camera_info_topic(self.config["camera_info_topic"])

            # aruco board detection
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100) # TODO: move to config   
            self.charuco_board = cv2.aruco.CharucoBoard_create(
                self.config["charuco"]["squares_x"],
                self.config["charuco"]["squares_y"],
                self.config["charuco"]["square_length"],
                self.config["charuco"]["marker_length"],
                self.aruco_dict
            )
            self.detector_params = cv2.aruco.DetectorParameters_create()
            self.detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.calib_flags = cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_FIX_PRINCIPAL_POINT + cv2.CALIB_FIX_FOCAL_LENGTH
            self.cv_bridge = CvBridge()

    def detect_charuco_board(self, image):
        """
        Detect charuco board in image

        Adapted from: https://github.com/AlexanderKhazatsky/R2D2/blob/1aa471ae35cd9b11e20cc004c15ad4c74e92605d/r2d2/calibration/calibration_utils.py#L122
        """
        # detect aruco markers
        corners, ids, rejectedImgPoints = aruco.detectMarkers(
            image, 
            self.aruco_dict, 
            parameters=self.detector_params,
        )
        
        # find undetected markers
        corners, ids, _, _ = cv2.aruco.refineDetectedMarkers(
            image,
            self.charuco_board,
            corners,
            ids,
            rejectedImgPoints,
            parameters=self.detector_params,
            cameraMatrix=np.array(self.camera_info.k).reshape(3,3),
            distCoeffs=np.array(self.camera_info.d),
            )

        # if no markers found, return
        if ids is None:
            print("No markers found!")
            return None, None

        # detect charuco board
        num_corners_found, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
            corners, 
            ids, 
            image, 
            self.charuco_board, 
            cameraMatrix=np.array(self.camera_info.k).reshape(3,3),
            distCoeffs=np.array(self.camera_info.d),
        )

        # if no charuco board found, return
        if num_corners_found < 5:
            print("Charuco board not found!")
            return None, None

        # draw detected charuco board
        image = aruco.drawDetectedCornersCharuco(
            image, charuco_corners,
        )
        

        return image, charuco_corners, charuco_ids, image.shape[:2]


    def calc_target_to_camera(self, readings):
        """
        Calculate target to camera transform

        Adapted from: https://github.com/AlexanderKhazatsky/R2D2/blob/1aa471ae35cd9b11e20cc004c15ad4c74e92605d/r2d2/calibration/calibration_utils.py#L164
        """
        init_corners_all = []  # Corners discovered in all images processed
        init_ids_all = []  # Aruco ids corresponding to corners discovered
        fixed_image_size = readings[0][3]

        # Proccess Readings #
        init_successes = []
        for i in range(len(readings)):
            corners, charuco_corners, charuco_ids, img_size = readings[i]
            assert img_size == fixed_image_size
            init_corners_all.append(charuco_corners)
            init_ids_all.append(charuco_ids)
            init_successes.append(i)

        # First Pass: Find Outliers #
        threshold = 10
        if len(init_successes) < threshold:
            print("len(init_successes) < threshold")
            return None

        calibration_error, cameraMatrix, distCoeffs, rvecs, tvecs, stdIntrinsics, stdExtrinsics, perViewErrors = (
            aruco.calibrateCameraCharucoExtended(
                charucoCorners=init_corners_all,
                charucoIds=init_ids_all,
                board=self.charuco_board,
                imageSize=fixed_image_size,
                flags=self.calib_flags,
                cameraMatrix=np.array(self.camera_info.k).reshape(3,3),
                distCoeffs=np.array(self.camera_info.d),
            )
        )
        print(f"perViewErrors:{perViewErrors}")

        # Remove Outliers #
        final_corners_all = [
                init_corners_all[i] for i in range(len(perViewErrors)) if perViewErrors[i] <= 3.0 # TODO: read from params
        ]
        final_ids_all = [
            init_ids_all[i] for i in range(len(perViewErrors)) if perViewErrors[i] <= 3.0
        ]
        final_successes = [
            init_successes[i] for i in range(len(perViewErrors)) if perViewErrors[i] <= 3.0
        ]
        if len(final_successes) < threshold:
            return None

        # Second Pass: Calculate Finalized Extrinsics #
        calibration_error, cameraMatrix, distCoeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
            charucoCorners=final_corners_all,
            charucoIds=final_ids_all,
            board=self.charuco_board,
            imageSize=fixed_image_size,
            flags=self.calib_flags,
            cameraMatrix=np.array(self.camera_info.k).reshape(3,3),
            distCoeffs=np.array(self.camera_info.d),
        )
        
        # Return Transformation #
        if calibration_error > 3.0:
            return None

        rmats = [R.from_rotvec(rvec.flatten()).as_matrix() for rvec in rvecs]
        tvecs = [tvec.flatten() for tvec in tvecs]

        return rmats, tvecs, final_successes
    def save_image(self, img, index):
        filename = f'CamCali/origin_img/img_{index}.png'
        print(filename)
        cv2.imwrite(filename, img)
    def run_take_samples(self):
        print('run_take_samples')
        
        # check if we have both camera info and image
        if self.current_image is None: #or self.camera_info is None:
            raise Exception("No image or camera info received yet")
        # capture image
        img = self.current_image.copy()
        gripper_pose = self.gripper_pose()
        print(gripper_pose)
        gripper2base = self.gripper2base()
        self.samples_img.append(img)
        self.samples_pose.append(gripper_pose)
        self.samples_gripper2base.append(gripper2base)
         # Save data to local files
        self.save_image(img, len(self.samples_img) - 1)
        with open(self.pose_file, 'a') as f:
            np.savetxt(f, [gripper_pose], delimiter=',', fmt='%1.3f')

    def run_compute(self):
        if self.calibration_type_eye_in_hand.isChecked():
            flag_eye_in_hand = True
        if self.calibration_type_hand_eye.isChecked():
            flag_eye_in_hand = False
        readings = []
        for image in self.samples_img:
            readings.append(self.detect_charuco_board(image))

        # calculate target to camera transform
        R_target2cam, t_target2cam, successes = self.calc_target_to_camera(readings)

        # filter gripper2base by successes
        self.samples_gripper2base = [self.samples_gripper2base[i] for i in successes]
        R_gripper2base = [t[:3,:3] for t in self.samples_gripper2base]
        t_gripper2base = [t[:3,3] for t in self.samples_gripper2base]
        R_base2gripper = [t[:3,:3].T for t in self.samples_gripper2base]
        t_base2gripper = [-R @ t[:3,3] for R, t in zip(R_base2gripper, self.samples_gripper2base)]

        print("R_gripper2base",R_gripper2base)
        print("t_gripper2base",t_gripper2base)
        print("R_base2gripper",R_base2gripper)
        print("t_base2gripper",t_base2gripper)

        # run calibration for cam2bas
        if flag_eye_in_hand:
            rmat, pos = cv2.calibrateHandEye(
                R_gripper2base=R_gripper2base,
                t_gripper2base=t_gripper2base,
                R_target2cam=R_target2cam,
                t_target2cam=t_target2cam,
                method=1,
            )
        else:
            rmat, pos = cv2.calibrateHandEye(
            R_gripper2base=R_base2gripper,
            t_gripper2base=t_base2gripper,
            R_target2cam=R_target2cam,
            t_target2cam=t_target2cam,
            method=4,
        )

        # overwrite params 
        self.rmat = rmat
        self.pos = pos
        self.write_results_to_file()

    def start_take_samples(self):
        worker = Worker(self.run_take_samples)
        self.threadpool.start(worker)
    def start_compute(self):
        worker = Worker(self.run_compute)
        self.threadpool.start(worker)

    def get_transformation_matrix(self,translation, rotation):
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation
        transformation_matrix[:3, 3] = translation.flatten()
        return transformation_matrix
    def write_results_to_file(self):
        # quat = R.from_matrix(self.rmat).as_quat()
        # camera_bottom_screw_frame to camera_color_optical_frame
        R_bottom_to_optical = R.from_euler('xyz', [-1.57,0,-1.57]).as_matrix()
        T_bottom_to_optical = np.array([0.0106, 0.0325, 0.0125])

        # camera_color_optical_frame to end
        R_optical_to_end = R.from_matrix(self.rmat).as_matrix()
        T_optical_to_end = np.array(self.pos)
        M_bottom_to_optical = self.get_transformation_matrix(T_bottom_to_optical,R_bottom_to_optical)
        M_optical_to_end = self.get_transformation_matrix(T_optical_to_end,R_optical_to_end)

        # camera_bottom_screw_frame to end
        M_bottom_to_end = np.dot(M_bottom_to_optical,M_optical_to_end)

        # end to camera_bottom_screw_frame
        M_end_to_bottom = np.linalg.inv(M_bottom_to_end)
        T_end_to_bottom = M_end_to_bottom[:3,3]
        R_end_to_bottom = R.from_matrix(M_end_to_bottom[:3,:3]).as_euler('xyz')
        # write results to file

        t = time.localtime()
        current_time = time.strftime("%H:%M:%S", t)
        self.calibration_status=f"success: {current_time}"
        with open(f"./results/{current_time}.txt", 'w') as file:
            file.write("Camera Topic:\n")
            file.write(f"{self.camera_topic_name.text()} \n")
            file.write("Camera Info:\n")
            file.write(f"{self.camera_info} \n")
            file.write("Calibration result wrist3_Linkt to camera_bottom_screw_frame:\n")
            file.write("Positions (x, y, z):\n")
            file.write(f"{T_end_to_bottom} \n")
            file.write("Euler angle:\n")
            file.write(f"{R_end_to_bottom} \n")

        np.savez(
            file=f"./results/{current_time}.npz",
            position=T_end_to_bottom,
            euler=R_end_to_bottom)


def main(args=None):
    parser = argparse.ArgumentParser()
    # parser.add_argument("--robot_ip", default="127.0.0.1", required=False)
    # parser.add_argument("--aubo_type", default="aubo_ES3", required=False)
    # parser.add_argument("--use_fake_hardware", default="false", required=False)
    # args = parser.parse_args()
    rclpy.init(args=None)
    app = QApplication(sys.argv)
    ex = MainWindow()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()
