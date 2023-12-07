import numpy as np
import open3d as o3d
import pybullet as p
import json
import cv2
import time
import threading
from scipy.spatial.transform import Rotation as R

# some codes are copied from https://github.com/ethz-asl/vgn.git


class CameraIntrinsic(object):
    """Intrinsic parameters of a pinhole camera model.

    Attributes:
        width (int): The width in pixels of the camera.
        height(int): The height in pixels of the camera.
        K: The intrinsic camera matrix.
    """

    def __init__(self, width, height, fx, fy, cx, cy):
        self.width = width
        self.height = height
        self.K = np.array(
            [[fx, 0.0, cx],
             [0.0, fy, cy],
             [0.0, 0.0, 1.0]]
        )

    @property
    def fx(self):
        return self.K[0, 0]

    @property
    def fy(self):
        return self.K[1, 1]

    @property
    def cx(self):
        return self.K[0, 2]

    @property
    def cy(self):
        return self.K[1, 2]

    def to_dict(self):
        """Serialize intrinsic parameters to a dict object."""
        data = {
            "width": self.width,
            "height": self.height,
            "K": self.K.flatten().tolist(),
        }
        return data

    @classmethod
    def from_dict(cls, data):
        """Deserialize intrinisic parameters from a dict object."""
        intrinsic = cls(
            width=data["width"],
            height=data["height"],
            fx=data["K"][0],
            fy=data["K"][4],
            cx=data["K"][2],
            cy=data["K"][5],
        )
        return intrinsic


class Camera(object):
    """Virtual RGB-D camera based on the PyBullet camera interface.

    Attributes:
        intrinsic: The camera intrinsic parameters.
    """

    def __init__(self, camera_config="/root/docker_mount/ur5_pybullet_ros/src/ur5_pybullet_ros/script/camera/setup.json", near=0.01, far=4):
        with open(camera_config, "r") as j:
            config = json.load(j)
        camera_intrinsic = CameraIntrinsic.from_dict(config["intrinsic"])
        self.intrinsic = camera_intrinsic
        self.near = near
        self.far = far
        self.proj_matrix = _build_projection_matrix(camera_intrinsic, near, far)
        self.gl_proj_matrix = self.proj_matrix.flatten(order="F")
        self.pose = [0, 0, 1]
        self.orien = [0, 0, 0, 1]
        # thread for updating camera image
        self.update_camera_image_thread = threading.Thread(
            target=self.update_camera_image)
        self.update_camera_image_thread.setDaemon(True)
        self.update_camera_image_thread.start()
        self.rgb = None
        


    def render(self, extrinsic):
        """Render synthetic RGB and depth images.

        Args:
            extrinsic: Extrinsic parameters, T_cam_ref.
        """
        # Construct OpenGL compatible view and projection matrices.
        gl_view_matrix = extrinsic.copy() if extrinsic is not None else np.eye(4)
        gl_view_matrix[2, :] *= -1  # flip the Z axis
        gl_view_matrix = gl_view_matrix.flatten(order="F")

        result = p.getCameraImage(
            width=self.intrinsic.width,
            height=self.intrinsic.height,
            viewMatrix=gl_view_matrix,
            projectionMatrix=self.gl_proj_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL,
        )

        rgb, z_buffer = np.ascontiguousarray(result[2][:, :, :3]), result[3]
        depth = (
            1.0 * self.far * self.near / (self.far - (self.far - self.near) * z_buffer)
        )

        return Frame(rgb, depth, self.intrinsic, extrinsic)
    
    def update_camera_image(self):
        # cv2.namedWindow("image")
        while True:
            wcT = self._bind_camera_to_end(self.pose, self.orien)
            cwT = np.linalg.inv(wcT)

            frame = self.render(cwT)
            assert isinstance(frame, Frame)

            self.rgb = frame.color_image()  # 这里以显示rgb图像为例, frame还包含了深度图, 也可以转化为点云
            self.bgr = np.ascontiguousarray(self.rgb[:, :, ::-1])  # flip the rgb channel
            # cv2.imshow("image", self.bgr)
            # key = cv2.waitKey(1)
            time.sleep(0.02)
            
    def update_pose(self, pose, orien):
        self.pose = pose
        self.orien = orien
    
    def _bind_camera_to_end(self, end_pos, end_orn):
        """设置相机坐标系与末端坐标系的相对位置
        
        Arguments:
        - end_pos: len=3, end effector position
        - end_orn: len=4, end effector orientation, quaternion (x, y, z, w)

        Returns:
        - wcT: shape=(4, 4), transform matrix, represents camera pose in world frame
        """
        relative_offset = [-0.05, 0, 0.1]  # 相机原点相对于末端执行器局部坐标系的偏移量
        end_orn = R.from_quat(end_orn).as_matrix()
        end_x_axis, end_y_axis, end_z_axis = end_orn.T

        wcT = np.eye(4)  # w: world, c: camera, ^w_c T
        wcT[:3, 0] = -end_y_axis  # camera x axis
        wcT[:3, 1] = -end_z_axis  # camera y axis
        wcT[:3, 2] = end_x_axis  # camera z axis
        wcT[:3, 3] = end_orn.dot(relative_offset) + end_pos  # eye position
        return wcT
            


class Frame(object):
    def __init__(self, rgb, depth, intrinsic, extrinsic=None):
        self.rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color=o3d.geometry.Image(rgb),
            depth=o3d.geometry.Image(depth),
            depth_scale=1.0,
            depth_trunc=2.0,
            convert_rgb_to_intensity=False
        )

        self.intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width=intrinsic.width,
            height=intrinsic.height,
            fx=intrinsic.fx,
            fy=intrinsic.fy,
            cx=intrinsic.cx,
            cy=intrinsic.cy,
        )

        self.extrinsic = extrinsic if extrinsic is not None \
            else np.eye(4)
    
    def color_image(self):
        return np.asarray(self.rgbd.color)
    
    def depth_image(self):
        return np.asarray(self.rgbd.depth)

    def point_cloud(self):
        pc = o3d.geometry.PointCloud.create_from_rgbd_image(
            image=self.rgbd,
            intrinsic=self.intrinsic,
            extrinsic=self.extrinsic
        )

        return pc

    
def _build_projection_matrix(intrinsic, near, far):
    perspective = np.array(
        [
            [intrinsic.fx, 0.0, -intrinsic.cx, 0.0],
            [0.0, intrinsic.fy, -intrinsic.cy, 0.0],
            [0.0, 0.0, near + far, near * far],
            [0.0, 0.0, -1.0, 0.0],
        ]
    )
    ortho = _gl_ortho(0.0, intrinsic.width, intrinsic.height, 0.0, near, far)
    return np.matmul(ortho, perspective)


def _gl_ortho(left, right, bottom, top, near, far):
    ortho = np.diag(
        [2.0 / (right - left), 2.0 / (top - bottom), -2.0 / (far - near), 1.0]
    )
    ortho[0, 3] = -(right + left) / (right - left)
    ortho[1, 3] = -(top + bottom) / (top - bottom)
    ortho[2, 3] = -(far + near) / (far - near)
    return ortho

