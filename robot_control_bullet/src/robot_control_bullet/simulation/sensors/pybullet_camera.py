# Created by Giuseppe Rizzi
# email grizzi@mavt.ehtz.ch

import pybullet as p
import numpy as np

# TODO (giuseppe) attach the camera to some link given offset


class CameraConfig:
    """ Container class for the camera intrinsics """
    def __init__(self, fov=60, width=128, height=128, near_plane=0.01, far_plane=100):
        self.fov = fov
        self.width = width
        self.height = height
        self.near_plane = near_plane
        self.far_plane = far_plane

    @property
    def aspect_ratio(self):
        return (1.0 * self.width) / self.height

    def get_projection_matrix(self):
        if self.far_plane < self.near_plane:
            raise NameError("The far plane cannot be closer than the near plane")
        return p.computeProjectionMatrixFOV(self.fov,
                                            self.aspect_ratio,
                                            self.near_plane,
                                            self.far_plane)

    def __str__(self):
        info = """
Camera config:
Field of View (FOV): {fov}
Width: {width}
Height: {height}
Aspect ratio: {aspect_ratio}
Near plane distance: {near_plane}
Far plane distance: {near_plane}
""".format(fov=self.fov, width=self.width, height=self.height, aspect_ratio=self.aspect_ratio,
           near_plane=self.near_plane, far_plane=self.far_plane)
        return info


def depth_buffer_to_depth_image(camera_config, depth_buffer):
    """
    Pybullet returns a 1-D vector of encoded depth values which this function converts to a depth image.
    :param camera_config: the camera configuration
    :param depth_buffer: the output from bullet simulation
    :return: the [width x height] depth image
    """
    assert isinstance(camera_config, CameraConfig)
    near = camera_config.near_plane
    far = camera_config.far_plane

    depth_buffer = np.reshape(depth_buffer, [camera_config.width, camera_config.height])
    depth_image = far * near / (far - (far - near) * depth_buffer)
    return depth_image


def get_camera_image(camera_config, object_id=None, link_id=None,
                     position_offset=(0, 0, 0), orientation_offset=(0.0, 0.0, 0.0, 1.0),
                     renderer=p.ER_BULLET_HARDWARE_OPENGL,
                     segmentation_mask=False):
    """
    Return the camera images
    :param camera_config: the camera config containing fov, width, eight, aspect, near and far plane
    :param object_id: the object id can optionally be provided to attach the camera to a object's link
    :param link_id: the link to which the camera is attached. Used it object_id provided
    :param position_offset: the position offset wrt to the attached link or world if object id is not provided
    :param orientation_offset: the orientation offset wrt to the attached link or world if object id is not provided
    :param segmentation_mask: set to false if the segmentation mask is not needed.
    :param renderer: which renderer to use
    :return: the output of the simulation renderer
    """
    assert isinstance(camera_config, CameraConfig)

    position = [0, 0, 0]
    orientation = [0, 0, 0, 1]
    if object_id is not None:
        _, _, _, _, position, orientation = p.getLinkState(object_id, link_id, computeForwardKinematics=True)

    camera_position, camera_orientation = p.multiplyTransforms(position, orientation,
                                                               position_offset, orientation_offset)
    rot_matrix = p.getMatrixFromQuaternion(camera_orientation)
    rot_matrix = np.array(rot_matrix).reshape(3, 3)

    # Initial vectors
    camera_z_camera = (0, 0, 1)
    camera_y_camera = (0, 1, 0)

    # Rotated vectors
    camera_vector = rot_matrix.dot(camera_z_camera)
    up_vector = rot_matrix.dot(camera_y_camera)

    view_matrix = p.computeViewMatrix(cameraEyePosition=camera_position,
                                      cameraTargetPosition=camera_position + camera_vector,
                                      cameraUpVector=up_vector)
    projection_matrix = camera_config.get_projection_matrix()
    if segmentation_mask:
        return p.getCameraImage(camera_config.width, camera_config.height, view_matrix, projection_matrix,
                                renderer=renderer)
    else:
        return p.getCameraImage(camera_config.width, camera_config.height, view_matrix, projection_matrix,
                                flags=p.ER_NO_SEGMENTATION_MASK,
                                renderer=renderer)


