# Copyright (c) 2022 Hamburg Bit-Bots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import Optional

from builtin_interfaces.msg import Time
from ipm_library import utils
from ipm_library.exceptions import (
    CameraInfoNotSetException,
    InvalidPlaneException,
    NoIntersectionError)
import numpy as np
from sensor_msgs.msg import CameraInfo
from shape_msgs.msg import Plane
from tf2_geometry_msgs import PointStamped
import tf2_ros
from vision_msgs.msg import Point2D


class IPM:
    _camera_info: Optional[CameraInfo] = None

    def __init__(
            self,
            tf_buffer: tf2_ros.Buffer,
            camera_info: Optional[CameraInfo] = None) -> None:
        """
        Create a new inverse perspective mapper instance.

        :param tf_buffer: This module needs access to a sufficiently large tf2 buffer
        :param camera_info: `CameraInfo` Message containing the
            camera intrinsics, camera frame, ...
            The camera info can be updated later on using the setter or
            provided directly if it is unlikly to change
        """
        # TF needs a listener that is init in the node context, so we need a reference
        self._tf_buffer = tf_buffer
        self.set_camera_info(camera_info)

    def set_camera_info(self, camera_info: CameraInfo) -> None:
        """
        Set a new `CameraInfo` message.

        :param camera_info: The updated camera info message.
        """
        self._camera_info = camera_info

    def get_camera_info(self):
        """
        Return the latest `CameraInfo` message.

        :returns: The message.
        """
        return self._camera_info

    def camera_info_received(self) -> bool:
        """
        Return if `CameraInfo` message has been received.

        :returns: If the message was received
        """
        return self._camera_info is not None

    def map_point(
            self,
            plane: Plane,
            point: Point2D,
            time: Time,
            plane_frame_id: str,
            output_frame_id: Optional[str] = None) -> PointStamped:
        """
        Map `Point2DStamped` to 3D `Point` assuming point lies on given plane.

        Uses latest CameraInfo intrinsics to convert `Point2DStamped` in image coordinates to 3D
        `Point` in output frame.

        :param plane: Plane in which the mapping should happen
        :param point: Point that should be mapped
        :param time: Time at which the point (or the image where it is from) is was captured
        :param plane_frame_id: TF2 frame referenced for the plane
        :param output_frame_id: TF2 frame in which the output should be provided
        :rasies CameraInfoNotSetException if camera info has not been provided
        :raise: InvalidPlaneException if the plane is invalid
        :raise: NoIntersectionError if the point is not on the plane
        :returns: The point mapped onto the given plane in the output frame
        """
        # Create numpy array from point and call map_points()
        np_point = self.map_points(
            plane,
            np.array([[point.x, point.y]]),
            time,
            plane_frame_id,
            output_frame_id=None)[0]

        # Check if we have any nan values, aka if we have a valid intersection
        if np.isnan(np_point).any():
            raise NoIntersectionError

        # Create output point
        intersection_stamped = PointStamped()
        intersection_stamped.point.x = np_point[0]
        intersection_stamped.point.y = np_point[1]
        intersection_stamped.point.z = np_point[2]
        intersection_stamped.header.stamp = time
        intersection_stamped.header.frame_id = self._camera_info.header.frame_id

        # Transform output point if output frame if needed
        if output_frame_id not in [None, self._camera_info.header.frame_id]:
            intersection_stamped = self._tf_buffer.transform(
                intersection_stamped, output_frame_id)

        return intersection_stamped

    def map_points(
            self,
            plane_msg: Plane,
            points: np.ndarray,
            time: Time,
            plane_frame_id: str,
            output_frame_id: Optional[str] = None) -> np.ndarray:
        """
        Map image points onto a given plane using the latest CameraInfo intrinsics.

        :param plane_msg: Plane in which the mapping should happen
        :param points: Points that should be mapped in the form of
            a nx2 numpy array where n is the number of points
        :param time: Time at which the point (or the image where it is from) is was captured
        :param plane_frame_id: TF2 frame referenced for the plane
        :param output_frame_id: TF2 frame in which the output should be provided
        :returns: The points mapped onto the given plane in the output frame
        :rasies CameraInfoNotSetException if camera info has not been provided
        :raises InvalidPlaneException if the plane is invalid
        """
        if not self.camera_info_received():
            raise CameraInfoNotSetException

        if not np.any(plane_msg.coef[:3]):
            raise InvalidPlaneException

        # Convert plane from general form to point normal form
        plane = utils.plane_general_to_point_normal(plane_msg)

        # View plane from camera frame
        plane_base_point, plane_normal = utils.transform_plane_to_frame(
            plane=plane,
            input_frame=plane_frame_id,
            output_frame=self._camera_info.header.frame_id,
            stamp=time,
            buffer=self._tf_buffer)

        # Convert points to float if they aren't allready
        if points.dtype.char not in np.typecodes['AllFloat']:
            points = points.astype(np.float32)

        # Get intersection points with plane
        np_points = utils.get_field_intersection_for_pixels(
            self._camera_info,
            points,
            plane_normal,
            plane_base_point)

        # Transform output point if output frame if needed
        if output_frame_id not in [None, self._camera_info.header.frame_id]:
            output_transformation = self._tf_buffer.lookup_transform(
                output_frame_id,
                self._camera_info.header.frame_id,
                time)
            np_points = utils.transform_points(
                np_points, output_transformation.transform)

        return np_points
