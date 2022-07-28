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

from ipm_interfaces.srv import ProjectPoint, ProjectPointCloud2
from ipm_library.exceptions import InvalidPlaneException, NoIntersectionError
from ipm_library.ipm import IPM
import rclpy
# from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32, read_points_numpy
from std_msgs.msg import Header
from tf2_ros import Buffer


class IPMService(Node):
    _has_recevied_camera_info = False

    def __init__(self) -> None:
        super().__init__('ipm_service')
        # self.tf_buffer = tf2.Buffer(Duration(seconds=5))   # TODO param
        # self.tf_listener = tf2.TransformListener(self.tf_buffer, self)
        # self.ipm = IPM(self.tf_buffer)
        self.ipm = IPM(Buffer())
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 'camera_info', self.camera_info_cb, 10)
        # self.camera_info_sub = self.create_subscription(
        #     CameraInfo, 'camera_info', self.ipm.set_camera_info)
        self.point_srv = self.create_service(
            ProjectPoint, 'project_point', self.point_projection_callback)
        self.point_cloud_srv = self.create_service(
            ProjectPointCloud2, 'project_pointcloud2', self.point_cloud_projection_callback)

    def camera_info_cb(self, msg: CameraInfo) -> None:
        self._has_recevied_camera_info = True
        self.ipm.set_camera_info(msg)

    def point_projection_callback(
            self,
            request: ProjectPoint.Request,
            response: ProjectPoint.Response) -> ProjectPoint.Response:

        if not self._has_recevied_camera_info:
            response.result = ProjectPoint.Response.RESULT_NO_CAMERA_INFO
            return response

        try:
            response.point = self.ipm.project_point(request.plane, request.point).point
            response.result = ProjectPoint.Response.RESULT_SUCCESS
        except InvalidPlaneException:
            response.result = ProjectPoint.Response.RESULT_INVALID_PLANE
        except NoIntersectionError:
            response.result = ProjectPoint.Response.RESULT_NO_INTERSECTION

        return response

        # Map optional marking from '' to None
        if request.output_frame == '':
            output_frame = None
        else:
            output_frame = request.output_frame
        # Project the given point
        response.point = self.ipm.project_point(
            request.plane,
            request.point,
            output_frame).point
        return response

    def point_cloud_projection_callback(
            self,
            request: ProjectPointCloud2.Request,
            response: ProjectPointCloud2.Response) -> ProjectPointCloud2.Response:

        if not self._has_recevied_camera_info:
            response.result = ProjectPointCloud2.Response.RESULT_NO_CAMERA_INFO
            return response

        try:
            # Project the points
            projected_points = self.ipm.project_points(
                request.plane, read_points_numpy(request.points))
            # Convert them into a PointCloud2
            response.points = create_cloud_xyz32(Header(), projected_points)
            response.result = ProjectPoint.Response.RESULT_SUCCESS
        except InvalidPlaneException:
            response.result = ProjectPoint.Response.RESULT_INVALID_PLANE

        return response

        # Map optional marking from '' to None
        if request.output_frame == '':
            output_frame = self.ipm.get_camera_info().header.frame_id
        else:
            output_frame = request.output_frame
        # Project the given points
        projected_points = self.ipm.project_points(
            request.plane,
            read_points_numpy(request.points),
            output_frame)
        # Convert them into a PointCloud2
        response.points = create_cloud_xyz32(
            Header(
                stamp=request.plane.stamp,
                frame_id=output_frame),
            projected_points)
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = IPMService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
