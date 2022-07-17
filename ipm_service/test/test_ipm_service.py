# Copyright (c) 2022 Kenji Brameld
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
from ipm_service.ipm import IPMService
import rclpy
from sensor_msgs.msg import CameraInfo
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header


def test_topics_and_services():

    rclpy.init()
    ipm_service_node = IPMService()

    # Check subscriptions
    dict_topics = dict(ipm_service_node.get_topic_names_and_types())

    assert '/camera_info' in dict_topics
    assert 'sensor_msgs/msg/CameraInfo' in dict_topics['/camera_info']

    # Check services
    dict_services = dict(ipm_service_node.get_service_names_and_types())

    assert '/project_point' in dict_services
    assert 'ipm_interfaces/srv/ProjectPoint' in dict_services['/project_point']

    assert '/project_pointcloud2' in dict_services
    assert 'ipm_interfaces/srv/ProjectPointCloud2' in dict_services['/project_pointcloud2']

    rclpy.shutdown()


def test_project_point_no_camera_info():

    rclpy.init()
    ipm_service_node = IPMService()
    test_node = rclpy.node.Node('test')

    client = test_node.create_client(ProjectPoint, 'project_point')

    req = ProjectPoint.Request()
    req.point.x = 1.0
    req.point.y = 1.0
    req.point.z = 1.0
    future = client.call_async(req)
    rclpy.spin_once(ipm_service_node, timeout_sec=0.1)

    rclpy.spin_once(test_node, timeout_sec=0.1)

    assert future.result() is not None
    assert future.result().result == ProjectPoint.Response.RESULT_NO_CAMERA_INFO

    rclpy.shutdown()


def test_project_point_cloud_no_camera_info():

    rclpy.init()
    ipm_service_node = IPMService()
    test_node = rclpy.node.Node('test')

    client = test_node.create_client(ProjectPointCloud2, 'project_pointcloud2')

    req = ProjectPointCloud2.Request()
    req.points = create_cloud_xyz32(Header(), [])
    future = client.call_async(req)
    rclpy.spin_once(ipm_service_node, timeout_sec=0.1)

    rclpy.spin_once(test_node, timeout_sec=0.1)

    assert future.result() is not None
    assert future.result().result == ProjectPointCloud2.Response.RESULT_NO_CAMERA_INFO

    rclpy.shutdown()


def test_project_point():

    rclpy.init()
    ipm_service_node = IPMService()
    test_node = rclpy.node.Node('test')

    camera_info_pub = test_node.create_publisher(CameraInfo, 'camera_info', 10)
    camera_info_pub.publish(CameraInfo())
    rclpy.spin_once(ipm_service_node, timeout_sec=0.1)

    client = test_node.create_client(ProjectPoint, 'project_point')
    req = ProjectPoint.Request()
    req.point.x = 1.0
    req.point.y = 1.0
    req.point.z = 1.0
    future = client.call_async(req)
    rclpy.spin_once(ipm_service_node, timeout_sec=0.1)

    rclpy.spin_once(test_node, timeout_sec=0.1)

    assert future.result() is not None
    assert future.result().result == ProjectPoint.Response.RESULT_SUCCESS

    rclpy.shutdown()


def test_project_point_cloud():

    rclpy.init()
    ipm_service_node = IPMService()
    test_node = rclpy.node.Node('test')

    camera_info_pub = test_node.create_publisher(CameraInfo, 'camera_info', 10)
    camera_info_pub.publish(CameraInfo())
    rclpy.spin_once(ipm_service_node, timeout_sec=0.1)

    client = test_node.create_client(ProjectPointCloud2, 'project_pointcloud2')
    req = ProjectPointCloud2.Request()
    req.points = create_cloud_xyz32(Header(), [])
    future = client.call_async(req)
    rclpy.spin_once(ipm_service_node, timeout_sec=0.1)

    rclpy.spin_once(test_node, timeout_sec=0.1)

    assert future.result() is not None
    assert future.result().result == ProjectPoint.Response.RESULT_SUCCESS

    rclpy.shutdown()
