# Copyright 2026 The Go2 Autonomy Stack contributors
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

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

from transform_sensors.transform_everything import transform_cloud


POINT_DTYPE = np.dtype({
    'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
    'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
    'offsets': [0, 4, 8, 16, 20, 24],
    'itemsize': 32,
})
FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=16, datatype=PointField.FLOAT32, count=1),
    PointField(name='ring', offset=20, datatype=PointField.UINT16, count=1),
    PointField(name='time', offset=24, datatype=PointField.FLOAT32, count=1),
]


def make_cloud(points):
    cloud = PointCloud2()
    cloud.header.frame_id = 'utlidar_lidar'
    cloud.header.stamp.sec = 10
    cloud.header.stamp.nanosec = 123
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = FIELDS
    cloud.point_step = 32
    cloud.row_step = 32 * len(points)
    cloud.is_bigendian = False
    cloud.is_dense = True
    cloud.data = points.tobytes()
    return cloud


def test_transform_cloud_keeps_point_fields_and_filters_body_points():
    points = np.zeros(3, dtype=POINT_DTYPE)
    points['x'] = [1.0, -0.4, -0.4]
    points['y'] = [2.0, 0.0, 0.0]
    points['z'] = [3.0, 1.0, -0.2]
    points['intensity'] = [7.5, 8.5, 9.5]
    points['ring'] = [17, 23, 31]
    points['time'] = [0.01, 0.02, 0.03]
    filter_bounds = (-0.7, -0.1, -0.3, 0.3, -0.646825, -0.046825)

    transformed = transform_cloud(
        make_cloud(points), np.eye(3), np.zeros(3), 0.046825,
        filter_bounds, 100)

    assert transformed.header.frame_id == 'body'
    assert transformed.header.stamp.sec == 10
    assert transformed.header.stamp.nanosec == 223
    assert transformed.height == 1
    assert transformed.width == 2
    assert transformed.point_step == 32
    output_points = np.frombuffer(transformed.data, dtype=POINT_DTYPE)
    assert output_points['ring'].tolist() == [17, 23]
    assert output_points['intensity'].tolist() == [7.5, 8.5]
    np.testing.assert_allclose(output_points['time'], [0.01, 0.02])
    np.testing.assert_allclose(output_points['z'], [3.0 - 0.046825, 1.0 - 0.046825])
