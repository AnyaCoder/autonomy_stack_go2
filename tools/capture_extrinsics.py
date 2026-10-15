"""Record raw L1/body IMU and clouds during a bounded calibration motion."""
import argparse
import json
import math
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from unitree_api.msg import Request, Response
from unitree_go.msg import SportModeState


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--static-only', action='store_true')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    output = args.output or Path('log') / ('extrinsics_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
    output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node('extrinsics_capture')
    publisher = node.create_publisher(Request, '/api/sport/request', 10)
    start = time.monotonic()
    imu, body, clouds, base, responses, commands, phases = [], [], [], [], [], [], []
    last_imu = last_body = None

    def stamp(m):
        return m.sec + m.nanosec * 1e-9

    def imu_cb(m):
        nonlocal last_imu
        last_imu = time.monotonic()
        imu.append([last_imu-start, stamp(m.header.stamp), m.angular_velocity.x,
                    m.angular_velocity.y, m.angular_velocity.z, m.linear_acceleration.x,
                    m.linear_acceleration.y, m.linear_acceleration.z,
                    m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w])

    def body_cb(m):
        nonlocal last_body
        last_body = time.monotonic()
        i = m.imu_state
        body.append([last_body-start, stamp(m.stamp), *i.gyroscope, *i.accelerometer,
                     *i.quaternion, *i.rpy, *m.position, *m.velocity, m.yaw_speed,
                     int(m.mode), m.body_height])

    def cloud_cb(m, collection):
        fields = {f.name: f for f in m.fields}
        names = [k for k in ('x','y','z','intensity','ring','time') if k in fields]
        types = {1:'i1', 2:'u1', 3:'<i2', 4:'<u2', 5:'<i4', 6:'<u4', 7:'<f4', 8:'<f8'}
        dtype = np.dtype({'names':names, 'formats':[types[fields[k].datatype] for k in names],
                          'offsets':[fields[k].offset for k in names], 'itemsize':m.point_step})
        a = np.ndarray((m.height,m.width), dtype=dtype, buffer=m.data,
                       strides=(m.row_step,m.point_step))
        collection.append((time.monotonic()-start, stamp(m.header.stamp), m.header.frame_id,
                           names, np.column_stack([a[k].ravel() for k in names]).astype(np.float32)))

    def response_cb(m):
        responses.append([time.monotonic()-start, m.header.identity.api_id,
                          m.header.status.code, m.data])

    subscriptions = [
        node.create_subscription(Imu, '/utlidar/imu', imu_cb, 300),
        node.create_subscription(SportModeState, '/sportmodestate', body_cb, qos_profile_sensor_data),
        node.create_subscription(PointCloud2, '/utlidar/cloud', lambda m:cloud_cb(m,clouds), qos_profile_sensor_data),
        node.create_subscription(PointCloud2, '/utlidar/cloud_base', lambda m:cloud_cb(m,base), qos_profile_sensor_data),
        node.create_subscription(Response, '/api/sport/response', response_cb, 10),
    ]

    def send(api, values=None):
        request = Request()
        request.header.identity.api_id = api
        if values is not None:
            request.parameter = json.dumps(dict(zip(('x','y','z'), values)))
        publisher.publish(request)
        commands.append([time.monotonic()-start, api, values])

    moving = False
    ok = False
    try:
        deadline = time.monotonic()+8
        while time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.03)
            if len(imu)>500 and len(body)>500 and len(clouds)>20:
                break
        if len(imu)<500 or len(body)<500 or len(clouds)<20:
            raise RuntimeError('Insufficient live data; no motion started')
        if abs(body[-1][-3])>.1 or np.linalg.norm(body[-1][18:21])>.05:
            raise RuntimeError('Robot is not stationary; no motion started')
        if not args.static_only and node.count_subscribers('/api/sport/request')<1:
            raise RuntimeError('Sport control subscriber missing')
        print('Preflight passed; capturing at '+str(output.resolve()), flush=True)
        sequence = [('static',6), ('balance',2), ('roll',12), ('pitch',12),
                    ('combined',12), ('restore',4), ('yaw',20), ('stop',5)]
        if args.static_only:
            sequence = [('static',8)]
        for phase,duration in sequence:
            print('Phase: '+phase, flush=True)
            phase_start=time.monotonic()
            phases.append([phase, phase_start-start, duration])
            last_command=0
            last_report=phase_start
            while time.monotonic()-phase_start<duration:
                now=time.monotonic()
                rclpy.spin_once(node,timeout_sec=.005)
                if now-last_imu>.7 or now-last_body>.7:
                    raise RuntimeError('Live telemetry interrupted')
                if now-last_report>=4:
                    print(json.dumps({'phase':phase, 'yaw_speed':body[-1][21],
                                      'rpy_deg':np.degrees(body[-1][12:15]).tolist()}), flush=True)
                    last_report=now
                if not args.static_only and phase!='static' and now-last_command>=.01:
                    moving=True
                    t=now-phase_start
                    envelope=math.sin(math.pi*t/duration)**2
                    if phase=='balance': send(1002)
                    elif phase in ('roll','pitch','combined'):
                        roll=.10*envelope*math.sin(2*math.pi*.35*t) if phase in ('roll','combined') else 0.
                        pitch=.12*envelope*math.sin(2*math.pi*.27*t) if phase in ('pitch','combined') else 0.
                        send(1007,[roll,pitch,0.])
                    elif phase=='restore':
                        send(1007,[0.,0.,0.])
                    elif phase=='yaw':
                        send(1008,[0.,0.,1.396])
                    elif phase=='stop': send(1003)
                    last_command=now
        ok=True
    finally:
        if moving:
            for _ in range(20):
                send(1003)
                rclpy.spin_once(node,timeout_sec=.04)
        arrays={'imu':np.asarray(imu),'body':np.asarray(body)}
        metadata={'completed':ok,'static_only':args.static_only,'responses':responses,
                  'commands':commands, 'phases':phases, 'clouds':[], 'cloud_base':[]}
        for prefix,collection in (('clouds',clouds),('cloud_base',base)):
            for k,(recv,source,frame,names,points) in enumerate(collection):
                key=prefix+'_'+str(k)
                arrays[key]=points
                metadata[prefix].append({'key':key,'receive_time':recv,'stamp':source,
                                         'frame':frame,'fields':names})
        np.savez_compressed(output/'data.npz',**arrays)
        (output/'metadata.json').write_text(json.dumps(metadata,indent=2))
        report={'completed':ok,'imu':len(imu),'body':len(body),'clouds':len(clouds),
                'cloud_base':len(base),'output':str(output.resolve())}
        print(json.dumps(report),flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
