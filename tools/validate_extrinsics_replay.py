"""Replay recorded sensors on a separate DDS domain; compare against body IMU.

Only raw cloud/IMU topics are published. No robot control topics are used.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation, Slerp
from sensor_msgs.msg import Imu, PointCloud2, PointField
from nav_msgs.msg import Odometry


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('session',type=Path)
    parser.add_argument('--baseline',action='store_true')
    parser.add_argument('--no-time-lag',action='store_true')
    parser.add_argument('--use-imu-as-input',action='store_true')
    parser.add_argument('--rate',type=float,default=1.5)
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    label='baseline' if args.baseline else ('rotation_only' if args.no_time_lag else 'calibrated')
    if args.use_imu_as_input:
        label += '_imu_input'
    # Domain and transport overrides are intentional: keep this replay local.
    os.environ['ROS_DOMAIN_ID']='179' if args.baseline else '180'
    os.environ['CYCLONEDDS_URI']='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" /></Interfaces></General></Domain></CycloneDDS>'
    data=np.load(args.session/'data.npz')
    metadata=json.loads((args.session/'metadata.json').read_text())
    rclpy.init();node=rclpy.create_node('extrinsics_replay_check')
    imu_pub=node.create_publisher(Imu,'/utlidar/imu',1000)
    cloud_pub=node.create_publisher(PointCloud2,'/utlidar/cloud',100)
    odometry=[];offset=[None];processed=[0]
    def odom_cb(m):
        p=m.pose.pose.position;q=m.pose.pose.orientation
        odometry.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,p.x,p.y,p.z,q.x,q.y,q.z,q.w])
    def cloud_cb(m):
        processed[0]+=1
        if offset[0] is None:
            offset[0]=m.header.stamp.sec+m.header.stamp.nanosec*1e-9-metadata['clouds'][0]['stamp']
    subs=[node.create_subscription(Odometry,'/state_estimation',odom_cb,qos_profile_sensor_data),
          node.create_subscription(PointCloud2,'/utlidar/transformed_cloud',cloud_cb,qos_profile_sensor_data)]
    log=args.session/(label+'_replay.log')
    handles=[];processes=[]
    config=root/'src/slam/point_lio_unilidar/config/utlidar.yaml'
    if args.baseline:
        config=args.session/'before_runtime_changes/utlidar.yaml'
        transform_command=['python3','-u',str(args.session/'before_runtime_changes/transform_everything.py')]
    else:
        transform_command=['ros2','run','transform_sensors','transform_everything','--ros-args','-p',
                           'extrinsics_file:='+str(root/'src/utilities/transform_sensors/config/l1_extrinsics.yaml')]
    if args.no_time_lag:
        parameters=yaml.safe_load(config.read_text())
        parameters['/**']['ros__parameters']['common']['time_lag_imu_to_lidar']=0.0
        config=args.session/'rotation_only_utlidar.yaml'
        config.write_text(yaml.safe_dump(parameters,sort_keys=False))
    slam_command=['ros2','run','point_lio_unilidar','pointlio_mapping','--ros-args','--params-file',str(config),
                  '-p',f'use_imu_as_input:={str(args.use_imu_as_input).lower()}','-p','init_map_size:=10',
                  '-p','filter_size_surf:=0.1','-p','filter_size_map:=0.1','-p','point_filter_num:=1',
                  '-r','/cloud_registered:=/registered_scan','-r','/aft_mapped_to_init:=/state_estimation']
    def set_stamp(m,t):
        ns=int(round(t*1e9));m.sec=ns//1000000000;m.nanosec=ns%1000000000
    try:
        for command in (transform_command,slam_command):
            handle=log.with_name(log.stem+'_'+str(len(handles))+'.log').open('w')
            handles.append(handle)
            processes.append(subprocess.Popen(command,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True))
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if imu_pub.get_subscription_count()>0 and cloud_pub.get_subscription_count()>0:
                break
        if not imu_pub.get_subscription_count() or not cloud_pub.get_subscription_count():
            raise RuntimeError('Replay input subscribers not discovered')
        imu=data['imu'];clouds=metadata['clouds']
        # Use receive order to faithfully preserve intra-scan delivery ordering.
        events=sorted([(float(x[0]),'imu',k) for k,x in enumerate(imu)]+
                      [(x['receive_time'],'cloud',k) for k,x in enumerate(clouds)])
        beginning=time.monotonic();report=10
        for acquisition,kind,k in events:
            target=beginning+acquisition/args.rate
            while time.monotonic()<target:
                rclpy.spin_once(node,timeout_sec=min(.01,max(0,target-time.monotonic())))
            if any(p.poll() is not None for p in processes):
                raise RuntimeError('Sensor/SLAM process exited during replay')
            if kind=='imu':
                row=imu[k];m=Imu();set_stamp(m.header.stamp,row[1]);m.header.frame_id='utlidar_imu'
                m.angular_velocity.x=float(row[2]);m.angular_velocity.y=float(row[3]);m.angular_velocity.z=float(row[4])
                m.linear_acceleration.x=float(row[5]);m.linear_acceleration.y=float(row[6]);m.linear_acceleration.z=float(row[7])
                m.orientation.x=float(row[8]);m.orientation.y=float(row[9]);m.orientation.z=float(row[10]);m.orientation.w=float(row[11])
                imu_pub.publish(m)
            else:
                record=clouds[k];a=data[record['key']]
                array=np.zeros(len(a),dtype={'names':['x','y','z','intensity','ring','time'],
                       'formats':['<f4','<f4','<f4','<f4','<u2','<f4'],
                       'offsets':[0,4,8,16,20,24],'itemsize':32})
                for i,key in enumerate(('x','y','z','intensity','ring','time')):array[key]=a[:,i]
                m=PointCloud2();set_stamp(m.header.stamp,record['stamp']);m.header.frame_id=record['frame']
                m.height=1;m.width=len(a);m.point_step=32;m.row_step=32*len(a);m.is_dense=True
                for name,field_offset,dtype in [('x',0,7),('y',4,7),('z',8,7),('intensity',16,7),('ring',20,4),('time',24,7)]:
                    f=PointField();f.name=name;f.offset=field_offset;f.datatype=dtype;f.count=1;m.fields.append(f)
                m.data=array.tobytes();cloud_pub.publish(m)
            rclpy.spin_once(node,timeout_sec=0.)
            if acquisition>=report:
                print(label,'replayed',round(acquisition,1),'seconds; odometry',len(odometry),flush=True);report+=10
        end=time.monotonic()+3
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
    finally:
        for process in processes:
            if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
        for process in processes:
            if process.poll() is None:
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=3)
        for handle in handles:handle.close()
        node.destroy_node();rclpy.shutdown()
        np.savez_compressed(args.session/(label+'_replay_odom.npz'),odom=np.asarray(odometry),timestamp_offset=offset)
    if not odometry or offset[0] is None:raise RuntimeError('No replay odometry')
    odom=np.asarray(odometry);source_times=odom[:,0]-offset[0]
    body=data['body'];times,ids=np.unique(body[:,1],return_index=True)
    valid=(source_times>=times[0])&(source_times<=times[-1])&np.isfinite(odom).all(axis=1)
    odom=odom[valid];source_times=source_times[valid]
    reference=Slerp(times-times[0],Rotation.from_quat(body[ids,8:12][:,[1,2,3,0]]))(source_times-times[0])
    estimated=Rotation.from_quat(odom[:,4:8])
    align=estimated[0]*reference[0].inv()
    errors=np.linalg.norm(((align*reference).inv()*estimated).as_rotvec(),axis=1)*180/np.pi
    elapsed=source_times-body[0,1]
    active=(elapsed>49)&(elapsed<70)
    yaw=Rotation.from_quat(body[ids,8:12][:,[1,2,3,0]])
    position=np.column_stack([np.interp(source_times,times,body[ids,k]) for k in range(15,18)])
    params=json.loads((args.session/'runtime_parameters.json').read_text())
    extrinsic=json.loads((args.session/'extrinsic_estimates.json').read_text())
    lever=(np.array(extrinsic['native']['translation_lidar_origin_in_body_m'])-
           np.array(extrinsic['dual_imu']['rotation_imu_to_body'])@np.array([.007698,.014655,-.00667]))
    sensor_positions=position+reference.apply(lever)
    expected_position=align.apply(sensor_positions-sensor_positions[0])+odom[0,1:4]
    position_error=np.linalg.norm(expected_position-odom[:,1:4],axis=1)
    summary={'label':label,'processed_clouds':processed[0],'odometry_messages':len(odometry),
             'finite_odometry_messages':int(valid.sum()),'motion_samples':int(active.sum()),
             'motion_attitude_rms_deg':float(np.sqrt(np.mean(errors[active]**2))),
             'motion_attitude_p95_deg':float(np.quantile(errors[active],.95)),
             'final_attitude_error_deg':float(errors[-1]),
             'motion_position_reference_rms_m':float(np.sqrt(np.mean(position_error[active]**2))),
             'motion_position_reference_p95_m':float(np.quantile(position_error[active],.95)),
             'final_position_reference_error_m':float(position_error[-1]),
             'body_reference_note':'Robot body odometry is an independent reference, not surveyed ground truth.'}
    (args.session/(label+'_replay_summary.json')).write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,indent=2),flush=True)


if __name__=='__main__':main()
