"""Offline calibration diagnostics from independent body IMU and raw L1 clouds.

This writes estimates and validation metrics. It does not change live settings.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp


def fit_rotation(a, b):
    u, singular, vh = np.linalg.svd(a.T @ b)
    return vh.T @ np.diag([1., 1., np.linalg.det(vh.T @ u.T)]) @ u.T, singular


def native_transform(data, metadata):
    base = {x['stamp']:x for x in metadata['cloud_base']}
    rotations, translations, errors = [], [], []
    for record in metadata['clouds'][::35]:
        other = base.get(record['stamp'])
        if other is None:
            continue
        a = data[record['key']]
        b = data[other['key']]
        ids = np.searchsorted(a[:,5], b[:,5])
        keep = ids < len(a)
        ids, b = ids[keep], b[keep]
        keep = (abs(a[ids,5]-b[:,5])<1e-8) & (abs(a[ids,3]-b[:,3])<.1)
        a, b = a[ids[keep],:3].astype(float), b[keep,:3].astype(float)
        if len(a)<100:
            continue
        ac, bc = a.mean(0), b.mean(0)
        r, _ = fit_rotation(a-ac,b-bc)
        t = bc-r@ac
        rotations.append(r)
        translations.append(t)
        errors.extend(np.linalg.norm(a@r.T+t-b,axis=1))
    rotation = Rotation.from_matrix(rotations).mean().as_matrix()
    translation = np.median(translations,axis=0)
    return {'rotation_lidar_to_body':rotation.tolist(), 'translation_lidar_origin_in_body_m':translation.tolist(),
            'rpy_deg':Rotation.from_matrix(rotation).as_euler('xyz',degrees=True).tolist(),
            'scans':len(rotations), 'point_pairs':len(errors),
            'pair_error_p95_m':float(np.quantile(errors,.95)),
            'rotation_variation_deg':float(np.max(np.linalg.norm(Rotation.from_matrix(rotations).as_rotvec()-Rotation.from_matrix(rotation).as_rotvec(),axis=1))*180/np.pi),
            'scope':'firmware transform, verified against matched points; not an independent physical translation measurement'}


def dual_imu(data):
    imu, body = data['imu'], data['body']
    origin = imu[0,1]
    ti, tb = imu[:,1]-origin, body[:,1]-origin
    bi = np.median(imu[imu[:,0]<6,2:5],axis=0)
    bb = np.median(body[body[:,0]<6,2:5],axis=0)
    grid = np.arange(max(ti[0],tb[0])+.3,min(ti[-1],tb[-1])-.3,.01)
    a = gaussian_filter1d(np.column_stack([np.interp(grid,ti,imu[:,k]) for k in range(2,5)])-bi,2,axis=0)
    b = gaussian_filter1d(np.column_stack([np.interp(grid,tb,body[:,k]) for k in range(2,5)])-bb,2,axis=0)
    active = (np.linalg.norm(b,axis=1)>.035)&(grid>9)&(grid<70)
    train = active & (np.floor(grid)%2==0)
    test = active & ~train
    best = None
    for lag in np.arange(-.15,.151,.002):
        x = np.column_stack([np.interp(grid+lag,grid,a[:,k]) for k in range(3)])
        r,singular = fit_rotation(x[train],b[train])
        rms = np.sqrt(np.mean(np.linalg.norm(x[test]@r.T-b[test],axis=1)**2))
        if best is None or rms<best[0]:
            best = rms,lag,r,singular
    _,lag,r,singular = best

    def residual(parameters,mask):
        rotation = Rotation.from_rotvec(parameters[:3]).as_matrix()
        x = np.column_stack([np.interp(grid[mask]+parameters[3],grid,a[:,k]) for k in range(3)])
        return (x@rotation.T-b[mask]).ravel()

    result = least_squares(lambda p:residual(p,train),np.r_[Rotation.from_matrix(r).as_rotvec(),lag],
                           bounds=([-np.inf]*3+[-.15],[np.inf]*3+[.15]),loss='soft_l1',f_scale=.02)
    rotation = Rotation.from_rotvec(result.x[:3]).as_matrix()
    errors = residual(result.x,test).reshape(-1,3)
    angle = float(np.degrees(np.linalg.norm(Rotation.from_matrix(rotation).as_rotvec())))
    norms = np.linalg.norm(errors,axis=1)
    return {'rotation_imu_to_body':rotation.tolist(),'rpy_deg':Rotation.from_matrix(rotation).as_euler('xyz',degrees=True).tolist(),
            'raw_gyro_bias_rad_s':bi.tolist(),'body_gyro_bias_rad_s':bb.tolist(),
            'imu_timestamp_minus_body_timestamp_s':float(result.x[3]),
            'holdout_rms_rad_s':float(np.sqrt(np.mean(norms**2))), 'holdout_p95_rad_s':float(np.quantile(norms,.95)),
            'axis_rms_rad_s':np.sqrt(np.mean(errors**2,axis=0)).tolist(),
            'train_samples':int(train.sum()),'holdout_samples':int(test.sum()),
            'excitation_singular_values':singular.tolist(),
            'excitation_min_max_ratio':float(singular[-1]/singular[0]),
            'passed':bool(np.sqrt(np.mean(norms**2))<.06 and np.quantile(norms,.95)<.15 and singular[-1]/singular[0]>.01)}


def voxel(points,size=.08):
    points = points[np.isfinite(points).all(axis=1)]
    _,ids = np.unique(np.floor(points/size).astype(np.int64),axis=0,return_index=True)
    return points[ids]


def surface(points):
    points = voxel(points)
    tree = cKDTree(points)
    distances,ids = tree.query(points,k=16)
    local = points[ids]-points[ids].mean(axis=1,keepdims=True)
    eigen,vectors = np.linalg.eigh(np.einsum('nki,nkj->nij',local,local)/16)
    supported = (eigen[:,1]>.001)&(eigen[:,0]<.2*eigen[:,1])&(distances[:,-1]<.55)
    return points,tree,vectors[:,:,0],supported


def icp(points,target,initial):
    p,tree,normals,supported = target
    candidate = initial.copy()
    for iteration in range(20):
        moved = points@candidate[:3,:3].T+candidate[:3,3]
        distances,ids = tree.query(moved)
        keep = (distances<.35)&supported[ids]
        if keep.sum()<120:
            return None, {'reason':'insufficient overlap'}
        a,b,n = moved[keep],p[ids[keep]],normals[ids[keep]]
        residual = np.sum((a-b)*n,axis=1)
        weights = np.sqrt(np.minimum(1.,.025/np.maximum(abs(residual),1e-9)))
        matrix = np.column_stack((np.cross(a,n),n))*weights[:,None]
        singular = np.linalg.svd(matrix,compute_uv=False)
        if singular[-1]/singular[0]<.005:
            return None, {'reason':'degenerate surface geometry'}
        step = np.linalg.lstsq(matrix,-residual*weights,rcond=1e-4)[0]
        if np.linalg.norm(step[:3])>.12 or np.linalg.norm(step[3:])>.25:
            return None, {'reason':'ICP step too large'}
        dr = Rotation.from_rotvec(step[:3]).as_matrix()
        candidate[:3,3] = dr@candidate[:3,3]+step[3:]
        candidate[:3,:3] = dr@candidate[:3,:3]
        if np.linalg.norm(step)<.0001:
            break
    moved = points@candidate[:3,:3].T+candidate[:3,3]
    distance,ids = tree.query(moved)
    keep = (distance<.25)&supported[ids]
    residual = abs(np.sum((moved[keep]-p[ids[keep]])*normals[ids[keep]],axis=1))
    report={'pairs':int(keep.sum()),'overlap':float(keep.mean()),
            'plane_median_m':float(np.median(residual)),'plane_p95_m':float(np.quantile(residual,.95)),
            'condition':float(singular[0]/singular[-1]),'iterations':iteration+1}
    if keep.mean()<.35 or report['plane_p95_m']>.09:
        return None,dict(report,reason='large residual or low overlap')
    return candidate,report


def register(data,metadata,native,output):
    body = data['body']
    origin = body[0,1]
    times,unique = np.unique(body[:,1]-origin,return_index=True)
    q = body[unique,8:12][:,[1,2,3,0]]
    slerp = Slerp(times,Rotation.from_quat(q))
    def orientation(t):
        return slerp(np.clip(t,times[0],times[-1]))
    reference_time = 4.
    r0 = orientation(reference_time).as_matrix()
    cloud_rotation = np.array(native['rotation_lidar_to_body'])
    translation = np.array(native['translation_lidar_origin_in_body_m'])
    static = [data[x['key']][:,:3]@cloud_rotation.T+translation for x in metadata['clouds'] if 1<x['receive_time']<6]
    target = surface(np.concatenate(static))
    print('ICP target points:',len(target[0]),flush=True)
    samples=[]
    for index,record in enumerate(metadata['clouds'][::4]):
        receive = record['receive_time']
        if not 9<receive<71:
            continue
        raw = data[record['key']]
        source = raw[:,:3].astype(float)@cloud_rotation.T+translation
        # Motion correction uses only within-scan relative body rotations.
        # The global pose remains a free ICP variable fitted to geometry.
        point_times = record['stamp']-origin+raw[:,5].astype(float)
        time = float(np.mean(point_times))
        midpoint = orientation(time).as_matrix()
        rotations = orientation(point_times).as_matrix()
        source = np.einsum('nij,nj->ni',np.einsum('ij,njk->nik',midpoint.T,rotations),source)
        ranges = np.linalg.norm(source,axis=1)
        source=voxel(source[(ranges>.65)&(ranges<7)])
        initial=np.eye(4);initial[:3,:3]=r0.T@midpoint
        initial[:3,3]=r0.T@(np.array([np.interp(time,times,body[unique,k]) for k in range(15,18)])-
                                    np.array([np.interp(reference_time,times,body[unique,k]) for k in range(15,18)]))
        fitted,report=icp(source,target,initial)
        if fitted is not None:
            samples.append({'time':time,'receive_time':receive,'rotation':fitted[:3,:3].tolist(),
                            'translation':fitted[:3,3].tolist(),'metrics':report})
        if index%20==0:
            print('Registered',len(samples),'scans; time',round(receive,1),flush=True)
    (output/'registrations.json').write_text(json.dumps(samples,indent=2))
    return samples,r0


def pose_fit(data,samples,r0,native,dual):
    body = data['body'];origin=body[0,1]
    times,unique=np.unique(body[:,1]-origin,return_index=True)
    slerp=Slerp(times,Rotation.from_quat(body[unique,8:12][:,[1,2,3,0]]))
    t=np.array([x['time'] for x in samples])
    rc=Rotation.from_matrix([x['rotation'] for x in samples])
    train=(np.floor(t)%2==0);test=~train
    def residual(parameters,mask):
        correction=Rotation.from_rotvec(parameters[:3])
        body_r=Rotation.from_matrix(r0.T@slerp(np.clip(t[mask]+parameters[3],times[0],times[-1])).as_matrix())
        predicted=correction*body_r*correction.inv()
        return (predicted.inv()*rc[mask]).as_rotvec().ravel()
    fit=least_squares(lambda x:residual(x,train),np.zeros(4),loss='soft_l1',f_scale=.006,
                      bounds=([-.15]*3+[-.15],[.15]*3+[.15]))
    error=residual(fit.x,test).reshape(-1,3)
    angle=np.linalg.norm(error,axis=1)*180/np.pi
    print('Initial pose timing fit:',fit.x.tolist(),flush=True)
    rnative=np.array(native['rotation_lidar_to_body'])
    correction=Rotation.from_rotvec(fit.x[:3]).as_matrix()
    # rc = correction * native-body-relative * correction^-1;
    # a correction on observations corresponds to an inverse change of extrinsics.
    fitted_lidar_to_body=correction.T@rnative
    report={'scan_count':len(samples),'lidar_pose_time_plus_body_shift_s':float(fit.x[3]),
            'rotation_correction_deg':(fit.x[:3]*180/np.pi).tolist(),
            'rotation_lidar_to_body_candidate':fitted_lidar_to_body.tolist(),
            'holdout_angle_median_deg':float(np.median(angle)),
            'holdout_angle_p95_deg':float(np.quantile(angle,.95)),
            'holdout_angle_rms_deg':float(np.sqrt(np.mean(angle**2))),
            'status':'candidate; timing uncertainty and split checks required before deployment'}
    subsets={}
    for label,mask in (('first_half',t<np.median(t)),('second_half',t>=np.median(t)),
                       ('posture',t<49),('variable_yaw',t>=49)):
        subset=least_squares(lambda x:residual(x,mask),fit.x,loss='soft_l1',f_scale=.006,
                             bounds=([-.15]*3+[-.15],[.15]*3+[.15]))
        subsets[label]={'samples':int(mask.sum()),'time_shift_s':float(subset.x[3]),
                        'rotation_correction_deg':(subset.x[:3]*180/np.pi).tolist(),
                        'rms_deg':float(np.sqrt(np.mean(np.linalg.norm(residual(subset.x,mask).reshape(-1,3),axis=1)**2))*180/np.pi)}
    report['split_estimates']=subsets
    # Resample whole 1 s blocks: samples within a scan trajectory are correlated.
    rng=np.random.default_rng(11)
    blocks=np.unique(np.floor(t))
    boot=[]
    for _ in range(24):
        selected=rng.choice(blocks,len(blocks),replace=True)
        ids=np.concatenate([np.flatnonzero(np.floor(t)==block) for block in selected])
        subset=least_squares(lambda x:residual(x,ids),fit.x,loss='soft_l1',f_scale=.006,
                             bounds=([-.15]*3+[-.15],[.15]*3+[.15]),max_nfev=100)
        boot.append(subset.x)
    boot=np.asarray(boot)
    report['block_bootstrap_time_shift_interval_s']=np.quantile(boot[:,3],[.025,.975]).tolist()
    report['block_bootstrap_rotation_interval_deg']=(np.quantile(boot[:,:3],[.025,.975],axis=0)*180/np.pi).tolist()
    # Joint hand-eye translation from independent robot body positions and ICP.
    body_rotation=Rotation.from_matrix(r0.T@slerp(t+fit.x[3]).as_matrix()).as_matrix()
    body_position=np.column_stack([np.interp(t+fit.x[3],times,body[unique,k]) for k in range(15,18)])
    ref_position=np.array([np.interp(4.,times,body[unique,k]) for k in range(15,18)])
    body_translation=(body_position-ref_position)@r0
    cloud_translation=np.array([x['translation'] for x in samples])
    cloud_to_body=correction.T
    lhs=np.eye(3)-body_rotation
    rhs=body_translation-cloud_translation@cloud_to_body.T
    shift=np.linalg.lstsq(lhs[train].reshape(-1,3),rhs[train].ravel(),rcond=None)[0]
    trans_error=np.linalg.norm(np.einsum('nij,j->ni',lhs[test],shift)-rhs[test],axis=1)
    translation_singular=np.linalg.svd(lhs[train].reshape(-1,3),compute_uv=False)
    trans_boot=[]
    for _ in range(24):
        selected=rng.choice(blocks,len(blocks),replace=True)
        ids=np.concatenate([np.flatnonzero(np.floor(t)==block) for block in selected])
        trans_boot.append(np.linalg.lstsq(lhs[ids].reshape(-1,3),rhs[ids].ravel(),rcond=None)[0])
    translation_candidate=cloud_to_body@np.array(native['translation_lidar_origin_in_body_m'])+shift
    trans_boot=np.asarray(trans_boot)
    uncertainty=np.quantile(trans_boot,[.025,.975],axis=0)
    report['translation_fit']={'lidar_origin_in_body_candidate_m':translation_candidate.tolist(),
                'correction_m':shift.tolist(),'holdout_rms_m':float(np.sqrt(np.mean(trans_error**2))),
                'holdout_p95_m':float(np.quantile(trans_error,.95)),
                'singular_values':translation_singular.tolist(),
                'correction_block_bootstrap_interval_m':uncertainty.tolist(),
                'passed':bool(np.linalg.norm(shift)<.1 and np.max(uncertainty[1]-uncertainty[0])<.04 and np.sqrt(np.mean(trans_error**2))<.025)}
    before=residual(np.zeros(4),test).reshape(-1,3)
    report['default_holdout_angle_rms_deg']=float(np.sqrt(np.mean(np.linalg.norm(before,axis=1)**2))*180/np.pi)
    report['rotation_lidar_to_imu_candidate']=(np.array(dual['rotation_imu_to_body']).T@fitted_lidar_to_body).tolist()
    report['clock_relation']='L1 IMU and body stamps use the same epoch. IMU stamp minus physical body time = dual_imu shift. For Point-LIO subtract (lidar_pose shift + dual_imu shift) from IMU timestamps.'
    report['pointlio_time_lag_imu_to_lidar_candidate_s']=float(fit.x[3]+dual['imu_timestamp_minus_body_timestamp_s'])
    report['rotation_timing_passed']=bool(np.quantile(angle,.95)<1.5 and
                abs(subsets['variable_yaw']['time_shift_s']-fit.x[3])<.015 and
                np.ptp(report['block_bootstrap_time_shift_interval_s'])<.025)
    return report


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('session',type=Path)
    parser.add_argument('--skip-registration',action='store_true')
    parser.add_argument('--reuse-registration',action='store_true')
    args=parser.parse_args()
    data=np.load(args.session/'data.npz')
    metadata=json.loads((args.session/'metadata.json').read_text())
    native=native_transform(data,metadata)
    dual=dual_imu(data)
    report={'native':native,'dual_imu':dual}
    (args.session/'extrinsic_estimates.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2),flush=True)
    if not args.skip_registration:
        if args.reuse_registration:
            samples=json.loads((args.session/'registrations.json').read_text())
            b=data['body'];t,ids=np.unique(b[:,1]-b[0,1],return_index=True)
            r0=Slerp(t,Rotation.from_quat(b[ids,8:12][:,[1,2,3,0]]))(4.).as_matrix()
        else:
            samples,r0=register(data,metadata,native,args.session)
        report['lidar_pose']=pose_fit(data,samples,r0,native,dual)
        (args.session/'extrinsic_estimates.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report['lidar_pose'],indent=2),flush=True)


if __name__=='__main__':
    main()
