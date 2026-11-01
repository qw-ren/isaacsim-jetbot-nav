"""Compare known offline true/estimated poses against AMCL's likelihood field.
No pose optimization, map shift, time shift, trajectory fitting, or online input.
Implements Humble likelihood_field score and coordinate macros for this map.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import yaml


def compare(case, parameter_file, map_dir=None):
    assert (case/'RUNNER_RESULT.json').exists()
    load=lambda name:[json.loads(line) for line in (case/name).read_text().splitlines() if line.strip()]
    truth={row['stamp']:row for row in load('truth_trajectory.jsonl')}
    tt=np.array(sorted(truth));tx=np.array([truth[t]['xyz'][:2] for t in tt])
    w,x,y,z=np.array([truth[t]['wxyz'] for t in tt]).T
    ty=np.unwrap(np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z)))
    nav={row['stamp']:row for row in load('navigation_feedback.jsonl')}
    nt=np.array(sorted(nav));nx=np.array([nav[t]['xy'] for t in nt]);ny=np.unwrap([nav[t]['yaw'] for t in nt])
    params=yaml.safe_load(parameter_file.read_text());params=next(iter(params.values()))['ros__parameters']
    assert params['laser_model_type']=='likelihood_field'
    map_dir=map_dir or case/'map'
    meta=yaml.safe_load((map_dir/'map.yaml').read_text());assert meta['origin'][2]==0 and meta['negate']==0
    image=map_dir/meta['image'];pixels=np.array(Image.open(image))[::-1]
    occupied=(255-pixels)/255.0>meta['occupied_thresh']
    distance=distance_transform_edt(~occupied)*meta['resolution']
    distance=np.minimum(distance,params['laser_likelihood_max_dist'])
    height,width=distance.shape
    scans=[]
    for path in sorted(case.glob('scan_*.npz')):
        data=np.load(path);t=float(data['stamp'])
        if not max(tt[0],nt[0])<=t<=min(tt[-1],nt[-1]):continue
        ranges=data['ranges'];assert len(ranges)==1440
        poses={'reference':np.r_[[np.interp(t,tt,tx[:,i]) for i in range(2)],np.interp(t,tt,ty)],
               'navigation_estimate':np.r_[[np.interp(t,nt,nx[:,i]) for i in range(2)],np.interp(t,nt,ny)]}
        entry=dict(snapshot=path.name,stamp=t,poses={key:value.tolist() for key,value in poses.items()},scores={})
        for beams in [90,360]:
            ids=np.arange(0,len(ranges),max(1,(len(ranges)-1)//(beams-1)))
            ids=ids[np.isfinite(ranges[ids])&(ranges[ids]<20.)]
            angles=-np.pi+ids*np.pi/720
            row={}
            for name,pose in poses.items():
                ends=np.column_stack([pose[0]+ranges[ids]*np.cos(pose[2]+angles),pose[1]+ranges[ids]*np.sin(pose[2]+angles)])
                # AMCL map origin uses integer size/2; terms cancel to ROS origin.
                cells=np.floor((ends-np.array(meta['origin'][:2]))/meta['resolution']+.5).astype(int)
                valid=(cells[:,0]>=0)&(cells[:,0]<width)&(cells[:,1]>=0)&(cells[:,1]<height)
                d=np.full(len(ids),params['laser_likelihood_max_dist'])
                d[valid]=distance[cells[valid,1],cells[valid,0]]
                pz=params['z_hit']*np.exp(-d*d/(2*params['sigma_hit']**2))+params['z_rand']/20.
                row[name]=dict(valid_beams=len(ids),score=float(1+np.sum(pz**3)),
                               mean_endpoint_distance_m=float(d.mean()),
                               zero_distance_fraction=float(np.mean(d==0)))
            entry['scores'][str(beams)]=row
        scans.append(entry)
    return dict(scope=__doc__,sensor_model={key:params[key] for key in ['laser_model_type','sigma_hit','z_hit','z_rand','laser_likelihood_max_dist']},
                snapshots=scans,limitations=['Only stored early scans; not the complete route.',
                  'Scores compare two recorded poses only; cannot isolate algorithm vs input error by themselves.',
                  'The navigation estimate includes AMCL plus estimated odometry, not a same-stamp raw AMCL update.'],
                sha256={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in [parameter_file,image,map_dir/'map.yaml',case/'truth_trajectory.jsonl',case/'navigation_feedback.jsonl',*sorted(case.glob('scan_*.npz'))]})


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--case',type=Path,required=True);p.add_argument('--amcl-params',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--map-dir',type=Path);a=p.parse_args()
    result=compare(a.case,a.amcl_params,a.map_dir)
    with a.output.open('x') as stream:json.dump(result,stream,indent=2);stream.write('\n')
    print(json.dumps(result['snapshots'],indent=2))
