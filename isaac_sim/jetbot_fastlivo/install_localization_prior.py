"""Verify and install the packaged static localization prior, without overwrites.
Uses only Python's standard library. Does not modify the planning map or USD.
"""
import argparse
import hashlib
import io
import json
from pathlib import Path
import tarfile


def require(condition, message):
    if not condition:
        raise ValueError(message)


def install(archive, output):
    expected=archive.with_name(archive.name+'.sha256').read_text().strip()
    payload=archive.read_bytes();require(hashlib.sha256(payload).hexdigest()==expected,'Archive SHA256 mismatch')
    files={}
    with tarfile.open(fileobj=io.BytesIO(payload),mode='r:gz') as tar:
        for member in tar:
            require(member.isfile() and member.name in ('map.pgm','map.yaml','MAP_PROVENANCE.json','checksums.json'),'Unexpected archive entry')
            require(member.name not in files and member.size<=16_000_000,'Duplicate or oversized archive entry')
            files[member.name]=tar.extractfile(member).read()
    checks=json.loads(files.pop('checksums.json'));require(set(files)==set(checks)=={'map.pgm','map.yaml','MAP_PROVENANCE.json'},'Unexpected manifest files')
    for name,data in files.items():require(hashlib.sha256(data).hexdigest()==checks[name],name+' digest mismatch')
    proof=json.loads(files['MAP_PROVENANCE.json'])
    require(proof['source_geometry_unchanged'] is True and proof['truth_or_trajectory_input'] is False,'Unexpected geometry provenance')
    require(proof['type']=='visible CAD mesh surface projection for localization only','Unexpected map type')
    if output.exists():
        require(output.is_dir() and not output.is_symlink(),'Output is not an ordinary directory')
        for name,digest in checks.items():
            p=output/name
            require(p.is_file() and not p.is_symlink() and hashlib.sha256(p.read_bytes()).hexdigest()==digest,'Existing map differs; refusing overwrite')
        return dict(output=str(output),status='existing map verified',archive_sha256=expected)
    output.mkdir(parents=True,exist_ok=False)
    for name,data in files.items():
        with (output/name).open('xb') as stream:stream.write(data)
    return dict(output=str(output),status='installed and verified',archive_sha256=expected)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--archive',type=Path,default=Path(__file__).parent/'priors/localization_mesh_002_20261030.tar.gz')
    p.add_argument('--output',type=Path,default=Path('/home/admin/jetbot-warehouse/map_candidates/localization_mesh_002_20261030'))
    a=p.parse_args();print(json.dumps(install(a.archive,a.output),indent=2))
