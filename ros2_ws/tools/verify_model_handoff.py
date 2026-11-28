#!/usr/bin/env python3
"""Offline checkpoint and recorded planar-stream identity verification; no ROS."""
import argparse
import hashlib
import json
from pathlib import Path

from check_planar_stream import validate_message


def verify(weights, manifest_path, observations=None):
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    digest = hashlib.sha256()
    with weights.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            digest.update(chunk)
    if digest.hexdigest() != manifest['sha256'] or weights.stat().st_size != manifest['size_bytes']:
        raise ValueError('checkpoint SHA256/size differs from manifest')
    classes = manifest['classes']
    if len(classes) != 8 or len(set(classes)) != 8:
        raise ValueError('expected eight distinct class names in ID order')
    count, objects, previous, runtimes = 0, 0, {}, {}
    if observations is not None:
        with observations.open(encoding='utf-8') as stream:
            for number, line in enumerate(stream, 1):
                if not line.strip():
                    continue
                try:
                    row = json.loads(line)
                    validate_message(row, 'map')
                    model = row['model']
                    if (model['weights_sha256'] != manifest['sha256'] or model['classes'] != classes
                            or model['inference_version'] != manifest['inference_version']):
                        raise ValueError('observation model identity differs from manifest')
                    if row['time_domain'] not in ('ros_sim', 'ros_system'):
                        raise ValueError('unknown acquisition clock domain')
                    runtime = model['runtime']
                    if any(not isinstance(runtime.get(p), str) or not runtime[p] for p in ('torch', 'ultralytics')):
                        raise ValueError('missing inference package version')
                    session = row['session_id']
                    if session in previous:
                        sequence, stamp, domain = previous[session]
                        if row['sequence'] <= sequence or row['stamp'] <= stamp or row['time_domain'] != domain:
                            raise ValueError('capture sequence/time/domain changed within session')
                        if runtime != runtimes[session]:
                            raise ValueError('inference package versions changed within session')
                    previous[session] = row['sequence'], row['stamp'], row['time_domain']
                    runtimes[session] = runtime
                    if any(obj['class_name'] not in classes for obj in row['objects']):
                        raise ValueError('class outside eight-class manifest')
                    count += 1
                    objects += len(row['objects'])
                except (ValueError, KeyError, TypeError, AttributeError) as exc:
                    raise ValueError('observation line %d: %s' % (number, exc)) from exc
        if not count:
            raise ValueError('observation file contains no packets')
    return dict(checkpoint_verified=True, weights_sha256=manifest['sha256'], classes=classes,
                inference_version=manifest['inference_version'], packets=count, objects=objects,
                session_runtimes=runtimes, scope='artifact identity and recorded wire contract; no live freshness, detection accuracy or target position accuracy check')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--weights', type=Path, required=True)
    ap.add_argument('--manifest', type=Path, required=True)
    ap.add_argument('--observations', type=Path)
    args = ap.parse_args()
    try:
        result = verify(args.weights, args.manifest, args.observations)
    except (ValueError, KeyError, OSError) as exc:
        ap.exit(1, str(exc)+'\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
