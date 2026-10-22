"""Wait for an observer-generated readiness file with a fixed wall deadline."""
import argparse,time,json
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('path',type=Path);p.add_argument('--timeout',type=float,default=150);a=p.parse_args();deadline=time.monotonic()+a.timeout
while time.monotonic()<deadline:
 if a.path.exists():print(json.dumps(json.loads(a.path.read_text())));break
 time.sleep(.2)
else:raise SystemExit('Readiness deadline expired; no dependent task started')
