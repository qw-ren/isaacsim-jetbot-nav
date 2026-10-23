"""Same map-frame safety limits, with acknowledged inserted geometry added."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException

sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'warehouse_lio'))
from lio_map_velocity_guard import LioMapVelocityGuard


class DynamicVelocityGuard(LioMapVelocityGuard):
    def __init__(self,boxes,fixture):
        super().__init__(boxes)
        self.original_boxes=self.boxes.copy()
        self.fixture=fixture
        self.fixture_id=None

    def fault(self,now):
        try:
            d=json.loads(self.fixture.read_text())
            if type(d['enabled']) is not bool:raise ValueError('Bad fixture enabled state')
            if d['id']!=self.fixture_id:
                values=np.array([[b['min'][:2],b['max'][:2]] for b in d['dynamic_boxes']],dtype=float).reshape(-1,2,2)
                if not np.isfinite(values).all() or np.any(values[:,1,:]<values[:,0,:]):raise ValueError('Bad fixture bounds')
                if bool(len(values))!=d['enabled']:raise ValueError('Inconsistent actual obstacle state')
                self.boxes=np.concatenate([self.original_boxes,values],axis=0)
                self.fixture_id=d['id']
        except Exception:
            return 'FIXTURE_STATE_INVALID'
        return super().fault(now)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--boxes',type=Path,required=True)
    parser.add_argument('--fixture',type=Path,required=True)
    args=parser.parse_args()
    rclpy.init(args=[])
    node=DynamicVelocityGuard(json.loads(args.boxes.read_text()),args.fixture)
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
