"""One fixed static box, toggled on the simulation thread for M3 sensing tests."""
import json
import math
from pathlib import Path
from pxr import UsdGeom, UsdPhysics, Gf
import omni.usd


class ObstacleFixture:
    def __init__(self, artifacts):
        self.root = Path(artifacts)
        self.root.mkdir(parents=True, exist_ok=True)
        config=json.loads((self.root/'box_config.json').read_text())
        self.center=config['center'];self.size=config['size']
        assert len(self.center)==len(self.size)==3 and all(math.isfinite(v) for v in self.center+self.size)
        assert all(.2<=v<=2. for v in self.size) and abs(self.center[2]-self.size[2]/2)<1e-6
        self.request = self.root / 'obstacle_request.json'
        self.response = self.root / 'obstacle_state.json'
        self.last_id = None
        self.set_enabled(False)

    def set_enabled(self, enabled):
        stage = omni.usd.get_context().get_stage()
        path = '/World/M3Obstacle'
        if stage.GetPrimAtPath(path).IsValid():
            stage.RemovePrim(path)
        if enabled:
            cube = UsdGeom.Cube.Define(stage, path)
            cube.CreateSizeAttr(1.)
            cube.AddTranslateOp().Set(Gf.Vec3d(*self.center))
            cube.AddScaleOp().Set(Gf.Vec3f(*self.size))
            cube.CreateDisplayColorAttr([Gf.Vec3f(1., .15, .05)])
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)

    def tick(self):
        if not self.request.exists():
            return
        command = json.loads(self.request.read_text())
        if command['id'] == self.last_id:
            return
        if type(command['enabled']) is not bool:
            raise ValueError('Obstacle enabled must be boolean')
        self.set_enabled(command['enabled'])
        self.last_id = command['id']
        state = dict(command, center=self.center, size=self.size)
        temporary = self.response.with_suffix('.tmp')
        temporary.write_text(json.dumps(state, indent=2) + '\n')
        temporary.replace(self.response)
        with (self.root / 'obstacle_events.jsonl').open('a') as stream:
            stream.write(json.dumps(state) + '\n')
