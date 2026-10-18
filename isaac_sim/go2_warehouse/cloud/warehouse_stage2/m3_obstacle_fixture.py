"""One fixed static box, toggled on the simulation thread for M3 sensing tests."""
import json
from pathlib import Path
from pxr import UsdGeom, UsdPhysics, Gf
import omni.usd


class ObstacleFixture:
    def __init__(self, artifacts):
        self.root = Path(artifacts)
        self.root.mkdir(parents=True, exist_ok=True)
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
            cube.AddTranslateOp().Set(Gf.Vec3d(4., -1., .5))
            cube.AddScaleOp().Set(Gf.Vec3f(.6, .6, 1.))
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
        state = dict(command, center=[4., -1., .5], size=[.6, .6, 1.])
        temporary = self.response.with_suffix('.tmp')
        temporary.write_text(json.dumps(state, indent=2) + '\n')
        temporary.replace(self.response)
        with (self.root / 'obstacle_events.jsonl').open('a') as stream:
            stream.write(json.dumps(state) + '\n')
