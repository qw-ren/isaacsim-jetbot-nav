"""Stationary C fixture: four physical walls enclose the B-map start.

The original prior divider is retained. The original B barrier stays absent.
Requests are applied on the simulator main thread, through ``tick``.
"""
import json
from pathlib import Path

import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics

from branch_obstacle_fixture import ObstacleFixture as BranchFixture


WALLS = [
    {"name": "east", "center": [3., 0., .5], "size": [.3, 2.7, 1.]},
    {"name": "west", "center": [-.5, 0., .5], "size": [.3, 2.7, 1.]},
    {"name": "north", "center": [1.25, 1.2, .5], "size": [3.8, .3, 1.]},
    {"name": "south", "center": [1.25, -1.2, .5], "size": [3.8, .3, 1.]},
]


class ObstacleFixture(BranchFixture):
    def __init__(self, artifacts):
        super().__init__(artifacts)
        self.enclosure_enabled = False
        config = {"scope": "stationary_start_enclosure_not_remote_two_passages",
                  "walls": WALLS, "original_B_barrier_enabled": False}
        path = self.root / "enclosure_config.json"
        if path.exists() and json.loads(path.read_text()) != config:
            raise RuntimeError("Different enclosure configuration already exists")
        path.write_text(json.dumps(config, indent=2) + "\n")
        self.set_enclosure(True)
        self.record({"id": "enclosure_initial", "enclosure_enabled": True})

    @staticmethod
    def wall_path(wall):
        return "/World/CStartEnclosure_" + wall["name"]

    def set_enclosure(self, enabled):
        stage = omni.usd.get_context().get_stage()
        # Never enable the B barrier in this independent experiment.
        self.set_enabled(False)
        for wall in WALLS:
            path = self.wall_path(wall)
            if stage.GetPrimAtPath(path).IsValid():
                if not stage.RemovePrim(path):
                    raise RuntimeError("Could not remove enclosure prim: " + path)
            if enabled:
                cube = UsdGeom.Cube.Define(stage, path)
                cube.CreateSizeAttr(1.)
                cube.AddTranslateOp().Set(Gf.Vec3d(*wall["center"]))
                cube.AddScaleOp().Set(Gf.Vec3f(*wall["size"]))
                cube.CreateDisplayColorAttr([Gf.Vec3f(.8, .25, .08)])
                UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)
        self.enclosure_enabled = enabled

    def record(self, command):
        stage = omni.usd.get_context().get_stage()
        actual = []
        for wall in WALLS:
            prim = stage.GetPrimAtPath(self.wall_path(wall))
            present = prim.IsValid()
            collision = bool(UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()) if present else False
            actual.append(dict(wall, prim_path=self.wall_path(wall), present=present,
                               collision_enabled=collision))
        state = dict(command, enabled=False, center=self.center, size=self.size,
                     prior_divider=self.divider, enclosure_walls=actual,
                     enclosure_enabled=self.enclosure_enabled,
                     original_B_barrier_present=stage.GetPrimAtPath("/World/M3Obstacle").IsValid())
        expected = self.enclosure_enabled
        if any(w["present"] != expected or w["collision_enabled"] != expected for w in actual):
            raise RuntimeError("Enclosure USD state differs from requested state")
        if state["original_B_barrier_present"]:
            raise RuntimeError("B barrier unexpectedly present")
        temporary = self.response.with_suffix(".tmp")
        temporary.write_text(json.dumps(state, indent=2) + "\n")
        temporary.replace(self.response)
        enclosure_state = self.root / "enclosure_state.json"
        temporary = enclosure_state.with_suffix(".tmp")
        temporary.write_text(json.dumps(state, indent=2) + "\n")
        temporary.replace(enclosure_state)
        with (self.root / "enclosure_events.jsonl").open("a") as stream:
            stream.write(json.dumps(state) + "\n")

    def tick(self):
        if not self.request.exists():
            return
        command = json.loads(self.request.read_text())
        if command["id"] == self.last_id:
            return
        if type(command.get("enclosure_enabled")) is not bool:
            raise ValueError("C fixture requires boolean enclosure_enabled")
        if command.get("enabled", False) is not False:
            raise ValueError("B barrier must remain disabled")
        self.set_enclosure(command["enclosure_enabled"])
        self.record(command)
        self.last_id = command["id"]
