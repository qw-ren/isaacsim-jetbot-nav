import json
from pathlib import Path
import tempfile
import unittest

from run_navigation_demo import load_route


class RouteContract(unittest.TestCase):
    def test_requires_surveyed_map_route(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "route.json"
            for route in ({}, {"frame_id": "map", "map_sha256": "test", "waypoints": [[0, 0, 0]]},
                          {"frame_id": "map", "map_sha256": "test", "waypoints": [[0, 0, 0], [1, float('nan'), 0]]}):
                path.write_text(json.dumps(route))
                with self.assertRaises(ValueError):
                    load_route(path)
            route = dict(frame_id="map", map_sha256="test", waypoints=[[0, 0, 0], [1, 0, 0]])
            path.write_text(json.dumps(route))
            self.assertEqual(load_route(path), route)


if __name__ == "__main__":
    unittest.main()
