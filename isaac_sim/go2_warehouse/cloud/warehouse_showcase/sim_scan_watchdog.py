"""Simulation showcase patch: scan age in physical sim time plus a hard wall cap.

Other stream watchdogs, geometry, posture, exclusivity and latch stay untouched.
This never changes message timestamps and is not a real-robot configuration.
"""
import ast

def patch_guard(source):
    old='''        for key, timeout in (("odom", .5), ("scan", .5), ("clock", .5), ("input", .35)):
            if now - self.last[key] > timeout:
                return key.upper() + "_TIMEOUT"'''
    new='''        for key, timeout in (("odom", .5), ("scan", 1.0), ("clock", .5), ("input", .35)):
            if key == "scan":
                # .20 sim seconds bounds travelled distance; 1.0 wall second is a hard cap.
                # Clock/odom/input retain their independent .5/.5/.35 wall deadlines.
                if self.sim is None or self.scan_stamp is None or now-self.last[key] > timeout or self.sim-self.scan_stamp > .20:
                    return "SCAN_TIMEOUT"
            elif now - self.last[key] > timeout:
                return key.upper() + "_TIMEOUT"'''
    if source.count(old)!=1:raise ValueError('Unexpected guard source: refusing patch')
    source=source.replace(old,new)
    source=source.replace('"scan_stamp": self.scan_stamp,','"scan_wall_age_s": time.monotonic()-self.last["scan"], "scan_sim_age_s": self.sim-self.scan_stamp if self.sim is not None and self.scan_stamp is not None else None, "scan_wall_limit_s": 1.0, "scan_sim_limit_s": .20,\n                                  "scan_stamp": self.scan_stamp,')
    ast.parse(source);return source
