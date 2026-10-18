"""Simulation-only contact-gated joint-target hold; normal policy remains unchanged."""
import json
from pathlib import Path
import math
import torch
from std_msgs.msg import String
import go2.go2_ctrl as go2_ctrl


class StanceAdapter:
    def __init__(self, env, bridge, enabled, artifact_dir, observation):
        self.env = env.unwrapped
        self.robot = self.env.scene['unitree_go2']
        self.contact = self.env.scene['contact_forces']
        assert self.env.num_envs == 1
        assert len(self.contact.body_names) == 4
        term = self.env.action_manager.get_term('joint_pos')
        assert term.action_dim == 12 and term.cfg.scale == .25
        self.enabled = enabled
        self.dt = float(self.env.step_dt)
        self.t = self.ready = self.bad = 0.0
        self.reference = None
        self.hold_from = None
        self.hold_elapsed = 0.
        self.held = None
        self.last = None
        self.mode = 'POLICY'
        self.resume_elapsed = .15
        self.resume_from = None
        self.publisher = bridge.create_publisher(String, '/nav_m1/stance_state', 10)
        self.foot_ids = [self.robot.body_names.index(name) for name in self.contact.body_names]
        policy_obs = observation if torch.is_tensor(observation) else observation['policy']
        contract = {'enabled': enabled, 'profile': 'calibrated_reference_v3', 'step_dt': self.dt, 'observation_shape': list(policy_obs.shape),
                    'observation_terms': self.env.observation_manager.active_terms,
                    'joint_names': self.robot.joint_names, 'foot_names': self.contact.body_names,
                    'action_scale': term.cfg.scale, 'default_joint_pos': self.robot.data.default_joint_pos[0].tolist(),
                    'height_observation_min_max': [float(policy_obs[0,48:].min()),float(policy_obs[0,48:].max())]}
        if artifact_dir:
            out=Path(artifact_dir);out.mkdir(parents=True,exist_ok=True)
            (out/'stance_contract.json').write_text(json.dumps(contract,indent=2)+'\n')

    def adapt(self, policy_actions):
        self.t += self.dt
        cmd = go2_ctrl.base_vel_cmd_input[0].detach().cpu().tolist()
        data = self.robot.data
        v = data.root_lin_vel_b[0].detach().cpu().tolist()
        w = data.root_ang_vel_b[0].detach().cpu().tolist()
        gravity = data.projected_gravity_b[0].detach().cpu().tolist()
        tilt = math.acos(max(-1., min(1., -gravity[2])))
        z = float(data.root_pos_w[0,2])
        jv = float(torch.sqrt(torch.mean(data.joint_vel[0]**2)))
        forces = torch.linalg.vector_norm(self.contact.data.net_forces_w[0],dim=-1).detach().cpu().tolist()
        speed = math.hypot(v[0],v[1])
        zero = max(abs(x) for x in cmd) < 1e-4
        # Conservative entry: do not freeze a leaning or unloaded gait pose.
        eligible = zero and speed < .03 and abs(w[2]) < .05 and tilt < .08 and .30 < z < .60 and jv < .15 and min(forces) > 15. and max(forces) / min(forces) < 3.
        # First calibrate a stable pose; subsequent stop transitions return to it.
        calibrating = self.reference is None
        if not calibrating:
            eligible = zero and speed < .12 and abs(w[2]) < .12 and tilt < .25 and .30 < z < .60 and jv < .8 and min(forces) > 8.
        self.ready = self.ready + self.dt if eligible else 0.
        reason = 'OBSERVE_ONLY' if not self.enabled else 'WAIT_SETTLE'
        output = policy_actions
        if self.enabled:
            if self.held is not None:
                if self.hold_elapsed < .50:
                    unhealthy = min(forces)<3. or speed>.2 or abs(w[2])>.3 or tilt>.35 or not .25<z<.65 or jv>2.
                else:
                    unhealthy = min(forces)<8. or speed>.08 or abs(w[2])>.15 or tilt>.15 or not .30<z<.60 or jv>.8
                self.bad = self.bad+self.dt if unhealthy else 0.
                if not zero or self.bad >= .10:
                    self.resume_from = self.last.clone()
                    self.resume_elapsed = 0.
                    self.held = None
                    self.ready = self.bad = 0.
                    reason = 'COMMAND_RESUME' if not zero else 'HOLD_HEALTH_RELEASE'
                else:
                    reason = 'CONTACT_HOLD'
            if self.held is None and self.ready >= (.30 if calibrating else .10) and self.resume_elapsed >= .15 and self.last is not None:
                if self.reference is None:
                    self.reference = self.last.clone()
                self.held = self.reference.clone()
                self.hold_from = self.last.clone()
                self.hold_elapsed = 0.
                self.bad = 0.
                reason = 'CONTACT_HOLD'
            if self.held is not None:
                self.hold_elapsed += self.dt
                alpha = min(1., self.hold_elapsed / .50)
                output = (1-alpha)*self.hold_from + alpha*self.held
                self.mode = 'HOLD'
            elif self.resume_from is not None and self.resume_elapsed < .15:
                self.resume_elapsed += self.dt
                alpha = min(1.,self.resume_elapsed/.15)
                output = (1-alpha)*self.resume_from + alpha*policy_actions
                self.mode = 'RESUME'
            else:
                self.mode = 'POLICY'
        self.last = output.detach().clone()
        msg=String()
        msg.data=json.dumps({'sim':self.t,'mode':self.mode,'reason':reason,'enabled':self.enabled,
                            'reference_calibrated': self.reference is not None, 'hold_blend_elapsed': self.hold_elapsed,
                            'command':cmd,'speed':speed,'wz':w[2],'height':z,'tilt':tilt,'joint_velocity_rms':jv,
                            'forces':forces,'eligible_time':self.ready,'output_actions':output[0].detach().cpu().tolist(),
                            'joint_pos':data.joint_pos[0].detach().cpu().tolist(),
                            'foot_pos_world':data.body_pos_w[0,self.foot_ids].detach().cpu().tolist()})
        self.publisher.publish(msg)
        return output
