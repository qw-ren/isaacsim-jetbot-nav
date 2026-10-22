"""Apply an explicitly configured noise model to native IMU before LIO input."""
import copy
import csv
import json

from sensor_msgs.msg import Imu

from imu_noise_model import ImuNoiseModel
from route_trial_adapter import RouteTrialAdapter


class NoisyPublisher:
    def __init__(self, parent, node, root, model):
        self.parent = parent
        self.ideal = node.create_publisher(Imu, '/evaluation/imu_ideal', 200)
        self.model = model
        self.stream = (root/'imu_noisy.csv').open('x', newline='')
        self.csv = csv.writer(self.stream)
        self.csv.writerow(['stamp','ax','ay','az','gx','gy','gz','bax','bay','baz','bgx','bgy','bgz'])
        self.count = 0

    def publish(self, message):
        self.ideal.publish(message)
        output = copy.deepcopy(message)
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        a, g = message.linear_acceleration, message.angular_velocity
        acceleration, gyro, acc_var, gyro_var = self.model.update(stamp, [a.x,a.y,a.z], [g.x,g.y,g.z])
        output.linear_acceleration.x, output.linear_acceleration.y, output.linear_acceleration.z = map(float, acceleration)
        output.angular_velocity.x, output.angular_velocity.y, output.angular_velocity.z = map(float, gyro)
        output.linear_acceleration_covariance = [float(acc_var[i//3]) if i%4==0 else 0. for i in range(9)]
        output.angular_velocity_covariance = [float(gyro_var[i//3]) if i%4==0 else 0. for i in range(9)]
        self.parent.publish(output)
        self.csv.writerow([stamp,*acceleration,*gyro,*self.model.acc_bias,*self.model.gyr_bias])
        self.count += 1
        if self.count%200 == 0:
            self.stream.flush()


class NoisyRouteAdapter(RouteTrialAdapter):
    def attach(self):
        super().attach()
        self.noise_model = ImuNoiseModel(seed=int(getattr(self.node.cfg, 'imu_noise_seed', 20261021)),
                                       enabled=bool(getattr(self.node.cfg, 'imu_noise_enabled', True)))
        (self.root/'noise_contract.json').write_text(json.dumps(self.noise_model.contract(), indent=2)+'\n')
        self.publisher = NoisyPublisher(self.publisher, self.node, self.root, self.noise_model)

    def close(self):
        if isinstance(self.publisher, NoisyPublisher):
            self.publisher.stream.close()
        super().close()
