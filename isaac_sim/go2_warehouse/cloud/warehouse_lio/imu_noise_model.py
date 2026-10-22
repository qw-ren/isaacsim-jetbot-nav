"""Reproducible BMI088-like noise model; not a calibrated physical-unit model.

Density values: Bosch BMI088 datasheet rev 1.9, Jan 2024, pp9-10.
We use one-sided white PSD, ideal Nyquist bandwidth fs/2. Initial residual bias
and bias diffusion are explicit experiment assumptions, not datasheet claims.
"""
import numpy as np

DATASHEET = 'https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf'


class ImuNoiseModel:
    def __init__(self, seed=20261021, enabled=True):
        self.enabled = enabled
        self.seed = seed
        self.rng = np.random.default_rng(seed)
        self.acc_density = np.array([160., 160., 190.])*1e-6*9.80665
        self.gyr_density = np.full(3, np.deg2rad(.014))
        self.acc_bias = self.rng.normal(0., 5e-3*9.80665, 3)
        self.gyr_bias = self.rng.normal(0., np.deg2rad(.05), 3)
        self.initial_acc_bias = self.acc_bias.copy()
        self.initial_gyr_bias = self.gyr_bias.copy()
        self.acc_walk = 2e-4  # m/s^2 per sqrt(s), assumed scenario
        self.gyr_walk = 2e-5  # rad/s per sqrt(s), assumed scenario
        self.last = None

    def contract(self):
        return dict(enabled=self.enabled, seed=self.seed, reference=DATASHEET,
                    profile='BMI088-like; no device-specific calibration',
                    acc_density_mps2_per_sqrt_hz=self.acc_density.tolist(),
                    gyr_density_radps_per_sqrt_hz=self.gyr_density.tolist(),
                    discretization='one-sided PSD; independent samples at ideal Nyquist bandwidth 1/(2*dt)',
                    initial_acc_bias_mps2=self.initial_acc_bias.tolist(),
                    initial_gyr_bias_radps=self.initial_gyr_bias.tolist(),
                    assumed_acc_bias_walk_mps2_per_sqrt_s=self.acc_walk,
                    assumed_gyr_bias_walk_radps_per_sqrt_s=self.gyr_walk,
                    unmodeled=['temperature', 'scale factor', 'cross-axis', 'finite analog filter response'])

    def update(self, stamp, acceleration, angular_velocity):
        acceleration = np.asarray(acceleration, dtype=float)
        angular_velocity = np.asarray(angular_velocity, dtype=float)
        dt = .005 if self.last is None else stamp-self.last
        if not np.isfinite(np.r_[stamp, acceleration, angular_velocity]).all() or not 0. < dt <= .02:
            raise ValueError('Invalid IMU sample/time for noise model')
        self.last = stamp
        if not self.enabled:
            return acceleration.copy(), angular_velocity.copy(), np.zeros(3), np.zeros(3)
        self.acc_bias += self.rng.normal(0., self.acc_walk*np.sqrt(dt), 3)
        self.gyr_bias += self.rng.normal(0., self.gyr_walk*np.sqrt(dt), 3)
        acc_sigma = self.acc_density/np.sqrt(2*dt)
        gyr_sigma = self.gyr_density/np.sqrt(2*dt)
        return (acceleration+self.acc_bias+self.rng.normal(0., acc_sigma),
                angular_velocity+self.gyr_bias+self.rng.normal(0., gyr_sigma),
                acc_sigma**2, gyr_sigma**2)
