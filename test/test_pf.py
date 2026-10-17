import math
import pytest
from robot_localization.pf import ParticleFilter, Particle

class FakeParticleFilter:
    def __init__(self):
        self.n_particles = 300

        self.particle_stddev_x = 0.1
        self.particle_stddev_y = 0.1
        self.particle_stddev_theta = 0.1

        self.particle_cloud = []

    def normalize_particles(self):
        ParticleFilter.normalize_particles(self)

    def update_robot_pose(self):
        ParticleFilter.update_robot_pose(self)


@pytest.fixture
def fake_pf():
    return FakeParticleFilter()


def test_sample_length(fake_pf):
    ParticleFilter.initialize_particle_cloud(
        fake_pf,
        timestamp=None,
        xy_theta=(1.0, 2.0, 0.5),
    )

    assert len(fake_pf.particle_cloud) == fake_pf.n_particles


def test_cloud_values(fake_pf):
    ParticleFilter.initialize_particle_cloud(
        fake_pf,
        timestamp=None,
        xy_theta=(1.0, 2.0, 0.5),
    )

    for p in fake_pf.particle_cloud:
        assert isinstance(p, Particle)
        assert isinstance(p.x, (int, float))
        assert isinstance(p.y, (int, float))
        assert isinstance(p.theta, (int, float))

        assert math.isfinite(p.x)
        assert math.isfinite(p.y)
        assert math.isfinite(p.theta)


def test_normalization(fake_pf):
    ParticleFilter.initialize_particle_cloud(
        fake_pf,
        timestamp=None,
        xy_theta=(1.0, 2.0, 0.5),
    )

    assert sum(p.w for p in fake_pf.particle_cloud) == pytest.approx(1.0)

    for p in fake_pf.particle_cloud:
        assert p.w == pytest.approx(1.0 / fake_pf.n_particles)


@pytest.mark.parametrize(
    "raw_weights, expected_weights",
    [
        ([1, 1, 1], [1 / 3, 1 / 3, 1 / 3]),
        ([2, 3, 5], [0.2, 0.3, 0.5]),
        ([10, 0, 0], [1.0, 0.0, 0.0]),
        ([0.25, 0.25, 0.50], [0.25, 0.25, 0.50]),
    ],
)
def test_normalization_different_weights(
    fake_pf, raw_weights, expected_weights
):
    fake_pf.particle_cloud = [
        Particle(float(i), 0.0, 0.0, weight)
        for i, weight in enumerate(raw_weights)
    ]

    fake_pf.normalize_particles()

    actual_weights = [p.w for p in fake_pf.particle_cloud]

    assert actual_weights == pytest.approx(expected_weights)
    assert sum(actual_weights) == pytest.approx(1.0)



def test_robot_update(fake_pf): #only works if the ros2 related lines in update_robot_pose are commented out
    fake_pf.particle_cloud = [
        Particle(1.0, 2.0, 0.1, 1.0),
        Particle(3.0, 4.0, 0.3, 2.0),
        Particle(5.0, 6.0, 0.5, 1.0),
    ]

    fake_pf.update_robot_pose()

    assert fake_pf.robot_pose.position.x == pytest.approx(3.0)
    assert fake_pf.robot_pose.position.y == pytest.approx(4.0)
