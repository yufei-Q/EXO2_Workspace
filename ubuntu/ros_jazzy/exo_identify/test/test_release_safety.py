from pathlib import Path
import sys

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / 'scripts'))

from collision import (  # noqa: E402
    collision_model_fingerprint,
    collision_settings,
    CollisionChecker,
)
import numpy as np  # noqa: E402
from trajectory_experiment import TrajectoryExperimentNode  # noqa: E402


class _Logger:
    def info(self, _message):
        pass


def _hold_node(disable_on_finish):
    node = TrajectoryExperimentNode.__new__(TrajectoryExperimentNode)
    node.state = 'stopping'
    node.joint_count = 7
    node.hold_q = np.zeros(7)
    node.desired_q = np.zeros(7)
    node.desired_dq = np.ones(7)
    node.desired_ddq = np.ones(7)
    node.hold_after_stop = 0.5
    node.disable_on_finish = disable_on_finish
    node.disable_deadline = None
    node._now = lambda: 10.0
    node._write_records = lambda: None
    node._feedback_is_fresh = lambda: True
    node._publish_joint_command = lambda _q, _dq: True
    node.get_logger = lambda: _Logger()
    node.ready_messages = []
    node.enable_messages = []
    node._publish_trajectory_ready = node.ready_messages.append
    node._publish_enable = node.enable_messages.append
    return node


def test_committed_trajectory_is_collision_free():
    results = PACKAGE_ROOT / 'results' / 'seven_dof'
    trajectory = np.genfromtxt(
        results / 'excitation' / 'excitation_id.csv',
        delimiter=',',
        names=True,
    )
    positions = np.column_stack([
        trajectory[f'q{index}'] for index in range(1, 8)
    ])
    checker = CollisionChecker(
        results / 'simulation_model' / 'exo7_sim.xml', 7, 0.005)

    result = checker.check_positions(positions, trajectory['t'])

    assert result.collision_free
    assert result.checked_samples == positions.shape[0]


def test_collision_clearance_must_be_non_negative():
    with np.testing.assert_raises(ValueError):
        collision_settings({'collision': {'minimum_clearance_m': -0.001}})


def test_collision_fingerprint_is_independent_of_checkout_path(tmp_path):
    first = tmp_path / 'first'
    second = tmp_path / 'second'
    for directory in (first, second):
        directory.mkdir()
        (directory / 'hull.stl').write_bytes(b'collision mesh')
        (directory / 'model.xml').write_text(
            '<mujoco><asset><mesh name="hull" file="hull.stl"/></asset>'
            '<worldbody><body><geom name="link_collision_1" '
            'mesh="hull"/></body></worldbody></mujoco>',
            encoding='utf-8',
        )

    assert collision_model_fingerprint(
        first / 'model.xml') == collision_model_fingerprint(
            second / 'model.xml')


def test_finish_hold_disables_after_deadline():
    node = _hold_node(disable_on_finish=True)
    node._finish_hold(np.arange(7, dtype=float))

    assert node.state == 'hold'
    assert node.ready_messages == []

    node.disable_deadline = 9.0
    node.timer_callback()

    assert node.state == 'idle'
    assert node.ready_messages == [True, False]
    assert node.enable_messages == [False]


def test_finish_hold_returns_to_ready_when_disable_is_off():
    node = _hold_node(disable_on_finish=False)
    node._finish_hold(np.arange(7, dtype=float))
    node.disable_deadline = 9.0

    node.timer_callback()

    assert node.state == 'ready'
    assert node.ready_messages == [True]
    assert node.enable_messages == []
