from pathlib import Path
import sys
import threading
from types import SimpleNamespace
from unittest.mock import patch


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from scripts.node import DmMotorUsbNode  # noqa: E402


def _interlock_node():
    node = DmMotorUsbNode.__new__(DmMotorUsbNode)
    node.lock = threading.Lock()
    node.require_trajectory_ready = True
    node.trajectory_ready_timeout = 0.2
    node.trajectory_ready = False
    node.trajectory_ready_time = None
    node.enabled = False
    node._log_throttled = lambda *_args: None
    return node


def test_enable_requires_fresh_trajectory_heartbeat():
    node = _interlock_node()

    node.enable_callback(SimpleNamespace(data=True))
    assert not node.enabled

    node.trajectory_ready = True
    node.trajectory_ready_time = 1.0
    with patch('scripts.node.time.monotonic', return_value=1.1):
        node.enable_callback(SimpleNamespace(data=True))
    assert node.enabled

    with patch('scripts.node.time.monotonic', return_value=1.3):
        node.enable_callback(SimpleNamespace(data=True))
    assert not node.enabled


def test_false_heartbeat_disables_an_enabled_node():
    node = _interlock_node()
    node.enabled = True

    with patch('scripts.node.time.monotonic', return_value=2.0):
        node.trajectory_ready_callback(SimpleNamespace(data=False))

    assert not node.enabled
    assert not node.trajectory_ready
    assert node.trajectory_ready_time == 2.0
