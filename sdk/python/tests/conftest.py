import os
import socket

import pytest

# Keep any ROS traffic from these tests on this machine and off the default domain.
os.environ.setdefault("ROS_DOMAIN_ID", "87")
os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")


@pytest.fixture
def free_port():
    """A UDP/TCP port number that is free right now."""
    def get(kind=socket.SOCK_DGRAM):
        s = socket.socket(socket.AF_INET, kind)
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
        s.close()
        return port
    return get
