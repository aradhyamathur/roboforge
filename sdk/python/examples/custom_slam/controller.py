"""Drive in a circle: forward 0.3 m/s, turning 0.4 rad/s."""
import roboforge as rf


class Circle(rf.Controller):
    def step(self, obs, t):
        return rf.velocity_command(linear=(0.3, 0.0, 0.0), angular=(0.0, 0.0, 0.4))
