"""Velocity command for the base. Edit freely: this is your code.

Publishes forward speed and turn rate; the built-in diff-drive controller
(see app.yaml) turns that into wheel speeds. With ROS 2 enabled you can
remove this controller from app.yaml and drive with `cmd_vel` instead.
"""
import roboforge as rf


class Circle(rf.Controller):
    def step(self, obs, t):
        return rf.velocity_command(linear=(0.3, 0.0, 0.0), angular=(0.0, 0.0, 0.4))
