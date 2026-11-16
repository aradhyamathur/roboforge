"""Sphinx configuration for the RoboForge SDK documentation.

Build locally:   sdk/python/.venv/bin/python -m sphinx -W -b html sdk/python/docs sdk/python/docs/_build/html
"""
import os
import sys

sys.path.insert(0, os.path.abspath(".."))
os.environ.setdefault("MUJOCO_GL", "egl")        # nothing is rendered, but importing must not need a display

import roboforge  # noqa: E402

project = "RoboForge SDK"
author = "RoboForge"
copyright = "2026, RoboForge"
release = version = roboforge.__version__

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.viewcode",
    "sphinx.ext.autosectionlabel",
    "sphinx_copybutton",
]
autosectionlabel_prefix_document = True
autosectionlabel_maxdepth = 2
autodoc_member_order = "bysource"
autodoc_default_options = {"members": True, "show-inheritance": True}
autodoc_mock_imports = ["rclpy", "geometry_msgs", "nav_msgs", "sensor_msgs", "std_msgs", "rosgraph_msgs", "tf2_msgs"]
autodoc_typehints = "description"

templates_path = ["_templates"]
exclude_patterns = ["_build"]
html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]
html_css_files = ["custom.css"]
html_title = f"RoboForge SDK {release}"
html_theme_options = {"navigation_depth": 3, "collapse_navigation": False}
copybutton_prompt_text = r"\$ "
copybutton_prompt_is_regexp = True
