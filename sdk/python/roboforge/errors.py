"""Errors a user can act on.

`ConfigError` is raised for anything wrong in an `app.yaml` or in the
arguments an app was built with: an unknown name, a misspelt argument, a value
out of range, parts that do not fit together. It is a `ValueError`, and its
message says what was found, what is allowed and, where it can, what was
probably meant.
"""
from __future__ import annotations

import difflib


class ConfigError(ValueError):
    pass


class SimulationError(RuntimeError):
    """The physics could not go on: the simulation blew up."""


def suggest(word, options) -> str:
    """` Did you mean 'x'?` if `word` is close to one of `options`, else an empty string."""
    close = difflib.get_close_matches(str(word), [str(o) for o in options], n=1, cutoff=0.6)
    return f" Did you mean {close[0]!r}?" if close else ""


def positive(value, what: str, allow_none: bool = False, integer: bool = False):
    """Check a number that must be greater than zero; returns it."""
    if value is None and allow_none:
        return value
    ok = isinstance(value, (int, float)) and not isinstance(value, bool) and value == value and value > 0
    if ok and integer and int(value) != value:
        ok = False
    if not ok:
        raise ConfigError(f"{what} must be a {'whole ' if integer else ''}number greater than 0, got {value!r}")
    return value
