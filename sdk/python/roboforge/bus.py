"""The signal bus: the one thing every component shares.

Physics backends, renderers, links (ROS 2, MAVLink) and controllers never
call each other. Each publishes and subscribes named, typed signals here, so
adding a backend or a transport is one adapter, not one bridge per pairing.

The bus is in-process and thread-safe: links usually publish from their own
receive threads. Subscribers run in the publisher's thread, so keep them short.
"""
from __future__ import annotations

import fnmatch
import threading
from collections import defaultdict
from typing import Any, Callable

Handler = Callable[[str, Any], None]


class Bus:
    def __init__(self):
        self._lock = threading.RLock()
        self._latest: dict[str, Any] = {}
        self._subs: dict[str, list[Handler]] = defaultdict(list)
        self._patterns: list[tuple[str, Handler]] = []
        self._count: dict[str, int] = defaultdict(int)
        self._source: dict[str, set[str]] = defaultdict(set)

    def publish(self, name: str, msg: Any, source: str = "") -> None:
        """Store `msg` as the latest value of `name` and notify subscribers."""
        with self._lock:
            self._latest[name] = msg
            self._count[name] += 1
            if source:
                self._source[name].add(source)
            handlers = list(self._subs.get(name, ()))
            handlers += [h for p, h in self._patterns if fnmatch.fnmatchcase(name, p)]
        failed = None
        for h in handlers:
            try:
                h(name, msg)
            except Exception as e:      # one failing subscriber must not starve the others
                if failed is None:
                    failed = e
        if failed is not None:
            failed.add_note(f"(raised by a subscriber of {name!r})")
            raise failed

    def subscribe(self, name: str, handler: Handler) -> Callable[[], None]:
        """Call `handler(name, msg)` on every publish. `name` may contain `*`.

        Returns a function that removes the subscription.
        """
        with self._lock:
            if "*" in name:
                entry = (name, handler)
                self._patterns.append(entry)
                return lambda: self._patterns.remove(entry) if entry in self._patterns else None
            self._subs[name].append(handler)
            return lambda: self._subs[name].remove(handler) if handler in self._subs[name] else None

    def unsubscribe_all(self) -> None:
        """Drop every subscription (the latest values stay readable). An app does this when it closes."""
        with self._lock:
            self._subs.clear()
            self._patterns.clear()

    def latest(self, name: str, default: Any = None) -> Any:
        with self._lock:
            return self._latest.get(name, default)

    def names(self) -> list[str]:
        with self._lock:
            return sorted(self._latest)

    def count(self, name: str) -> int:
        with self._lock:
            return self._count.get(name, 0)

    def describe(self) -> list[dict]:
        """One row per signal seen so far: name, message type, publishers, count."""
        with self._lock:
            return [
                {"name": n, "type": type(self._latest[n]).__name__, "sources": sorted(self._source.get(n, ())), "count": self._count[n]}
                for n in sorted(self._latest)
            ]
