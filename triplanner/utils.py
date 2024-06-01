"""Miscellaneous utilities."""

from __future__ import annotations

import sys
from collections.abc import Iterable


class classproperty:
    def __init__(self, fget):
        self.fget = fget

    def __get__(self, obj, objtype=None):
        return self.fget(objtype)


class InteractiveString(str):
    @classmethod
    def from_iter(cls, values: Iterable[str]) -> InteractiveString:
        return cls("\n".join(values))

    def __repr__(self) -> str:
        return str(self)


class InteractiveExit:
    def __repr__(self) -> str:
        sys.exit(0)
