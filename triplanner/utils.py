"""Miscellaneous utilities."""

from __future__ import annotations

import sys
from collections.abc import Iterable, Iterator
from pathlib import Path

MAP_FILE_SUFFIX = ".osm.pbf"


class InteractiveString(str):
    @classmethod
    def from_iter(cls, values: Iterable[str]) -> InteractiveString:
        return cls("\n".join(values))

    def __repr__(self) -> str:
        return str(self)


class InteractiveExit:
    def __repr__(self) -> str:
        sys.exit(0)


def iter_maps() -> Iterator[str]:
    for file in Path.cwd().iterdir():
        if file.name.endswith(MAP_FILE_SUFFIX):
            yield file.name.removesuffix(MAP_FILE_SUFFIX)
