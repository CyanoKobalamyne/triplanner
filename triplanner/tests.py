# flake8: noqa
# pyright: reportUnusedExpression = false
import os
import unittest
import warnings

from . import *
from .api import Map
from .utils import iter_maps


def setUpModule():
    global maps, stanford
    os.chdir(f"{os.path.dirname(__file__)}/test_files")
    maps = list(iter_maps())
    stanford = Map.load("stanford")


class TestTriplanner(unittest.TestCase):
    def test_static(self):
        self.assertIn("stanford", maps)
        (x1, y1), (x2, y2) = stanford.limits
        self.assertLess(x1, x2)
        self.assertLess(y1, y2)

    @unittest.skipUnless("TEST_DRAW" in os.environ, "draws map")
    def test_warn_empty_map(self):
        stanford.start
        with (
            warnings.catch_warnings(action="ignore", category=RuntimeWarning),
            self.assertWarns(RuntimeWarning),
        ):
            stanford.draw

    def test_basic(self):
        stanford.start >> (37.42982, -122.17341) >> (37.42415, -122.15325)
        self.assertTrue(stanford.directions.splitlines())
        if "TEST_DRAW" in os.environ:
            stanford.draw

    def test_loop(self):
        stanford.start >> (37.42982, -122.17341)
        stanford.loop @ (LENGTH >= 250 * M)
        self.assertTrue(stanford.directions.splitlines())
        if "TEST_DRAW" in os.environ:
            stanford.draw

    def test_stops(self):
        stanford.start >> (37.42982, -122.17341) >> (37.4323, -122.1721)
        stanford >> (37.42415, -122.15325)
        self.assertTrue(stanford.directions.splitlines())
        if "TEST_DRAW" in os.environ:
            stanford.draw

    def test_amenity(self):
        stanford.start >> (37.42624, -122.15705) >> WATER
        self.assertTrue(stanford.directions.splitlines())
        if "TEST_DRAW" in os.environ:
            stanford.draw


if __name__ == "__main__":
    unittest.main()
