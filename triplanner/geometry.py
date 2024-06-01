import enum
import math
from collections.abc import Collection, Iterator, Mapping
from math import atan2, cos, sin, sqrt

MEAN_EARTH_RADIUS_METERS = 6_371_009

"""tuple of (latitude, longitude)"""
Location = tuple[float, float]


class Direction(enum.StrEnum):
    NORTH = "north"
    SOUTH = "south"
    WEST = "west"
    EAST = "east"


def cardinal_direction(bearing: float) -> Direction:
    """Return the cardinal direction corresponding to the given bearing (in radians)."""
    if -3 / 4 < bearing / math.pi <= -1 / 4:
        return Direction.WEST
    elif -1 / 4 < bearing / math.pi <= 1 / 4:
        return Direction.NORTH
    elif 1 / 4 < bearing / math.pi <= 3 / 4:
        return Direction.EAST
    else:
        return Direction.SOUTH


def bearing(loc1: Location, loc2: Location) -> float:
    """Return the bearing (angular direction) in radians from loc1 to loc2."""
    lat1, lon1 = loc1
    lat2, lon2 = loc2
    phi1, lambda1 = math.radians(lat1), math.radians(lon1)
    phi2, lambda2 = math.radians(lat2), math.radians(lon2)
    delta_lambda = lambda2 - lambda1

    y = sin(delta_lambda) * cos(phi2)
    x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(delta_lambda)

    return atan2(y, x)


def great_circle_distance(loc1: Location, loc2: Location) -> float:
    """Return the approximate distance between loc1 and loc2 in meters.

    This function takes into account the Earth's curvature but assumes a spherical earth.

    Locations are (latitude, longitude) tuples. Latitude and longitude are in degrees.
    """
    lat1, lon1 = loc1
    lat2, lon2 = loc2
    phi1, lambda1 = math.radians(lat1), math.radians(lon1)
    phi2, lambda2 = math.radians(lat2), math.radians(lon2)
    delta_lambda = lambda2 - lambda1
    sinpsi = sqrt(
        (sin(lambda1) * cos(phi1) * sin(phi2) - sin(lambda2) * cos(phi2) * sin(phi1))
        ** 2
        + (cos(lambda2) * cos(phi2) * sin(phi1) - cos(lambda1) * cos(phi1) * sin(phi2))
        ** 2
        + (cos(phi1) * cos(phi2) * sin(delta_lambda)) ** 2
    )
    cospsi = sin(phi1) * sin(phi2) + cos(phi1) * cos(phi2) * cos(delta_lambda)
    return atan2(sinpsi, cospsi) * MEAN_EARTH_RADIUS_METERS


class NearestNeighbor:
    """Implements a multi-way k-d tree."""

    def __init__(
        self,
        locs: Collection[tuple[float, float]],
        max_depth: int,
        branching_factor: int,
        flip: bool = False,
        depth: int = 0,
    ):
        self.children = []
        self.bounds = []
        self.locs = None
        self.flip = flip
        if depth == max_depth:
            self.locs = locs
            return
        sorted_locs = sorted(locs, key=lambda loc: loc[1]) if flip else sorted(locs)
        for b in range(branching_factor):
            lo = b * len(locs) // branching_factor
            hi = (b + 1) * len(locs) // branching_factor
            child = NearestNeighbor(
                sorted_locs[lo:hi],
                max_depth,
                branching_factor,
                not flip,
                depth + 1 if flip else depth,
            )
            self.children.append(child)
            if hi < len(locs):
                self.bounds.append(sorted_locs[hi])

    def __getitem__(self, loc: Location) -> Location:
        if self.locs is not None:
            min_dist = float("inf")
            min_dist_loc = None
            for nloc in self.locs:
                dist = great_circle_distance(loc, nloc)
                if dist < min_dist:
                    min_dist = dist
                    min_dist_loc = nloc
            assert min_dist_loc is not None
            return min_dist_loc
        i_loc = int(self.flip)
        for index, bound in enumerate(self.bounds):
            if bound[i_loc] <= loc[i_loc]:
                break
        else:
            raise RuntimeError("missing bounds from internal tree node")
        candidate = self.children[index][loc]
        dist = great_circle_distance(loc, candidate)
        bound_loc = (loc[0], bound[1]) if self.flip else (bound[0], loc[1])
        bound_dist = great_circle_distance(loc, bound_loc)
        if bound_dist < dist:
            candidate2 = self.children[index + 1][loc]
            dist2 = great_circle_distance(loc, candidate2)
            if dist2 < dist:
                return candidate2
        return candidate


class NodeLookup(Mapping[Location, int]):
    """Data structure that returns the node closest to a given location."""

    def __init__(self, locs: dict[int, Location], max_depth=0, branching_factor=2):
        """Create a new node lookup object."""
        self.nid = {location: id_ for id_, location in locs.items()}
        self.cache = self.nid.copy()
        self.tree = NearestNeighbor(locs.values(), max_depth, branching_factor)

    def __getitem__(self, loc: Location) -> int:
        """Look up node closest to loc."""
        if loc not in self.cache:
            self.cache[loc] = self.nid[self.tree[loc]]
        return self.cache[loc]

    def __iter__(self) -> Iterator[Location]:
        yield from self.nid

    def __len__(self) -> int:
        return len(self.nid)
