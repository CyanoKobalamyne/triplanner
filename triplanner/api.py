"""API classes for TriPlanner."""

from __future__ import annotations

import dataclasses
import itertools
import typing
import warnings
from typing import Collection, Optional, Self

import geopandas as gpd
import matplotlib.axes
import matplotlib.figure
import matplotlib.pyplot
import pyrosm
import shapely

from .constraints import Constraint
from .geometry import (
    Location,
    NodeLookup,
    bearing,
    cardinal_direction,
    great_circle_distance,
)
from .mapping import Graph, NodeKind, build_graph, sort_nodes
from .routing import (
    a_star_search,
    constrained_shortest_path,
    make_distance_heuristic,
    null_heuristic,
)
from .utils import MAP_FILE_SUFFIX, InteractiveString


@dataclasses.dataclass
class Map:
    """Represents a map over which planning takes place."""

    name: str
    nodes: dataclasses.InitVar[gpd.GeoDataFrame]
    ways: gpd.GeoDataFrame = dataclasses.field(repr=False)
    pois: dataclasses.InitVar[gpd.GeoDataFrame]
    adj: Graph = dataclasses.field(init=False, repr=False)
    loc: dict[int, Location] = dataclasses.field(init=False, repr=False)
    node_lookup: NodeLookup = dataclasses.field(init=False, repr=False)
    nodes_of_kind: dict[NodeKind, set[int]] = dataclasses.field(init=False, repr=False)
    route: Optional[Route] = None

    def __post_init__(self, nodes: gpd.GeoDataFrame, pois: gpd.GeoDataFrame):
        self.adj, self.loc = build_graph(nodes, self.ways)
        self.node_lookup = NodeLookup(self.loc)
        self.nodes_of_kind = sort_nodes(pois, self.node_lookup)

    @classmethod
    def load(cls, name: str) -> Self:
        """Load map with the given name located in the current working directory."""
        reader = pyrosm.OSM(name + MAP_FILE_SUFFIX)
        with warnings.catch_warnings(action="ignore", category=FutureWarning):
            nodes, ways = typing.cast(
                tuple[gpd.GeoDataFrame, gpd.GeoDataFrame],
                reader.get_network(nodes=True),
            )
            pois = typing.cast(
                gpd.GeoDataFrame, reader.get_pois(custom_filter={"amenity": True})
            )
        return cls(name, nodes, ways, pois)

    def __rshift__(self, other: object) -> Route:
        if self.route is None:
            raise ValueError(f"first start a route with {self.name}.start")
        return self.route >> other

    def __matmul__(self, other: object) -> Route:
        if self.route is None:
            raise ValueError(f"first start a route with {self.name}.start")
        return self.route @ other

    @property
    def limits(self) -> tuple[Location, Location]:
        """Return the limits of the map as (southwest corner, northeast corner)."""
        xmin, ymin, xmax, ymax = self.ways.geometry.total_bounds
        return (ymin, xmin), (ymax, xmax)

    @property
    def start(self) -> Route:
        self.route = Route(self.adj, self.loc, self.node_lookup, self.nodes_of_kind)
        return self.route

    @property
    def loop(self) -> Route:
        if self.route is None:
            raise ValueError(f"first start a route with {self.name}.start")
        self.route.make_loop()
        return self.route

    @property
    def directions(self) -> str:
        if self.route is None:
            raise ValueError(f"first start a route with {self.name}.start")
        path = self.route.path()
        lines = []
        prev_direction = None
        for a, b in zip(path, path[1:]):
            distance = great_circle_distance(a, b)
            direction = cardinal_direction(bearing(a, b))
            if prev_direction is None:
                lines.append(f"Go {distance:.1f} meters towards the {direction}")
            else:
                lines.append(f"Turn towards the {direction}")
                lines.append(f"Go {distance:.1f} meters")
            prev_direction = direction
        return InteractiveString.from_iter(lines)

    @property
    def draw(self) -> None:
        fig, ax = matplotlib.pyplot.subplots()
        ax.axis("off")
        self.ways.plot(ax=ax, color="blue", zorder=1)
        if self.route is None:
            warnings.warn("drawing map without a route", RuntimeWarning)
        else:
            self.route.draw(fig, ax)
        matplotlib.pyplot.show()


@dataclasses.dataclass
class Route:
    adj: Graph = dataclasses.field(repr=False)
    loc: dict[int, Location] = dataclasses.field(repr=False)
    node_lookup: NodeLookup = dataclasses.field(repr=False)
    nodes_of_kind: dict[NodeKind, set[int]] = dataclasses.field(repr=False)
    nodes: list[Collection[int]] = dataclasses.field(default_factory=list)
    paths: list[list[Location]] = dataclasses.field(default_factory=list, repr=False)
    constraints: dict[int, Constraint] = dataclasses.field(default_factory=dict)
    closed: bool = False

    def __rshift__(self, other: object) -> Self:
        if self.closed:
            raise ValueError(f"route is closed, can't add {other}")
        if (
            isinstance(other, tuple)
            and len(other) == 2
            and all(isinstance(c, (int, float)) for c in other)
        ):
            # Location given
            node_id = self.node_lookup[other]
            self.nodes.append({node_id})
        elif isinstance(other, NodeKind):
            node_filter = self.nodes_of_kind[other]
            self.nodes.append(node_filter)
        else:
            raise TypeError(
                "route elements must be (latitude, longitude) tuples or place types"
            )
        return self

    def __matmul__(self, other: object) -> Self:
        if len(self.nodes) < 2:
            raise ValueError("no segments in route, cannot add a constraint")
        if not isinstance(other, Constraint):
            raise TypeError(f"invalid constraint: {other}")
        node_index = len(self.nodes) - 2
        if node_index in self.constraints:
            raise ValueError("only one constraint allowed per segment")
        self.constraints[node_index] = other
        return self

    def make_loop(self) -> None:
        if self.closed:
            warnings.warn("route is already a loop", RuntimeWarning)
            return
        if not self.nodes:
            raise ValueError("route is empty, cannot make it a loop")
        self.nodes.append(self.nodes[0])
        self.closed = True

    def path(self) -> list[Location]:
        self._update_paths()
        return [loc for path in self.paths for loc in path]

    def draw(self, fig: matplotlib.figure.Figure, ax: matplotlib.axes.Axes) -> None:
        self._update_paths()
        if not self.paths:
            warnings.warn("drawing empty route", RuntimeWarning)
        for path in self.paths:
            self._draw_path(fig, ax, path)

    def _update_paths(self) -> None:
        n = len(self.paths)
        for i, (start, goal) in enumerate(zip(self.nodes[n:], self.nodes[n + 1 :])):
            if len(start) != 1:
                raise NotImplementedError("generic goal only supported in last place")
            start = next(iter(start))
            if i in self.constraints:
                constraint = self.constraints[i]
                path = constrained_shortest_path(self.adj, start, goal, constraint)
            else:
                if len(goal) == 1:
                    heuristic = make_distance_heuristic(self.loc, next(iter(goal)))
                else:
                    heuristic = null_heuristic
                path = a_star_search(self.adj, start, goal, heuristic)
            self.paths.append([self.loc[n] for n in path])

    def _draw_path(
        self,
        fig: matplotlib.figure.Figure,
        ax: matplotlib.axes.Axes,
        path: list[Location],
    ) -> None:
        # Need to flip coordinates for plotting
        line = shapely.LineString((x, y) for (y, x) in path)
        series = gpd.GeoSeries(line, crs="EPSG:4326")
        series.plot(ax=ax, color="red", linewidth=3, aspect="equal", zorder=2)
        # Start and end points
        y0, x0 = path[0]
        yend, xend = path[-1]
        ax.plot(x0, y0, marker="o", mec="red", mfc="white", zorder=3)
        ax.plot(xend, yend, marker="o", color="red", zorder=3)
        # Add arrows
        arrowsize = 1.8e-4  # about 20 meters in real distance
        for (x1, y1), (x2, y2) in itertools.pairwise(line.coords):
            dx = x2 - x1
            dy = y2 - y1
            if arrowsize * 2 <= (dx**2 + dy**2) ** 0.5:
                ax.arrow(
                    x1,
                    y1,
                    dx,
                    dy,
                    color="red",
                    width=0,
                    head_width=arrowsize,
                    head_length=arrowsize,
                )
        # Zoom to path
        xmin, ymin, xmax, ymax = line.bounds
        xavg, yavg = (xmin + xmax) / 2, (ymin + ymax) / 2
        width, height = xmax - xmin, ymax - ymin
        figwidth, figheight = fig.get_size_inches()
        aspect_ratio = figwidth / figheight
        if width / height < aspect_ratio:
            # Tall and narrow figure, use height to scale
            xheight = height * aspect_ratio
            ax.set_xlim(xavg - 0.6 * xheight, xavg + 0.6 * xheight)
            ax.set_ylim(ymin - 0.1 * height, ymax + 0.1 * height)
        else:
            # Wide and short figure, use width to scale
            ywidth = width / aspect_ratio
            ax.set_xlim(xmin - 0.1 * width, xmax + 0.1 * width)
            ax.set_ylim(yavg - 0.6 * ywidth, yavg + 0.6 * ywidth)
