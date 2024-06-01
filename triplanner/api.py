"""API classes for TriPlanner."""

from __future__ import annotations

import dataclasses
import itertools
import typing
import warnings
from collections.abc import Iterator
from pathlib import Path as FilePath
from typing import Collection, Optional, Self

import geopandas
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
from .utils import InteractiveString, classproperty

MAP_FILE_SUFFIX = ".osm.pbf"


class Maps:
    """Allows querying and loading of available maps."""

    @classproperty
    def available(cls) -> str:
        """Return list of available maps in the current working directory."""
        return InteractiveString.from_iter(cls._iter_available())

    @staticmethod
    def _iter_available() -> Iterator[str]:
        for file in FilePath.cwd().iterdir():
            if file.name.endswith(MAP_FILE_SUFFIX):
                yield file.name.removesuffix(MAP_FILE_SUFFIX)

    @staticmethod
    def load(name: str) -> Map:
        """Load map with the given name located in the current working directory."""
        reader = pyrosm.OSM(name + MAP_FILE_SUFFIX)
        with warnings.catch_warnings(action="ignore", category=FutureWarning):
            nodes, ways = typing.cast(
                tuple[geopandas.GeoDataFrame, geopandas.GeoDataFrame],
                reader.get_network(nodes=True),
            )
            pois = typing.cast(
                geopandas.GeoDataFrame, reader.get_pois(custom_filter={"amenity": True})
            )
        return Map(name, nodes, ways, pois)


class Map:
    """Represents a map over which planning takes place."""

    name: str
    adj: Graph
    loc: dict[int, Location]
    node_lookup: NodeLookup
    nodes_of_kind: dict[NodeKind, set[int]]
    ways: geopandas.GeoDataFrame
    path: Optional[Path]

    def __init__(
        self,
        name: str,
        nodes: geopandas.GeoDataFrame,
        ways: geopandas.GeoDataFrame,
        pois: geopandas.GeoDataFrame,
    ) -> None:
        """Create map from nodes and ways."""
        self.name = name
        self.adj, self.loc = build_graph(nodes, ways)
        self.node_lookup = NodeLookup(self.loc)
        self.nodes_of_kind = sort_nodes(pois, self.node_lookup)
        self.ways = ways
        self.path = None

    def __rshift__(self, other: object) -> Path:
        if self.path is None:
            raise ValueError(f"first start a path with {self.name}.start")
        return self.path >> other

    @property
    def limits(self) -> tuple[Location, Location]:
        """Return the limits of the map as (southwest corner, northeast corner)."""
        xmin, ymin, xmax, ymax = self.ways.geometry.total_bounds
        return (ymin, xmin), (ymax, xmax)

    @property
    def start(self) -> Path:
        self.path = Path(self.adj, self.loc, self.node_lookup, self.nodes_of_kind)
        return self.path

    @property
    def loop(self) -> Path:
        if self.path is None:
            raise ValueError(f"first start a path with {self.name}.start")
        self.path.make_loop()
        return self.path

    @property
    def directions(self) -> str:
        if self.path is None:
            raise ValueError(f"first start a path with {self.name}.start")
        route = self.path.route()
        lines = []
        prev_direction = None
        for a, b in zip(route, route[1:]):
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
        if self.path is None:
            warnings.warn("drawing map without a path", RuntimeWarning)
        else:
            self.path.draw(fig, ax)
        matplotlib.pyplot.show()


@dataclasses.dataclass(repr=False)
class Path:
    adj: Graph
    loc: dict[int, Location]
    node_lookup: NodeLookup
    nodes_of_kind: dict[NodeKind, set[int]]
    nodes: list[Collection[int]] = dataclasses.field(default_factory=list, init=False)
    routes: list[list[Location]] = dataclasses.field(default_factory=list, init=False)
    closed: bool = dataclasses.field(default=False, init=False)
    constraints: dict[int, Constraint] = dataclasses.field(
        default_factory=dict, init=False
    )

    def __rshift__(self, other: object) -> Self:
        if self.closed:
            raise ValueError(f"path is closed, can't add {other}")
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
                "path elements must be (latitude, longitude) tuples or place types"
            )
        return self

    def __matmul__(self, other: object) -> Self:
        if len(self.nodes) < 2:
            raise ValueError("no segments in path, cannot add a constraint")
        if not isinstance(other, Constraint):
            raise TypeError(f"invalid constraint: {other}")
        node_index = len(self.nodes) - 2
        if node_index in self.constraints:
            raise ValueError("only one constraint allowed per segment")
        self.constraints[node_index] = other
        return self

    def make_loop(self) -> None:
        if not self.nodes:
            raise ValueError("path is empty, cannot make it a loop")
        self.nodes.append(self.nodes[0])
        self.closed = True

    def route(self) -> list[Location]:
        self._update_routes()
        return [loc for route in self.routes for loc in route]

    def draw(self, fig: matplotlib.figure.Figure, ax: matplotlib.axes.Axes) -> None:
        self._update_routes()
        if not self.routes:
            warnings.warn("drawing empty path", RuntimeWarning)
        for route in self.routes:
            self._draw_route(fig, ax, route)

    def _update_routes(self) -> None:
        n = len(self.routes)
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
            self.routes.append([self.loc[n] for n in path])

    def _draw_route(
        self,
        fig: matplotlib.figure.Figure,
        ax: matplotlib.axes.Axes,
        route: list[Location],
    ) -> None:
        # Need to flip coordinates for plotting
        line = shapely.LineString((x, y) for (y, x) in route)
        series = geopandas.GeoSeries(line, crs="EPSG:4326")
        series.plot(ax=ax, color="red", linewidth=3, aspect="equal", zorder=2)
        # Start and end points
        y0, x0 = route[0]
        yend, xend = route[-1]
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
        # Zoom to route
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
