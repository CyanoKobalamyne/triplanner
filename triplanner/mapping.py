import enum
import typing
import warnings
from collections.abc import Mapping
from typing import Literal, NamedTuple, Optional

import geopandas
import shapely

from .geometry import Location

Graph = dict[int, dict[int, float]]


class NodeKind(enum.Enum):
    WATER = enum.auto()


WATER = NodeKind.WATER

AMENITY_KIND = {"drinking_water": NodeKind.WATER}


class NodeType(NamedTuple):
    tags: Optional[str]
    visible: bool
    timestamp: int
    version: int
    lat: float
    lon: float
    changeset: float
    id: int
    geometry: shapely.Geometry


class WayType(NamedTuple):
    access: Optional[str]
    area: float
    bicycle: Optional[str]
    bridge: Optional[str]
    cycleway: Optional[str]
    foot: Optional[str]
    footway: Optional[str]
    highway: str
    junction: Optional[str]
    lanes: Optional[str]
    lit: Optional[str]
    maxspeed: Optional[str]
    motorcar: Optional[str]
    motor_vehicle: Optional[str]
    name: Optional[str]
    oneway: Optional[str]
    passing_places: Optional[str]
    psv: Optional[str]
    ref: Optional[str]
    service: Optional[str]
    segregated: Optional[str]
    sidewalk: Optional[str]
    smoothness: Optional[str]
    surface: Optional[str]
    tracktype: Optional[str]
    tunnel: Optional[str]
    width: Optional[str]
    id: int
    timestamp: int
    version: int
    tags: str
    osm_type: Literal["way"]
    geometry: shapely.Geometry
    u: int
    v: int
    length: float


class PoiType(NamedTuple):
    tags: Optional[str]
    id: int
    timestamp: int
    visible: bool
    version: int
    lat: float
    lon: float
    changeset: float
    f_8: Optional[str]  # addr:city
    f_9: Optional[str]  # addr:country
    f_10: Optional[str]  # addr:housenumber
    f_11: Optional[str]  # addr:housename
    f_12: Optional[str]  # addr:postcode
    f_13: Optional[str]  # addr:street
    email: Optional[str]
    name: Optional[str]
    opening_hours: Optional[str]
    operator: Optional[str]
    phone: Optional[str]
    ref: Optional[str]
    url: Optional[str]
    website: Optional[str]
    amenity: str
    atm: Optional[str]
    bank: Optional[str]
    bicycle_parking: Optional[str]
    drinking_water: Optional[str]
    fountain: Optional[str]
    fuel: Optional[str]
    internet_access: Optional[str]
    parking: Optional[str]
    social_facility: Optional[str]
    source: Optional[str]
    start_date: Optional[str]
    theatre: Optional[str]
    university: Optional[str]
    wikipedia: Optional[str]
    geometry: shapely.Geometry
    osm_type: Literal["node"] | Literal["way"] | Literal["relation"]
    building: Optional[str]
    f_40: Optional[str]  # building:levels
    fast_food: Optional[str]
    landuse: Optional[str]
    library: Optional[str]
    office: Optional[str]
    school: Optional[str]


def build_graph(
    nodes: geopandas.GeoDataFrame, ways: geopandas.GeoDataFrame
) -> tuple[Graph, dict[int, Location]]:
    """Return graph and node info dictionary built from OSM nodes and ways."""
    adj: Graph = {}
    for way in ways.itertuples(index=False):
        way = typing.cast(WayType, way)
        adj.setdefault(way.u, {})[way.v] = way.length
        adj.setdefault(way.v, {})[way.u] = way.length
    loc: dict[int, Location] = {}
    for node in nodes.itertuples(index=False):
        node = typing.cast(NodeType, node)
        if node.id not in adj:
            # Skip nodes not connected to ways.
            continue
        if node.id in loc:
            warnings.warn(f"dupicate node {node.id}", category=RuntimeWarning)
        loc[node.id] = node.lat, node.lon
    return adj, loc


def sort_nodes(
    pois: geopandas.GeoDataFrame, closest_node: Mapping[Location, int]
) -> dict[NodeKind, set[int]]:
    nodes = {}
    for poi in pois.itertuples():
        poi = typing.cast(PoiType, poi)
        if poi.amenity in AMENITY_KIND:
            kind = AMENITY_KIND[poi.amenity]
            node = closest_node[poi.lat, poi.lon]
            nodes.setdefault(kind, set()).add(node)
    return nodes
