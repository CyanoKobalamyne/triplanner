# TriPlanner: an advanced trip planner for running, biking, hiking

The goal of this application is to make it easier to plan trips that are more complicated
than getting from point A to point B.

## Installation

You can install this package using `pip`:
```
pip install triplanner
```

It depends on `pyrosm` to read map files and on `matplotlib` for drawing maps.

## Basic usage

To do any planning, you will first need to download a map in OpenStreetMap's PBF format.

When you launch the app, You will be presented with a prompt like this:
```
>>>
```

Maps in the current working directory will be made available to you. You can list the
available maps by typing `maps` at the prompt:
```
>>> maps
```

The guide below assumes that you have downloaded "mycity.osm.pbf" in the current working
directory and the command above prints "mycity". The file name is arbitrary; if it is
called something else, replace "mycity" appropriately. The extension must be ".osm.pbf".

To create a route, you can type map.start and use the ">>" operator to add locations:
```
>>> mycity.start >> (37.42982, -122.17341)
```

Once a route has been started, you can add more points to it:
```
>>> mycity >> (37.42415, -122.15325)
```

Locations can be specified as (latitude, longitude) pairs or as location types. The
boundaries of the current map can be obtained with:
```
>>> mycity.limits
```

Currently, the only supported location type is WATER, meaning drinking fountain. If
included in the path, it must come last:
```
>>> mycity >> WATER
```

It is also possible to add path constraints. These will apply to the last added segment:
```
>>> mycity @ (LENGTH >= 100 * M)
```

This means that the last path segment will be at least 100 meters long. The available
units are `M` (meters) and `FT` (feet).

Routes can be made into a loop (starting and ending at the same place) with:
```
>>> mycity.loop
```

Directions for routes can be obtained with:
```
>>> mycity.directions
```

Routes can be visualized with:
```
>>> mycity.draw
```
