#!/usr/bin/env python3
from collections.abc import Container
from typing import Callable, Optional

from .geometry import great_circle_distance
from .mapping import Graph


def null_heuristic(_node: int) -> float:
    return 0


def make_distance_heuristic(
    loc: dict[int, tuple[float, float]], goal: int
) -> Callable[[int], float]:
    """Return an admissible heuristic for A* search based on distance to a goal."""

    def heuristic(node: int) -> float:
        return great_circle_distance(loc[node], loc[goal])

    return heuristic


def reconstruct_path(parent: dict[int, Optional[int]], last_node: int) -> list[int]:
    """Recreate path to last node from parent dictionary."""
    path = []
    node = last_node
    while node is not None:
        path.append(node)
        node = parent[node]
    path.reverse()
    return path


def qpush(queue, item):
    """Add item to the queue."""
    for i, value in enumerate(queue):
        if item > value:
            queue.insert(i, item)
            break
    else:
        queue.append(item)


def qpop(queue):
    """Remove and return minimal item from the queue."""
    return queue.pop()


def a_star_search(
    adj: Graph, start: int, goals: Container[int], heur_fn: Callable[[int], float]
) -> list[int]:
    """Return minimum-cost path from start to a node in goals."""
    queue = [(heur_fn(start), start)]
    parent: dict[int, Optional[int]] = {start: None}
    min_cost: dict[int, float] = {start: 0}
    while queue:
        _, node = qpop(queue)
        if node in goals:
            return reconstruct_path(parent, node)
        path_cost = min_cost[node]
        for neighbor, edge_cost in adj[node].items():
            if path_cost + edge_cost < min_cost.get(neighbor, float("inf")):
                parent[neighbor] = node
                total_cost = path_cost + edge_cost
                min_cost[neighbor] = total_cost
                heuristic = heur_fn(neighbor)
                qpush(queue, (total_cost + heuristic, neighbor))
    return []


def constrained_shortest_path(
    adj: Graph, start: int, goals: Container[int], filter_fn: Callable[[float], bool]
) -> list[int]:
    """Return minimum-cost path from start to a node in goals, subject to filter_fn."""

    def reconstruct(path):
        if not path:
            return
        pred, node = path
        yield from reconstruct(pred)
        yield node

    queue = [(0, ((), start), {start})]
    while queue:
        path_cost, path, visited = qpop(queue)
        node = path[1]
        for neighbor, edge_cost in adj[node].items():
            if neighbor in goals and filter_fn(path_cost):
                return list(reconstruct((path, neighbor)))
            if neighbor in visited:
                continue
            total_cost = path_cost + edge_cost
            qpush(queue, (total_cost, (path, neighbor), visited | {neighbor}))
    return []
