from __future__ import annotations

import dataclasses
import typing

if typing.TYPE_CHECKING:
    from typing import Callable


@dataclasses.dataclass
class ConstraintValue:
    amount: float


@dataclasses.dataclass
class ConstraintMetric:
    multiplier: float

    def __mul__(self, other: object) -> ConstraintValue:
        if not isinstance(other, (int, float)):
            raise TypeError(f"{other} is not a number")
        return ConstraintValue(other * self.multiplier)

    def __rmul__(self, other: object) -> ConstraintValue:
        return self * other


@dataclasses.dataclass
class Constraint:
    filter_fn: Callable[[float], bool]

    def __call__(self, cost: float, /) -> bool:
        return self.filter_fn(cost)


@dataclasses.dataclass
class ConstraintBuilder:
    calc_fn: Callable[[float], float]

    def __gt__(self, other: object) -> Constraint:
        if not isinstance(other, ConstraintValue):
            raise TypeError("constraint values must be specified as AMOUNT * UNIT")
        return Constraint(lambda x: self.calc_fn(x) > other.amount)

    def __ge__(self, other: object) -> Constraint:
        if not isinstance(other, ConstraintValue):
            raise TypeError("constraint values must be specified as AMOUNT * UNIT")
        return Constraint(lambda x: self.calc_fn(x) >= other.amount)


M = ConstraintMetric(1.0)
FT = ConstraintMetric(0.3048)

LENGTH = ConstraintBuilder(lambda cost: cost)
