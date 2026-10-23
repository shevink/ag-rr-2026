from dataclasses import dataclass
from typing import List

from ag_labs_py.types import Point

@dataclass
class Obstacle:
  oscillation: bool
  speed_ms: float
  path_points: List[Point]