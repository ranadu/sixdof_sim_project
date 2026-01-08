from __future__ import annotations
from dataclasses import dataclass, field
from typing import Dict, List
import numpy as np

@dataclass
class TelemetryLog:
    t: List[float] = field(default_factory=list)
    X: List[np.ndarray] = field(default_factory=list)
    F: List[np.ndarray] = field(default_factory=list)
    M: List[np.ndarray] = field(default_factory=list)

    def append(self, t: float, x: np.ndarray, F: np.ndarray, M: np.ndarray) -> None:
        self.t.append(float(t))
        self.X.append(np.asarray(x, dtype=float).copy())
        self.F.append(np.asarray(F, dtype=float).copy())
        self.M.append(np.asarray(M, dtype=float).copy())

    def to_arrays(self) -> Dict[str, np.ndarray]:
        return {
            "t": np.asarray(self.t, dtype=float),
            "X": np.vstack(self.X) if self.X else np.zeros((0, 13)),
            "F": np.vstack(self.F) if self.F else np.zeros((0, 3)),
            "M": np.vstack(self.M) if self.M else np.zeros((0, 3)),
        }