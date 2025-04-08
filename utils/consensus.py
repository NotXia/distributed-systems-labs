from collections.abc import Iterable


def has_reached_consensus(states: Iterable[float], tolerance: float=1e-3):
    for i in range(0, len(states)):
        for j in range(i+1, len(states)):
            if states[i] - states[j] > tolerance:
                return False
    return True