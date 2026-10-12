"""Exclusive, expiring lease on the shared delivery square (pure logic, time passed in)."""

from collections import Counter


class ZoneLease:
    def __init__(self, lease_seconds: float = 60.0):
        self.lease_seconds = lease_seconds
        self._holder: str | None = None
        self._expires_at = 0.0
        self._counts: Counter[str] = Counter()

    def occupant(self, now: float) -> str | None:
        """Current holder; an expired lease frees the zone."""
        if self._holder is not None and now >= self._expires_at:
            self._holder = None
        return self._holder

    def request(self, robot: str, now: float) -> tuple[bool, str | None]:
        """Grant the zone if free or already held by robot (renewing the lease)."""
        holder = self.occupant(now)
        if holder not in (None, robot):
            return False, holder
        self._holder = robot
        self._expires_at = now + self.lease_seconds
        return True, robot

    def release(self, robot: str, delivered: bool, color: str, now: float) -> bool:
        """Free the zone; only the current holder may do it. Delivered cubes are counted."""
        if self.occupant(now) != robot:
            return False
        self._holder = None
        if delivered:
            self._counts[robot] += 1
        return True

    @property
    def counts(self) -> dict[str, int]:
        return dict(self._counts)

    def delivered_total(self, robot: str) -> int:
        return self._counts[robot]
