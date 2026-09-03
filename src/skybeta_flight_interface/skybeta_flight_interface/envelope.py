"""Safety envelope: the clamp every competitor setpoint passes through.

Setpoints outside the envelope are **clamped, never rejected**.  Rejecting a
setpoint leaves PX4 holding whatever it had last, which for a velocity
setpoint means a vehicle that keeps flying at the command that took it out of
bounds; clamping keeps the stream continuous and pins the commanded state to
the boundary.  The clamp is deliberately dumb -- no hysteresis, no
projection along the violated axis -- because it has to be obviously correct
under review, not clever.

All quantities are in the ENU ``odom`` frame the competitor writes in, so the
clamp happens *before* any frame conversion.  Altitude is measured from the
origin captured at the first odometry sample, which for a vehicle sitting on
the ground at startup is ground level.
"""
import math
from dataclasses import dataclass, field


@dataclass(frozen=True)
class Clamp:
    """Result of one clamp: the value to use, and why it changed."""

    value: tuple
    reasons: tuple = field(default=())

    @property
    def clamped(self):
        return bool(self.reasons)


@dataclass
class SafetyEnvelope:
    """Cylinder around the odom origin, plus a speed cap.

    geofence_radius_m  horizontal radius about the origin
    max_altitude_m     ceiling above the origin
    min_altitude_m     floor above the origin -- NOT a landing inhibit; the
                       pilot lands on RC, this only stops a competitor
                       commanding the vehicle into the ground
    max_speed_mps      cap on the norm of a velocity setpoint
    """

    geofence_radius_m: float = 10.0
    max_altitude_m: float = 3.0
    min_altitude_m: float = 0.3
    max_speed_mps: float = 1.5

    def __post_init__(self):
        self.geofence_radius_m = float(self.geofence_radius_m)
        self.max_altitude_m = float(self.max_altitude_m)
        self.min_altitude_m = float(self.min_altitude_m)
        self.max_speed_mps = float(self.max_speed_mps)
        if self.geofence_radius_m <= 0.0:
            raise ValueError('geofence_radius_m must be > 0')
        if self.max_speed_mps <= 0.0:
            raise ValueError('max_speed_mps must be > 0')
        if self.min_altitude_m > self.max_altitude_m:
            raise ValueError(
                'min_altitude_m must be <= max_altitude_m '
                f'({self.min_altitude_m} > {self.max_altitude_m})')

    # ------------------------------------------------------------------
    def clamp_position(self, position_enu, origin_enu=(0.0, 0.0, 0.0)):
        """Clamp an ENU position into the cylinder around ``origin_enu``.

        Horizontal excursions are scaled back onto the boundary along the
        same bearing, so the commanded direction is preserved and only the
        distance changes.  Altitude is clipped independently.
        """
        px, py, pz = (float(v) for v in position_enu)
        ox, oy, oz = (float(v) for v in origin_enu)
        dx, dy, dz = px - ox, py - oy, pz - oz
        reasons = []

        radius = math.hypot(dx, dy)
        if radius > self.geofence_radius_m:
            if radius > 0.0:
                scale = self.geofence_radius_m / radius
                dx, dy = dx * scale, dy * scale
            reasons.append('geofence_radius')

        if dz > self.max_altitude_m:
            dz = self.max_altitude_m
            reasons.append('max_altitude')
        elif dz < self.min_altitude_m:
            dz = self.min_altitude_m
            reasons.append('min_altitude')

        return Clamp((ox + dx, oy + dy, oz + dz), tuple(reasons))

    # ------------------------------------------------------------------
    def clamp_velocity(self, velocity_enu):
        """Scale a velocity vector back to ``max_speed_mps`` if over."""
        vx, vy, vz = (float(v) for v in velocity_enu)
        speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        if speed > self.max_speed_mps:
            scale = self.max_speed_mps / speed
            return Clamp((vx * scale, vy * scale, vz * scale), ('max_speed',))
        return Clamp((vx, vy, vz), ())

    # ------------------------------------------------------------------
    def contains(self, position_enu, origin_enu=(0.0, 0.0, 0.0)):
        """True when an ENU position is inside the envelope, floor included.

        This is the strict test, and it is the right one for a *setpoint*.
        For the vehicle's own position use ``vehicle_inside``.
        """
        return not self.clamp_position(position_enu, origin_enu).clamped

    def vehicle_inside(self, position_enu, origin_enu=(0.0, 0.0, 0.0)):
        """True when the *vehicle* is within the radius and under the ceiling.

        Drives the ``geofence_ok`` field of /skybeta/state, so a competitor
        can see the vehicle being carried out of bounds by momentum or by the
        pilot.  Unlike ``contains`` it ignores ``min_altitude_m``: that floor
        exists to stop a competitor commanding the vehicle into the ground,
        and a vehicle sitting on the pad -- or being landed by the pilot -- is
        legitimately below it.  Reporting that as a breach would make
        ``geofence_ok`` false for the whole of every flight's start and end.
        """
        reasons = self.clamp_position(position_enu, origin_enu).reasons
        return not [r for r in reasons if r != 'min_altitude']

    def describe(self):
        return (f'radius {self.geofence_radius_m:.1f} m, '
                f'altitude {self.min_altitude_m:.2f}-{self.max_altitude_m:.2f} m, '
                f'speed <= {self.max_speed_mps:.2f} m/s')
