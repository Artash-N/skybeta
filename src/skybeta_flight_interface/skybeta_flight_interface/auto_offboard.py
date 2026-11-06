"""When to ask PX4 for Offboard after the pilot arms.

The flow on the vehicle: the pilot selects Position and arms, and the Jetson then
enters Offboard, so arming is the go signal. This module decides whether this
arming may do that. It has no ROS in it, so every rule is tested on its own.

The rules exist because the failure is a drone flown by team code when the pilot
did not expect it:

* **Only a disarmed -> armed transition that we saw.** A flight interface that
  starts up, or a link that comes back, and finds the vehicle already armed does
  nothing: the pilot may be flying it.
* **Only from Position.** Arming in Manual or Altitude is how a pilot flies with
  no autonomy.
* **One attempt per arming.** Once the pilot takes the vehicle back, nothing here
  can take it again until it has been disarmed.
* **Only with valid position, a live setpoint and no hold in force.** With no team
  node running there is nothing to fly: the vehicle stays in Position, and PX4
  disarms it after COM_DISARM_PRFLT.
"""
from collections import namedtuple

#: PX4 can miss a command sent over the best-effort link, so it is repeated briefly.
RETRY_S = 0.5
WINDOW_S = 2.0

Inputs = namedtuple('Inputs', [
    'now_s',            # seconds, any monotonic origin
    'link',             # vehicle_status is arriving
    'armed',
    'in_position',      # nav_state is Position
    'offboard',         # nav_state is Offboard
    'position_valid',   # PX4 has a valid local position
    'setpoint_age_s',   # seconds since the team's last setpoint, None if never
    'frozen',           # the hold service is in force
])

WAIT_DISARMED = 'wait_disarmed'   # never saw this vehicle disarmed: not ours to touch
READY = 'ready'                   # disarmed: the next arming counts
REQUESTING = 'requesting'
DONE = 'done'                     # this arming has had its chance


class AutoOffboard:

    def __init__(self, enabled, setpoint_timeout_s):
        self.enabled = bool(enabled)
        self.setpoint_timeout_s = float(setpoint_timeout_s)
        self.state = WAIT_DISARMED
        self._deadline = 0.0
        self._next_send = 0.0

    def _refusal(self, s):
        """Why this arming may not enter Offboard, or None."""
        if not s.in_position:
            return 'not armed in Position'
        if not s.position_valid:
            return 'no valid local position (is VIO tracking?)'
        if s.frozen:
            return 'the hold service is in force'
        if s.setpoint_age_s is None:
            return 'no team setpoint received'
        if s.setpoint_age_s > self.setpoint_timeout_s:
            return f'the last team setpoint is {s.setpoint_age_s:.1f} s old'
        return None

    def step(self, s):
        """One tick -> (send the mode request now, message to log or None)."""
        if not self.enabled:
            return False, None
        if not s.link:
            # The vehicle could be armed by the time the link returns.
            self.state = WAIT_DISARMED
            return False, None
        if not s.armed:
            self.state = READY
            return False, None
        if self.state == READY:
            self.state = DONE
            why = self._refusal(s)
            if why:
                return False, f'staying out of Offboard: {why}'
            self.state = REQUESTING
            self._deadline = s.now_s + WINDOW_S
            self._next_send = s.now_s + RETRY_S
            return True, 'armed in Position: requesting Offboard'
        if self.state == REQUESTING:
            if s.offboard:
                self.state = DONE
                return False, 'Offboard active'
            if not s.in_position:
                self.state = DONE
                return False, 'mode changed while requesting Offboard: stopping'
            if s.now_s >= self._deadline:
                self.state = DONE
                return False, f'PX4 did not enter Offboard within {WINDOW_S:.0f} s'
            if s.now_s >= self._next_send:
                self._next_send = s.now_s + RETRY_S
                return True, None
        return False, None
