"""The rules for entering Offboard after arming. No ROS needed."""
from skybeta_flight_interface.auto_offboard import (
    AutoOffboard, DONE, Inputs, READY, REQUESTING, WAIT_DISARMED)


def inputs(now=0.0, link=True, armed=False, in_position=True, offboard=False,
           position_valid=True, setpoint_age=0.1, frozen=False):
    return Inputs(now, link, armed, in_position, offboard, position_valid,
                  setpoint_age, frozen)


def ready():
    auto = AutoOffboard(True, 1.0)
    auto.step(inputs())
    assert auto.state == READY
    return auto


def test_arming_in_position_requests_offboard():
    request, message = ready().step(inputs(armed=True))
    assert request is True
    assert 'requesting' in message


def test_disabled_does_nothing():
    auto = AutoOffboard(False, 1.0)
    auto.step(inputs())
    assert auto.step(inputs(armed=True)) == (False, None)


def test_already_armed_at_startup_is_left_alone():
    # A flight interface restarted in flight must not seize the vehicle.
    auto = AutoOffboard(True, 1.0)
    for t in range(5):
        assert auto.step(inputs(now=float(t), armed=True)) == (False, None)
    assert auto.state == WAIT_DISARMED


def test_armed_during_a_link_dropout_is_left_alone():
    # The pilot may have armed and taken off while the link was down.
    auto = ready()
    assert auto.step(inputs(link=False)) == (False, None)
    assert auto.step(inputs(now=1.0, armed=True)) == (False, None)
    assert auto.state == WAIT_DISARMED


def test_a_link_dropout_while_requesting_stops_it():
    auto = ready()
    auto.step(inputs(armed=True))
    auto.step(inputs(now=0.1, armed=True, link=False))
    assert auto.step(inputs(now=0.6, armed=True)) == (False, None)


def assert_refused(text, **kw):
    auto = ready()
    request, message = auto.step(inputs(armed=True, **kw))
    assert request is False
    assert text in message
    assert auto.state == DONE
    # and it does not get a second chance when the condition clears
    assert auto.step(inputs(now=1.0, armed=True)) == (False, None)


def test_arming_in_another_mode_means_no_autonomy():
    assert_refused('not armed in Position', in_position=False)


def test_no_valid_position():
    assert_refused('valid local position', position_valid=False)


def test_hold_in_force():
    assert_refused('hold', frozen=True)


def test_no_team_node_running():
    assert_refused('no team setpoint', setpoint_age=None)


def test_team_node_went_quiet():
    assert_refused('2.5 s old', setpoint_age=2.5)


def test_repeats_until_px4_answers():
    auto = ready()
    assert auto.step(inputs(now=0.0, armed=True))[0] is True
    assert auto.step(inputs(now=0.1, armed=True))[0] is False
    assert auto.step(inputs(now=0.5, armed=True))[0] is True
    assert auto.step(inputs(now=0.6, armed=True))[0] is False
    assert auto.step(inputs(now=1.0, armed=True))[0] is True


def test_stops_once_in_offboard():
    auto = ready()
    auto.step(inputs(armed=True))
    request, message = auto.step(inputs(now=0.5, armed=True, in_position=False,
                                        offboard=True))
    assert request is False
    assert 'active' in message
    assert auto.state == DONE


def test_gives_up_after_the_window():
    auto = ready()
    auto.step(inputs(armed=True))
    request, message = auto.step(inputs(now=2.0, armed=True))
    assert request is False
    assert 'did not enter' in message
    assert auto.state == DONE


def test_pilot_moving_the_switch_stops_the_request():
    auto = ready()
    auto.step(inputs(armed=True))
    request, message = auto.step(inputs(now=0.1, armed=True, in_position=False))
    assert request is False
    assert 'mode changed' in message


def test_never_retakes_the_vehicle_from_the_pilot():
    auto = ready()
    auto.step(inputs(armed=True))
    auto.step(inputs(now=0.2, armed=True, in_position=False, offboard=True))
    # Pilot takes over: back in Position, still armed, for the rest of the flight.
    for t in range(100):
        request, _ = auto.step(inputs(now=1.0 + t, armed=True))
        assert request is False


def test_each_arming_gets_its_own_chance():
    auto = ready()
    auto.step(inputs(armed=True))
    auto.step(inputs(now=1.0, armed=True, in_position=False, offboard=True))
    auto.step(inputs(now=2.0, armed=False))
    assert auto.state == READY
    assert auto.step(inputs(now=3.0, armed=True))[0] is True
    assert auto.state == REQUESTING
