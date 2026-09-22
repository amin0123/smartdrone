import math
import pytest
from drone_navigation.planner import Area, Box, RouteFollower, lawnmower, validate_route


def test_pattern_covers_edges_and_alternates():
    p = lawnmower(Area(origin_x=0, origin_y=0, length=8, width=7, row_spacing=3))
    assert [x[1] for x in p[::2]] == [0, 3, 6, 7]
    assert [(a[0], b[0]) for a, b in zip(p[::2], p[1::2])] == [(0, 8), (8, 0), (0, 8), (8, 0)]
    assert len(set(p)) == len(p)


@pytest.mark.parametrize('kwargs', [{'row_spacing': 0}, {'width': -1}, {'speed': float('nan')}, {'altitude': 0}])
def test_bad_configuration(kwargs):
    with pytest.raises(ValueError):
        Area(**kwargs)


def test_entire_segments_checked_not_just_waypoints():
    fence = Box((-5, -5, -1), (20, 20, 8))
    obstacle = Box((3, -1, 0), (5, 1, 4))
    with pytest.raises(ValueError, match='Blocked'):
        validate_route([(0, 0, 3), (8, 0, 3)], fence, [obstacle], .4)
    validate_route([(0, 0, 5), (8, 0, 5)], fence, [obstacle], .4)
    with pytest.raises(ValueError, match='geofence'):
        validate_route([(0, 0, 3), (30, 0, 3)], fence, [], .4)


def test_follower_waits_for_aircraft_and_cannot_skip_corner():
    f = RouteFollower((0, 0, 2), [(2, 0, 2), (2, 2, 2)], speed=1, max_lead=.8)
    for _ in range(100):
        f.step((0, 0, 2), .1)
    assert f.index == 0
    assert math.dist(f.reference, (0, 0, 2)) <= .8
    actual = f.reference
    for _ in range(80):
        old = actual
        actual = f.step(actual, .1)
        assert math.dist(old, actual) <= .100001
    assert f.complete
    assert actual == pytest.approx((2, 2, 2))


def test_paused_executor_does_not_jump_target():
    f = RouteFollower((0, 0, 2), [(8, 0, 2)], speed=1)
    assert f.step((0, 0, 2), 120)[0] == pytest.approx(.2)
