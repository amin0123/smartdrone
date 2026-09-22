from drone_core.models import Vehicle
from drone_docking.controller import dock_contact, descent_step


def test_contact_requires_all_physical_conditions():
    from dataclasses import replace
    v = Vehicle(connected=True, localized=True, landed=True)
    assert dock_contact(v, (0, 0, 0))
    for bad in (replace(v, armed=True), replace(v, landed=False), replace(v, connected=False),
                replace(v, position=(2, 0, 0)), replace(v, position=(0, 0, .5))):
        assert not dock_contact(bad, (0, 0, 0))


def test_marker_descent_requires_alignment_and_rejects_jumps():
    v = Vehicle(position=(.6, 0, 1.5), localized=True)
    result = descent_step(v, (0, 0, 0), (.05, 0, 0), .1, .1, use_marker=True)
    assert result.source == 'visual_marker'
    assert result.target[2] == 1.5
    assert not result.land
    stale = descent_step(v, (0, 0, 0), (.05, 0, 0), 8., .1, use_marker=True)
    assert stale.source == 'known_station'
    wrong = descent_step(v, (0, 0, 0), (3, 0, 0), .1, .1, use_marker=True)
    assert wrong.source == 'known_station'
    near = Vehicle(position=(.04, 0, .65), localized=True)
    assert descent_step(near, (0, 0, 0), (.05, 0, 0), .1, .1, use_marker=True).land
