import math
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from drone_core.frames import (
    enu_to_ned, ned_to_enu, flu_to_frd, frd_to_flu, yaw_enu_to_ned,
    yaw_ned_to_enu, attitude_ned_frd_to_enu_flu, attitude_enu_flu_to_ned_frd,
)


def test_named_axes_and_inverse():
    assert enu_to_ned((1, 0, 0)) == (0, 1, 0)  # east
    assert enu_to_ned((0, 1, 0)) == (1, 0, 0)  # north
    assert enu_to_ned((0, 0, 2)) == (0, 0, -2)  # up
    assert ned_to_enu(enu_to_ned((2, -3, 4))) == (2, -3, 4)
    assert flu_to_frd((1, 2, 3)) == (1, -2, -3)
    assert frd_to_flu(flu_to_frd((1, 2, 3))) == (1, 2, 3)


def test_yaw_convention():
    assert yaw_enu_to_ned(0) == pytest.approx(math.pi / 2)
    assert yaw_enu_to_ned(math.pi / 2) == pytest.approx(0)
    for angle in np.linspace(-3.1, 3.1, 21):
        assert yaw_ned_to_enu(yaw_enu_to_ned(angle)) == pytest.approx(angle)


def test_full_attitude_roundtrip_and_north_facing_vehicle():
    north = Rotation.from_quat(attitude_ned_frd_to_enu_flu((0, 0, 0, 1)))
    np.testing.assert_allclose(north.apply([1, 0, 0]), [0, 1, 0], atol=1e-12)
    for r in Rotation.random(100, random_state=17):
        q = attitude_enu_flu_to_ned_frd(attitude_ned_frd_to_enu_flu(r.as_quat()))
        np.testing.assert_allclose(Rotation.from_quat(q).as_matrix(), r.as_matrix(), atol=1e-12)


@pytest.mark.parametrize('v', [(1, 2), (1, 2, float('nan')), (1, 2, float('inf'))])
def test_reject_invalid_vectors(v):
    with pytest.raises(ValueError):
        enu_to_ned(v)
