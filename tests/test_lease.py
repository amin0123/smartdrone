from drone_px4_interface.lease import ControllerLease


def test_dead_controller_and_replayed_sequence_do_not_keep_offboard_alive():
    lease = ControllerLease(.5)
    for i in range(15):
        assert lease.accept(i, i * .1)
    assert lease.warmed_up(1.45)
    assert not lease.accept(14, 1.8)
    assert not lease.fresh(2.)
    assert lease.accept(15, 2.1)
    assert not lease.warmed_up(2.1)


def test_clock_cannot_go_backwards_and_reset_clears_warmup():
    lease = ControllerLease()
    lease.accept(2, 10)
    assert not lease.fresh(9)
    lease.reset()
    assert not lease.warmed_up(10)
    assert lease.accept(1, 10)
