from coordinator.core.zone_lease import ZoneLease


def test_free_zone_is_granted():
    lease = ZoneLease(lease_seconds=60.0)
    assert lease.request("phineas", now=0.0) == (True, "phineas")
    assert lease.occupant(now=1.0) == "phineas"


def test_held_zone_is_denied_to_other_robot():
    lease = ZoneLease(lease_seconds=60.0)
    lease.request("phineas", now=0.0)
    assert lease.request("ferb", now=1.0) == (False, "phineas")


def test_rerequest_by_holder_is_idempotent_and_renews():
    lease = ZoneLease(lease_seconds=60.0)
    lease.request("phineas", now=0.0)
    assert lease.request("phineas", now=50.0) == (True, "phineas")
    assert lease.occupant(now=100.0) == "phineas"


def test_expired_lease_frees_the_zone():
    lease = ZoneLease(lease_seconds=60.0)
    lease.request("phineas", now=0.0)
    assert lease.occupant(now=60.5) is None
    assert lease.request("ferb", now=61.0) == (True, "ferb")


def test_only_holder_can_release():
    lease = ZoneLease()
    lease.request("phineas", now=0.0)
    assert not lease.release("ferb", delivered=True, color="green", now=1.0)
    assert lease.occupant(now=1.0) == "phineas"
    assert lease.release("phineas", delivered=False, color="", now=2.0)
    assert lease.occupant(now=2.0) is None


def test_delivered_release_increments_official_count():
    lease = ZoneLease()
    lease.request("phineas", now=0.0)
    lease.release("phineas", delivered=True, color="red", now=5.0)
    lease.request("phineas", now=6.0)
    lease.release("phineas", delivered=False, color="", now=7.0)
    lease.request("ferb", now=8.0)
    lease.release("ferb", delivered=True, color="blue", now=9.0)
    assert lease.counts == {"phineas": 1, "ferb": 1}
    assert lease.delivered_total("phineas") == 1
    assert lease.delivered_total("perry") == 0


def test_release_after_expiry_is_rejected():
    lease = ZoneLease(lease_seconds=10.0)
    lease.request("phineas", now=0.0)
    assert not lease.release("phineas", delivered=True, color="red", now=11.0)
    assert lease.delivered_total("phineas") == 0
