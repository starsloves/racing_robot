import math

import pytest

from racing_stage3.stage3_return_navigator import Stage3ReturnNavigator


def test_entry_anchor_rotates_odom_translation_delta_into_map():
    entry_map = (2.80, 3.25)
    entry_odom = (4.116, 2.106)

    position = Stage3ReturnNavigator._position_from_entry_anchor(
        entry_map, entry_odom, (4.416, 1.906), 0.0,
    )

    assert position == (3.10, 3.05)


def test_entry_anchor_applies_map_from_odom_rotation():
    position = Stage3ReturnNavigator._position_from_entry_anchor(
        (2.0, 3.0), (1.0, 1.0), (2.0, 1.0), math.pi / 2.0,
    )

    assert position == pytest.approx((2.0, 4.0))


def test_entry_anchor_keeps_configured_map_position_at_odom_reference():
    entry_map = (2.80, 3.25)
    entry_odom = (4.116, 2.106)

    assert Stage3ReturnNavigator._position_from_entry_anchor(
        entry_map, entry_odom, entry_odom, math.pi / 2.0,
    ) == entry_map
