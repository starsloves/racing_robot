from racing_stage3.stage3_return_navigator import Stage3ReturnNavigator


def test_entry_anchor_uses_only_odom_translation_delta():
    entry_map = (2.80, 3.25)
    entry_odom = (4.116, 2.106)

    position = Stage3ReturnNavigator._position_from_entry_anchor(
        entry_map, entry_odom, (4.416, 1.906)
    )

    assert position == (3.10, 3.05)


def test_entry_anchor_keeps_configured_map_position_at_odom_reference():
    entry_map = (2.80, 3.25)
    entry_odom = (4.116, 2.106)

    assert Stage3ReturnNavigator._position_from_entry_anchor(
        entry_map, entry_odom, entry_odom
    ) == entry_map
