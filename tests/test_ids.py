import ids


def test_duplicate_ids():
    ids.check_ids(ids.CanIds, ids.PcmChannels, ids.PwmPorts)
