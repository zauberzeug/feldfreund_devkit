# pylint: disable=protected-access
from nicegui import Client
from nicegui.page import page

from feldfreund_devkit.navigation.recorded_track import RecordedTrack


def _open(devkit_system):
    """Open the recorder for a fresh track and return the live dialog."""
    devkit_system.recorded_track_navigation._open_recorder(RecordedTrack())
    return devkit_system.recorded_track_navigation._current_recorder


async def test_reopening_recorder_releases_the_previous_one(devkit_system):
    """Each open must tear the previous recorder down instead of leaking its
    controller subscriptions and 1-second tick timer into a long-lived session."""
    controller = devkit_system.track_recording_controller

    with Client(page('/')):  # standalone slot to build the dialog into (no running server)
        previous = _open(devkit_system)
        assert not previous._tick_timer._is_canceled
        waypoint_subscribers = len(controller.WAYPOINT_ADDED.callbacks)

        current = _open(devkit_system)
        assert current is not previous
        # the previous recorder is fully released ...
        assert previous._tick_timer._is_canceled
        assert not any(c.func == previous._on_waypoint_added for c in controller.WAYPOINT_ADDED.callbacks)
        # ... and the replacement does not accumulate subscriptions on top of it.
        assert len(controller.WAYPOINT_ADDED.callbacks) == waypoint_subscribers


async def test_tear_down_is_idempotent(devkit_system):
    """``hide`` and the owner may both tear a recorder down; doing so twice is a no-op."""
    controller = devkit_system.track_recording_controller

    with Client(page('/')):
        recorder = _open(devkit_system)
        recorder.tear_down()
        recorder.tear_down()

        assert recorder._tick_timer._is_canceled
        assert not any(c.func == recorder._on_waypoint_added for c in controller.WAYPOINT_ADDED.callbacks)
