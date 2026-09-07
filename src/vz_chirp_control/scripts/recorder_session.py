"""Pure OFFBOARD bag-recording policy for the vz_chirp recorder (no ROS, no I/O).

The recorder node owns the real rosbag and drives a RecorderSession with state
events; the session decides whether a bag may open/close and whether
the recorder is healthy. Keeping this free of rospy lets the policy be driven
deterministically in unit tests (open refusal on a mid-OFFBOARD respawn, open
and write failures, the finished-session tail).

The arming rule mirrors the chirp controller's OFFBOARD re-entry: only a healthy
(connected), explicitly non-OFFBOARD state arms the next OFFBOARD rising edge.
A link drop, or a (re)start that first observes OFFBOARD already active, does
not arm -- so a respawned recorder never manufactures a half-length bag.
"""


class RecorderSession:
    """Decides when a bag may be opened/closed and whether the recorder is
    healthy, given the OFFBOARD state and status feed.

    The caller keeps `_bag_open` in sync through bag_opened(), bag_closed() and
    fail(), and applies the returned 'open'/'close' actions to the real bag.
    """

    def __init__(self):
        self._offboard_active = False
        self._bag_open = False
        self._fatal = False
        self._session_over = False
        self._armed = False

    def on_state(self, connected, offboard_mode, armed):
        """Feed one /mavros/state snapshot; return 'open'/'close'/None.

        active = connected and armed and offboard_mode (the caller normally
        computes this from message.mode == 'OFFBOARD' etc.).
        """
        connected = bool(connected)
        offboard_mode = bool(offboard_mode)
        armed = bool(armed)
        active = connected and armed and offboard_mode

        if connected and not offboard_mode:
            # A healthy, explicitly non-OFFBOARD report arms the next OFFBOARD
            # edge and clears the finished-session flag.
            self._armed = True
            self._session_over = False
        if not connected:
            # A link drop must not re-arm recording: a reconnect straight back
            # into OFFBOARD must not open a half-length bag.
            self._armed = False

        was_active = self._offboard_active
        self._offboard_active = active
        if not active and was_active:
            self._session_over = False
        if active and not was_active:
            self._session_over = False
            if self._armed and not self._fatal:
                return 'open'
        if not active and was_active and self._bag_open:
            return 'close'
        return None

    def bag_opened(self):
        self._bag_open = True

    def bag_closed(self):
        self._bag_open = False

    def fail(self):
        self._fatal = True
        self._bag_open = False

    def note_terminal(self):
        """A COMPLETE/ABORT status arrived for the active session."""
        if self._offboard_active:
            self._session_over = True

    @property
    def offboard_active(self):
        return self._offboard_active

    def healthy(self):
        """Healthy = able to record now or for the next session.

        After a COMPLETE/ABORT the bag closes at the end of the 1 s tail while
        the vehicle may still hold in OFFBOARD; that finished session is still
        'healthy' because no further step can run until a real exit + re-entry.
        """
        if self._fatal:
            return False
        if not self._offboard_active:
            return True
        return self._bag_open or self._session_over

    def recording(self):
        return self._bag_open

    def __repr__(self):
        return (
            'RecorderSession(active={}, bag={}, fatal={}, over={}, armed={})'
        ).format(self._offboard_active, self._bag_open, self._fatal,
                 self._session_over, self._armed)
