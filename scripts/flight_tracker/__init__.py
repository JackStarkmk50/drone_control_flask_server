"""
flight_tracker — per-arm flight recording for the GPS-denied airframe.

Each arm/disarm cycle becomes one flight: a row in `flights`, a 5 Hz track in
`track_points`, timeline markers in `flight_events`, and optionally an mp4 on
the SD card. Served over REST + Socket.IO so the webapp, a Windows/Mac build
and a mobile client all consume the same API.

Server wiring (see README.md for the full diff):

    from flight_tracker import FlightTracker, FrameHub
    from flight_tracker.routes import make_blueprint

    tracker = FlightTracker(db_path=..., video_dir=..., socketio=socketio)
    app.register_blueprint(make_blueprint(tracker), url_prefix="/flights")
    ...
    tracker.attach(vehicle, mc, frame_hub, start_camera)
    tracker.start()
"""

from .frames import FrameHub
from .recorder import FlightTracker, SAMPLE_HZ
from .sources import LiveSource, SimSource
from .store import FlightStore, SCHEMA_VERSION

__all__ = [
    "FlightTracker", "FrameHub", "FlightStore",
    "LiveSource", "SimSource",
    "SAMPLE_HZ", "SCHEMA_VERSION",
]
