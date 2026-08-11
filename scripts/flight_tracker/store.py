"""
SQLite persistence for the flight tracker.

One file, no server process, queryable with the sqlite3 CLI. WAL journal mode is
enabled so the recorder thread can write while Flask request threads read.

Coordinate convention
---------------------
The aircraft is GPS-denied, so lat/lon are meaningless here. Position is stored
in metres in the EKF local frame, RELATIVE TO THE ARM POINT:

    x_m  = east,  positive right of the arm heading-independent origin
    y_m  = north, positive forward of the origin
    z_m  = UP-positive altitude (NED "down" is negated on the way in)

The EKF local origin resets when the vehicle arms, which is exactly why each arm
produces an independent track. flights.origin_* records the raw local_frame
reading at arm so a track can be re-based later if needed.
"""

import os
import sqlite3
import threading
import uuid as _uuid
from datetime import datetime, timezone

SCHEMA_VERSION = 1

# Columns written for every sample, in insert order. Kept as a module constant so
# the recorder can build its batch tuples without repeating the list.
TRACK_COLUMNS = (
    "flight_id", "t_ms",
    "x_m", "y_m", "z_m", "pos_source",
    "alt_rf_m", "alt_rel_m",
    "roll", "pitch", "yaw",
    "vx", "vy", "vz", "groundspeed",
    "rc_roll", "rc_pitch", "rc_throttle", "rc_yaw",
    "corr_roll", "corr_pitch",
    "mode", "armed",
    "battery_v", "battery_pct", "battery_current",
    "ekf_ok", "vibe_x", "vibe_y", "vibe_z",
    "sats", "gps_fix",
)

_DDL = """
CREATE TABLE IF NOT EXISTS flights (
    id                    INTEGER PRIMARY KEY AUTOINCREMENT,
    uuid                  TEXT    NOT NULL UNIQUE,
    label                 TEXT,
    started_at            TEXT    NOT NULL,   -- ISO8601 UTC
    started_at_local      TEXT,               -- ISO8601 with local offset, for display
    ended_at              TEXT,
    duration_s            REAL,
    end_reason            TEXT,               -- disarm|manual|interrupted|error
    source                TEXT    NOT NULL DEFAULT 'live',   -- live|sim

    -- Raw EKF local_frame reading at arm. Track rows are stored relative to it.
    origin_north          REAL,
    origin_east           REAL,
    origin_down           REAL,

    -- Summary, filled in when the flight closes.
    point_count           INTEGER DEFAULT 0,
    max_alt_m             REAL,
    max_dist_m            REAL,               -- furthest straight-line from origin
    distance_m            REAL,               -- path length travelled
    max_groundspeed       REAL,
    battery_start_v       REAL,
    battery_end_v         REAL,
    pos_source            TEXT,               -- dominant source: ekf|deadreckon|none

    -- Video (recorded locally on the Pi; cloud upload is a later phase).
    video_path            TEXT,
    video_codec           TEXT,
    video_start_offset_ms INTEGER,            -- video t=0 relative to flight t=0
    video_duration_s      REAL,
    video_size_bytes      INTEGER,

    notes                 TEXT,
    uploaded_at           TEXT,               -- reserved: cloud sync marker
    schema_version        INTEGER DEFAULT 1
);

CREATE TABLE IF NOT EXISTS track_points (
    id           INTEGER PRIMARY KEY AUTOINCREMENT,
    flight_id    INTEGER NOT NULL REFERENCES flights(id) ON DELETE CASCADE,
    t_ms         INTEGER NOT NULL,            -- ms since flight start; scrubber key

    -- position, metres, relative to arm origin. z_m is UP-positive.
    x_m          REAL,
    y_m          REAL,
    z_m          REAL,
    pos_source   TEXT,                        -- ekf|deadreckon|none

    alt_rf_m     REAL,                        -- rangefinder AGL
    alt_rel_m    REAL,                        -- barometric relative alt

    roll         REAL,                        -- radians
    pitch        REAL,
    yaw          REAL,

    vx           REAL,                        -- m/s, NED
    vy           REAL,
    vz           REAL,
    groundspeed  REAL,

    -- Control path: what the Pi commanded, and what the closed-loop corrector
    -- added on top. Recording both is what makes this a tuning record and not
    -- just a breadcrumb trail.
    rc_roll      INTEGER,
    rc_pitch     INTEGER,
    rc_throttle  INTEGER,
    rc_yaw       INTEGER,
    corr_roll    INTEGER,
    corr_pitch   INTEGER,

    mode         TEXT,
    armed        INTEGER,

    battery_v       REAL,
    battery_pct     REAL,
    battery_current REAL,

    ekf_ok       INTEGER,
    vibe_x       REAL,
    vibe_y       REAL,
    vibe_z       REAL,
    sats         INTEGER,
    gps_fix      INTEGER
);

CREATE INDEX IF NOT EXISTS idx_track_flight_t
    ON track_points(flight_id, t_ms);

CREATE TABLE IF NOT EXISTS flight_events (
    id        INTEGER PRIMARY KEY AUTOINCREMENT,
    flight_id INTEGER NOT NULL REFERENCES flights(id) ON DELETE CASCADE,
    t_ms      INTEGER NOT NULL,
    kind      TEXT    NOT NULL,   -- arm|disarm|mode|rc|takeoff|land|error|marker
    detail    TEXT
);

CREATE INDEX IF NOT EXISTS idx_event_flight_t
    ON flight_events(flight_id, t_ms);
"""


def utc_now_iso():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def local_now_iso():
    return datetime.now().astimezone().isoformat(timespec="seconds")


class FlightStore:
    """
    Thin SQLite wrapper.

    The recorder thread holds one write connection guarded by a lock; every
    read path opens a short-lived connection of its own. WAL mode makes those
    reads safe against a concurrent write.
    """

    def __init__(self, db_path):
        self.db_path = os.path.abspath(db_path)
        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        self._wlock = threading.Lock()
        self._w = self._connect()
        self._init_schema()

    # ── connection helpers ────────────────────────────────────────────

    def _connect(self):
        conn = sqlite3.connect(self.db_path, check_same_thread=False, timeout=10.0)
        conn.row_factory = sqlite3.Row
        conn.execute("PRAGMA journal_mode=WAL")
        # NORMAL trades a fsync-per-commit for a fsync-per-checkpoint. On an SD
        # card that is the difference between a healthy card and a worn one.
        conn.execute("PRAGMA synchronous=NORMAL")
        conn.execute("PRAGMA foreign_keys=ON")
        return conn

    def _init_schema(self):
        with self._wlock:
            self._w.executescript(_DDL)
            self._w.execute(f"PRAGMA user_version={SCHEMA_VERSION}")
            self._w.commit()

    def close(self):
        with self._wlock:
            try:
                self._w.commit()
                self._w.close()
            except Exception:
                pass

    # ── writes (recorder thread) ──────────────────────────────────────

    def open_flight(self, source="live", label=None, origin=None):
        """Insert a new flight row and return (id, uuid)."""
        fu = str(_uuid.uuid4())
        o_n, o_e, o_d = (origin or (None, None, None))
        with self._wlock:
            cur = self._w.execute(
                """INSERT INTO flights
                   (uuid, label, started_at, started_at_local, source,
                    origin_north, origin_east, origin_down, schema_version)
                   VALUES (?,?,?,?,?,?,?,?,?)""",
                (fu, label, utc_now_iso(), local_now_iso(), source,
                 o_n, o_e, o_d, SCHEMA_VERSION),
            )
            self._w.commit()
            return cur.lastrowid, fu

    def insert_points(self, rows):
        """Batch-insert track rows. Each row is a tuple matching TRACK_COLUMNS."""
        if not rows:
            return
        ph = ",".join("?" * len(TRACK_COLUMNS))
        sql = f"INSERT INTO track_points ({','.join(TRACK_COLUMNS)}) VALUES ({ph})"
        with self._wlock:
            self._w.executemany(sql, rows)
            self._w.commit()

    def insert_event(self, flight_id, t_ms, kind, detail=None):
        with self._wlock:
            self._w.execute(
                "INSERT INTO flight_events (flight_id, t_ms, kind, detail) VALUES (?,?,?,?)",
                (flight_id, int(t_ms), kind, detail),
            )
            self._w.commit()

    def close_flight(self, flight_id, end_reason, summary):
        """Stamp end time and write the computed summary fields."""
        with self._wlock:
            self._w.execute(
                """UPDATE flights SET
                     ended_at=?, duration_s=?, end_reason=?,
                     point_count=?, max_alt_m=?, max_dist_m=?, distance_m=?,
                     max_groundspeed=?, battery_start_v=?, battery_end_v=?,
                     pos_source=?
                   WHERE id=?""",
                (utc_now_iso(), summary.get("duration_s"), end_reason,
                 summary.get("point_count", 0), summary.get("max_alt_m"),
                 summary.get("max_dist_m"), summary.get("distance_m"),
                 summary.get("max_groundspeed"), summary.get("battery_start_v"),
                 summary.get("battery_end_v"), summary.get("pos_source"),
                 flight_id),
            )
            self._w.commit()

    def set_video(self, flight_id, path, codec, start_offset_ms,
                  duration_s=None, size_bytes=None):
        with self._wlock:
            self._w.execute(
                """UPDATE flights SET
                     video_path=?, video_codec=?, video_start_offset_ms=?,
                     video_duration_s=?, video_size_bytes=?
                   WHERE id=?""",
                (path, codec, start_offset_ms, duration_s, size_bytes, flight_id),
            )
            self._w.commit()

    def close_orphans(self):
        """
        Any flight with no ended_at is a flight the process died during. Called
        once at startup so a power-cut mid-flight does not leave a row that the
        UI renders as 'still flying' forever.
        """
        with self._wlock:
            cur = self._w.execute(
                """SELECT f.id,
                          (SELECT MAX(t_ms) FROM track_points WHERE flight_id=f.id) AS last_t,
                          (SELECT COUNT(*)  FROM track_points WHERE flight_id=f.id) AS n
                   FROM flights f WHERE f.ended_at IS NULL"""
            )
            rows = cur.fetchall()
            for r in rows:
                self._w.execute(
                    """UPDATE flights SET ended_at=?, duration_s=?, end_reason='interrupted',
                                          point_count=? WHERE id=?""",
                    (utc_now_iso(), (r["last_t"] or 0) / 1000.0, r["n"] or 0, r["id"]),
                )
            self._w.commit()
            return [r["id"] for r in rows]

    # ── reads (request threads) ───────────────────────────────────────

    def list_flights(self, limit=50, offset=0, include_empty=False):
        q = "SELECT * FROM flights"
        if not include_empty:
            q += " WHERE point_count > 0 OR ended_at IS NULL"
        q += " ORDER BY id DESC LIMIT ? OFFSET ?"
        conn = self._connect()
        try:
            return [dict(r) for r in conn.execute(q, (limit, offset))]
        finally:
            conn.close()

    def get_flight(self, flight_id):
        conn = self._connect()
        try:
            r = conn.execute("SELECT * FROM flights WHERE id=?", (flight_id,)).fetchone()
            return dict(r) if r else None
        finally:
            conn.close()

    def get_track(self, flight_id, fields=None, decimate=1, t_from=None, t_to=None):
        """
        Return the track as a dict-of-columns (compact on the wire — no repeated
        key names for thousands of rows).

        decimate=N returns every Nth row, for zoomed-out overview rendering.
        """
        cols = list(fields) if fields else [c for c in TRACK_COLUMNS if c != "flight_id"]
        cols = [c for c in cols if c in TRACK_COLUMNS]
        if "t_ms" not in cols:
            cols.insert(0, "t_ms")

        where = ["flight_id=?"]
        args = [flight_id]
        if t_from is not None:
            where.append("t_ms>=?")
            args.append(int(t_from))
        if t_to is not None:
            where.append("t_ms<=?")
            args.append(int(t_to))

        q = f"SELECT {','.join(cols)} FROM track_points WHERE {' AND '.join(where)} ORDER BY t_ms"
        conn = self._connect()
        try:
            rows = conn.execute(q, args).fetchall()
        finally:
            conn.close()

        if decimate and decimate > 1:
            rows = rows[::int(decimate)]

        out = {c: [] for c in cols}
        for r in rows:
            for c in cols:
                out[c].append(r[c])
        return {"count": len(rows), "columns": cols, "data": out}

    def get_events(self, flight_id):
        conn = self._connect()
        try:
            return [dict(r) for r in conn.execute(
                "SELECT t_ms, kind, detail FROM flight_events WHERE flight_id=? ORDER BY t_ms",
                (flight_id,))]
        finally:
            conn.close()

    def set_label(self, flight_id, label):
        with self._wlock:
            self._w.execute("UPDATE flights SET label=? WHERE id=?", (label, flight_id))
            self._w.commit()

    def set_notes(self, flight_id, notes):
        with self._wlock:
            self._w.execute("UPDATE flights SET notes=? WHERE id=?", (notes, flight_id))
            self._w.commit()

    def delete_flight(self, flight_id):
        """Delete the row (points/events cascade). Returns the video path, if any,
        so the caller can decide whether to unlink the file."""
        f = self.get_flight(flight_id)
        with self._wlock:
            self._w.execute("DELETE FROM flight_events WHERE flight_id=?", (flight_id,))
            self._w.execute("DELETE FROM track_points  WHERE flight_id=?", (flight_id,))
            self._w.execute("DELETE FROM flights       WHERE id=?", (flight_id,))
            self._w.commit()
        return (f or {}).get("video_path")

    def stats(self):
        conn = self._connect()
        try:
            r = conn.execute(
                """SELECT COUNT(*) AS flights,
                          COALESCE(SUM(duration_s),0) AS total_s,
                          COALESCE(SUM(video_size_bytes),0) AS video_bytes
                   FROM flights"""
            ).fetchone()
            p = conn.execute("SELECT COUNT(*) AS n FROM track_points").fetchone()
            size = os.path.getsize(self.db_path) if os.path.exists(self.db_path) else 0
            return {
                "flights": r["flights"],
                "total_flight_time_s": round(r["total_s"], 1),
                "track_points": p["n"],
                "db_bytes": size,
                "video_bytes": r["video_bytes"],
            }
        finally:
            conn.close()
