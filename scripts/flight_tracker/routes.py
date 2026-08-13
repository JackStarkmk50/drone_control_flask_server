"""
REST blueprint for the flight tracker.

Mounted at /flights by the server. Everything a client needs — webapp, the
Windows/Mac build and mobile — comes through here, so there is exactly one
implementation of the history logic no matter how many front-ends exist.

Track payloads are column-oriented ({"x_m": [...], "y_m": [...]}) rather than a
list of objects. For a 10-minute flight that is roughly a third of the bytes,
because the key names are not repeated 3000 times.
"""

import os
import re

from flask import Blueprint, jsonify, request, Response, send_file


def make_blueprint(tracker):
    bp = Blueprint("flights", __name__)
    store = tracker.store

    def ok(**kw):
        d = {"success": True}
        d.update(kw)
        return jsonify(d)

    def err(msg, code=400):
        return jsonify({"success": False, "message": msg}), code

    # ── listing ───────────────────────────────────────────────────────

    @bp.route("", methods=["GET"])
    @bp.route("/", methods=["GET"])
    def list_flights():
        limit = min(int(request.args.get("limit", 50)), 500)
        offset = int(request.args.get("offset", 0))
        include_empty = request.args.get("include_empty", "0") in ("1", "true")
        rows = store.list_flights(limit=limit, offset=offset,
                                  include_empty=include_empty)
        for r in rows:
            r["has_video"] = bool(r.get("video_path"))
            r.pop("video_path", None)      # path is server-internal
        return ok(flights=rows, count=len(rows), limit=limit, offset=offset)

    @bp.route("/current", methods=["GET"])
    def current():
        return ok(**tracker.status())

    @bp.route("/stats", methods=["GET"])
    def stats():
        s = store.stats()
        try:
            vd = tracker.video_dir
            s["video_dir"] = vd
            s["video_files"] = len([f for f in os.listdir(vd)]) if os.path.isdir(vd) else 0
        except Exception:
            pass
        return ok(**s)

    # ── one flight ────────────────────────────────────────────────────

    @bp.route("/<int:fid>", methods=["GET"])
    def get_flight(fid):
        f = store.get_flight(fid)
        if not f:
            return err("flight not found", 404)
        f["has_video"] = bool(f.get("video_path"))
        f["video_url"] = f"/flights/{fid}/video" if f["has_video"] else None
        f.pop("video_path", None)
        f["events"] = store.get_events(fid)
        f["is_recording"] = (tracker.flight_id == fid)
        return ok(flight=f)

    @bp.route("/<int:fid>/track", methods=["GET"])
    def get_track(fid):
        if not store.get_flight(fid):
            return err("flight not found", 404)
        fields = request.args.get("fields")
        fields = [f.strip() for f in fields.split(",")] if fields else None
        decimate = max(1, int(request.args.get("decimate", 1)))
        t_from = request.args.get("from", type=int)
        t_to = request.args.get("to", type=int)
        track = store.get_track(fid, fields=fields, decimate=decimate,
                                t_from=t_from, t_to=t_to)
        return ok(flight_id=fid, **track)

    @bp.route("/<int:fid>/events", methods=["GET"])
    def get_events(fid):
        if not store.get_flight(fid):
            return err("flight not found", 404)
        return ok(flight_id=fid, events=store.get_events(fid))

    @bp.route("/<int:fid>/label", methods=["POST"])
    def set_label(fid):
        if not store.get_flight(fid):
            return err("flight not found", 404)
        data = request.get_json(silent=True) or {}
        if "label" in data:
            store.set_label(fid, str(data["label"])[:200])
        if "notes" in data:
            store.set_notes(fid, str(data["notes"])[:4000])
        return ok(flight=store.get_flight(fid))

    @bp.route("/<int:fid>", methods=["DELETE"])
    def delete_flight(fid):
        if tracker.flight_id == fid:
            return err("cannot delete the flight currently recording", 409)
        if not store.get_flight(fid):
            return err("flight not found", 404)
        vpath = store.delete_flight(fid)
        removed = False
        if vpath and request.args.get("keep_video", "0") not in ("1", "true"):
            try:
                if os.path.exists(vpath):
                    os.remove(vpath)
                    removed = True
            except Exception as e:
                print(f"[tracker] could not remove {vpath}: {e}")
        return ok(deleted=fid, video_removed=removed)

    @bp.route("/<int:fid>/mark", methods=["POST"])
    def mark(fid):
        """Drop a user marker on the live timeline ('interesting thing here')."""
        if tracker.flight_id != fid:
            return err("that flight is not recording", 409)
        data = request.get_json(silent=True) or {}
        tracker.mark("marker", str(data.get("detail", ""))[:200])
        return ok()

    # ── video ─────────────────────────────────────────────────────────

    @bp.route("/<int:fid>/video", methods=["GET"])
    def video(fid):
        """
        Serve the flight video with HTTP Range support. Range is what lets a
        browser <video> element seek — without it the scrubber can only play
        from the start, which defeats syncing video to t_ms.
        """
        f = store.get_flight(fid)
        if not f:
            return err("flight not found", 404)
        path = f.get("video_path")
        if not path or not os.path.exists(path):
            return err("no video for this flight", 404)

        size = os.path.getsize(path)
        mime = "video/mp4" if path.endswith(".mp4") else "video/x-msvideo"
        range_hdr = request.headers.get("Range")
        if not range_hdr:
            resp = send_file(path, mimetype=mime, conditional=True)
            resp.headers["Accept-Ranges"] = "bytes"
            return resp

        m = re.match(r"bytes=(\d+)-(\d*)", range_hdr)
        if not m:
            return err("bad Range header", 416)
        start = int(m.group(1))
        end = int(m.group(2)) if m.group(2) else size - 1
        end = min(end, size - 1)
        if start > end:
            return err("range out of bounds", 416)
        length = end - start + 1

        def chunks(chunk=64 * 1024):
            with open(path, "rb") as fh:
                fh.seek(start)
                left = length
                while left > 0:
                    buf = fh.read(min(chunk, left))
                    if not buf:
                        break
                    left -= len(buf)
                    yield buf

        resp = Response(chunks(), status=206, mimetype=mime,
                        direct_passthrough=True)
        resp.headers["Content-Range"] = f"bytes {start}-{end}/{size}"
        resp.headers["Accept-Ranges"] = "bytes"
        resp.headers["Content-Length"] = str(length)
        return resp

    # ── simulator ─────────────────────────────────────────────────────

    @bp.route("/sim/start", methods=["POST"])
    def sim_start():
        """
        Synthetic flight, no Pixhawk required. Produces identical DB rows,
        socket events and REST payloads to a real flight, so the whole UI can
        be built and demoed without waiting for space to fly.
        """
        d = request.get_json(silent=True) or {}
        profile = str(d.get("profile", "square")).lower()
        if profile not in ("square", "nav"):
            return err("profile must be 'square' or 'nav'")

        # The two profiles take different kwargs; handing the nav sim a square's
        # `side` would be a TypeError, so each gets its own whitelist.
        allowed = (("alt", float), ("battery_v", float), ("yaw_deg", float)) \
            if profile == "nav" else \
            (("alt", float), ("side", float), ("speed", float),
             ("hover_s", float), ("battery_v", float))

        kw = {}
        for k, cast in allowed:
            if k in d:
                try:
                    kw[k] = cast(d[k])
                except (TypeError, ValueError):
                    return err(f"bad value for {k}")
        res = tracker.start_sim(profile=profile, **kw)
        return (ok(**res) if res.get("success")
                else (jsonify(res), 409))

    @bp.route("/sim/stop", methods=["POST"])
    def sim_stop():
        return jsonify(tracker.stop_sim())

    return bp
