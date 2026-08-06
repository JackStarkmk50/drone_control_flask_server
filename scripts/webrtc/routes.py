import asyncio

from flask import Blueprint, jsonify, request

from ...webrtc.signaling import (
    connect,
    disconnect,
    status
)

bp = Blueprint("webrtc", __name__)


@bp.route("/webrtc/connect", methods=["POST"])
def webrtc_connect():

    offer = request.json

    answer = asyncio.run(connect(offer))

    return jsonify(answer)


@bp.route("/webrtc/disconnect", methods=["POST"])
def webrtc_disconnect():

    asyncio.run(disconnect())

    return jsonify({"success": True})


@bp.route("/webrtc/status")
def webrtc_status():

    return jsonify(status())
