import asyncio

from flask import Blueprint, jsonify, request

from .signaling import (
    connect,
    disconnect,
    status
)

bp = Blueprint("webrtc", __name__)

# Route paths are relative to the blueprint's url_prefix ("/webrtc" in
# new_api_server_1.py). They previously repeated the prefix, producing
# /webrtc/webrtc/connect, while the client calls /webrtc/connect -> 404.


@bp.route("/connect", methods=["POST"])
def webrtc_connect():

    offer = request.json

    answer = asyncio.run(connect(offer))

    return jsonify(answer)


@bp.route("/disconnect", methods=["POST"])
def webrtc_disconnect():

    asyncio.run(disconnect())

    return jsonify({"success": True})


@bp.route("/status")
def webrtc_status():

    return jsonify(status())
