from aiortc import (
    RTCPeerConnection,
    RTCSessionDescription
)

from .audio import audio


class WebRTCPeer:

    def __init__(self):

        self.pc = RTCPeerConnection()

        self.player = audio.microphone()

        self.recorder = audio.speaker()

        if self.player.audio:
            self.pc.addTrack(self.player.audio)

        @self.pc.on("track")
        async def on_track(track):

            if track.kind == "audio":
                self.recorder.addTrack(track)
                await self.recorder.start()

        @self.pc.on("connectionstatechange")
        async def on_state():

            print(self.pc.connectionState)

            if self.pc.connectionState in [
                "closed",
                "failed",
                "disconnected"
            ]:
                await self.close()

    async def create_answer(self, offer):

        offer = RTCSessionDescription(
            sdp=offer["sdp"],
            type=offer["type"]
        )

        await self.pc.setRemoteDescription(offer)

        answer = await self.pc.createAnswer()

        await self.pc.setLocalDescription(answer)

        return {
            "type": self.pc.localDescription.type,
            "sdp": self.pc.localDescription.sdp
        }

    async def add_candidate(self, candidate):
        await self.pc.addIceCandidate(candidate)

    async def close(self):

        await self.recorder.stop()

        if self.player:
            self.player.stop()

        await self.pc.close()
