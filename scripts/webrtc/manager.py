import asyncio
from ...webrtc.peer import WebRTCPeer


class WebRTCManager:
    def __init__(self):
        self.peer = None
        self.connected = False
        self.client_id = None

    async def connect(self, offer):
        if self.peer:
            await self.disconnect()

        self.peer = WebRTCPeer()

        answer = await self.peer.create_answer(offer)

        self.connected = True

        return answer

    async def disconnect(self):
        if self.peer:
            await self.peer.close()

        self.peer = None
        self.connected = False
        self.client_id = None

    def status(self):
        return {
            "connected": self.connected
        }


manager = WebRTCManager()
