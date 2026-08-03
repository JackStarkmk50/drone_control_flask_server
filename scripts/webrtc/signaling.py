from .manager import manager


async def connect(data):
    return await manager.connect(data)


async def disconnect():
    await manager.disconnect()


def status():
    return manager.status()
