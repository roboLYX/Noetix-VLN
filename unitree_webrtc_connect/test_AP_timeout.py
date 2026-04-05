import asyncio
from unitree_webrtc_connect import UnitreeWebRTCConnection, WebRTCConnectionMethod

async def main():
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP)
    try:
        await asyncio.wait_for(conn.connect(), timeout=20)
        print("[OK] connected")
    except asyncio.TimeoutError:
        print("[TIMEOUT] connect() > 20s")
    except Exception as e:
        print(f"[ERROR] {type(e).__name__}: {e}")
    finally:
        try:
            await conn.disconnect()
        except Exception:
            pass

asyncio.run(main())