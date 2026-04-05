import asyncio
import json
from unitree_webrtc_connect import (
    UnitreeWebRTCConnection,
    WebRTCConnectionMethod,
    RTC_TOPIC,
    SPORT_CMD,
)

async def request(conn, topic, api_id, parameter=None):
    payload = {"api_id": api_id}
    if parameter is not None:
        payload["parameter"] = parameter
    return await conn.datachannel.pub_sub.publish_request_new(topic, payload)

async def main():
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP)
    await asyncio.wait_for(conn.connect(), timeout=20)
    print("[OK] WebRTC connected")

    def low_cb(msg):
        data = msg.get("data", {})
        print("[LOW_STATE]", list(data.keys())[:8])

    def sport_cb(msg):
        data = msg.get("data", {})
        print("[SPORT_MOD_STATE]", list(data.keys())[:8])

    def lf_sport_cb(msg):
        data = msg.get("data", {})
        print("[LF_SPORT_MOD_STATE]", list(data.keys())[:8])

    conn.datachannel.pub_sub.subscribe(RTC_TOPIC["LOW_STATE"], low_cb)
    conn.datachannel.pub_sub.subscribe(RTC_TOPIC["SPORT_MOD_STATE"], sport_cb)
    conn.datachannel.pub_sub.subscribe(RTC_TOPIC["LF_SPORT_MOD_STATE"], lf_sport_cb)

    # 查询当前运动模式
    resp = await request(conn, RTC_TOPIC["MOTION_SWITCHER"], 1001)
    print("[MOTION_SWITCHER]")
    print(json.dumps(resp, indent=2, ensure_ascii=False))

    # 先发一个最安全的停止命令
    stop_resp = await request(conn, RTC_TOPIC["SPORT_MOD"], SPORT_CMD["StopMove"])
    print("[STOP_MOVE]")
    print(json.dumps(stop_resp, indent=2, ensure_ascii=False))

    # 观察 15 秒
    await asyncio.sleep(15)

    print("[INFO] disconnecting...")
    await conn.disconnect()

if __name__ == "__main__":
    asyncio.run(main())