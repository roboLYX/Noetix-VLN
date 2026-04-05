import asyncio
import json
from unitree_webrtc_connect import (
    UnitreeWebRTCConnection,
    WebRTCConnectionMethod,
    RTC_TOPIC,
    SPORT_CMD,
)

def short_sport_state(msg):
    data = msg.get("data", {})
    out = {
        "mode": data.get("mode"),
        "error_code": data.get("error_code"),
        "progress": data.get("progress"),
        "gait_type": data.get("gait_type"),
        "position": data.get("position"),
    }
    print("[LF_SPORT_MOD_STATE]", json.dumps(out, ensure_ascii=False))

def short_low_state(msg):
    data = msg.get("data", {})
    out = {
        "power_v": data.get("power_v"),
        "foot_force": data.get("foot_force"),
    }
    print("[LOW_STATE]", json.dumps(out, ensure_ascii=False))

async def request(conn, topic, api_id, parameter=None):
    payload = {"api_id": api_id}
    if parameter is not None:
        payload["parameter"] = parameter
    resp = await conn.datachannel.pub_sub.publish_request_new(topic, payload)
    return resp

async def stop(conn):
    resp = await request(conn, RTC_TOPIC["SPORT_MOD"], SPORT_CMD["StopMove"])
    print("[STOP]", json.dumps(resp, indent=2, ensure_ascii=False))
    await asyncio.sleep(0.8)
    return resp

async def move(conn, name, x=0.0, y=0.0, z=0.0, duration=1.0):
    print(f"[TEST] {name} | x={x}, y={y}, z={z}, duration={duration}")
    resp = await request(
        conn,
        RTC_TOPIC["SPORT_MOD"],
        SPORT_CMD["Move"],
        {"x": x, "y": y, "z": z}
    )
    print("[MOVE]", json.dumps(resp, indent=2, ensure_ascii=False))
    await asyncio.sleep(duration)
    await stop(conn)
    await asyncio.sleep(2.0)

async def main():
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP)
    await asyncio.wait_for(conn.connect(), timeout=20)
    print("[OK] connected")

    conn.datachannel.pub_sub.subscribe(RTC_TOPIC["LOW_STATE"], short_low_state)
    conn.datachannel.pub_sub.subscribe(RTC_TOPIC["LF_SPORT_MOD_STATE"], short_sport_state)

    resp = await request(conn, RTC_TOPIC["MOTION_SWITCHER"], 1001)
    print("[MOTION_SWITCHER]", json.dumps(resp, indent=2, ensure_ascii=False))

    print("[INFO] wait 3s before motion...")
    await asyncio.sleep(3)

    # 1) 先做明显的原地左转
    await move(conn, "yaw left strong", x=0.0, y=0.0, z=0.40, duration=1.5)

    # 2) 再做明显的原地右转
    await move(conn, "yaw right strong", x=0.0, y=0.0, z=-0.40, duration=1.5)

    # 3) 再做前进
    await move(conn, "forward medium", x=0.25, y=0.0, z=0.0, duration=1.2)

    # 4) 再做后退
    await move(conn, "backward medium", x=-0.25, y=0.0, z=0.0, duration=1.2)

    # 5) 最后再试横移
    await move(conn, "left strafe test", x=0.0, y=0.18, z=0.0, duration=1.2)
    await move(conn, "right strafe test", x=0.0, y=-0.18, z=0.0, duration=1.2)

    print("[INFO] final stop")
    await stop(conn)

    print("[INFO] disconnecting...")
    await conn.disconnect()

if __name__ == "__main__":
    asyncio.run(main())