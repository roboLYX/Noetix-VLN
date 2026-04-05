import asyncio
import logging
from unitree_webrtc_connect import (
    UnitreeWebRTCConnection,
    WebRTCConnectionMethod,
    RTC_TOPIC,
    SPORT_CMD,
)

# 启用日志查看详细信息
logging.basicConfig(level=logging.DEBUG)


async def main():
    print("Connecting to robot in AP mode...")
    print("Make sure you're connected to Go2's WiFi hotspot!")
    print("Go2 AP IP should be: 192.168.12.1")

    # AP 模式连接（自动使用 192.168.12.1）
    conn = UnitreeWebRTCConnection(
        WebRTCConnectionMethod.LocalAP
    )

    # 建立 WebRTC 连接
    await conn.connect()

    # 等待 data channel 完全打开
    await conn.datachannel.wait_datachannel_open()
    print("Data channel opened.")

    # 可选：先停一下当前运动
    try:
        stop_resp = await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"],
            {
                "api_id": SPORT_CMD["StopMove"]
            }
        )
        print("StopMove response:", stop_resp)
        await asyncio.sleep(0.5)
    except Exception as e:
        print("StopMove failed, continue anyway:", e)

    # 发送站立指令
    stand_resp = await conn.datachannel.pub_sub.publish_request_new(
        RTC_TOPIC["SPORT_MOD"],
        {
            "api_id": SPORT_CMD["StandUp"]
        }
    )
    print("StandUp response:", stand_resp)

    # 给机器人一点执行时间
    await asyncio.sleep(3)
    print("Done.")


if __name__ == "__main__":
    asyncio.run(main())